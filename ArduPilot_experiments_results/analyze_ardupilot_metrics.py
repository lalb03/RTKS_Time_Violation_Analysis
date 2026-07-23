#!/usr/bin/env python3
"""
ArduPilot TimeTrap CSV analysis.

The script:
1. reads every experiment CSV in an input directory;
2. keeps the three runs separate;
3. calculates the agreed metrics for each run;
4. calculates mean and sample standard deviation across runs;
5. optionally creates simple trace plots aligned to the transition
   NAV_INDEX 2 -> 3.

Expected filenames include the experiment and run number, for example:
    E0_exp_log_run1.csv
    E1_exp_log_run2.csv
    E2_exp_log_run3.csv
    E3-30_exp_log_run1.csv
    E3-50_exp_log_run2.csv
    E4_exp_log_run3.csv

The prefixes A-E0, A-E1, ... are also accepted.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


REQUIRED_COLUMNS = {
    "SIM_TIME_US",
    "WALL_TIME_US",
    "CONTROL_DT_WALL_US",
    "STATE_AGE_AT_WPNAV_US",
    "WPNAV_ELAPSED_WALL_US",
    "PRODUCER_SIM_US",
    "AGE_SIM_US",
    "NAV_INDEX",
    "WP_DISTANCE_CM",
    "REF_VEL_N",
    "SIM_VEL_N",
    "REF_VEL_E",
    "SIM_VEL_E",
}

OUTPUT_FILENAMES = {
    "per_run_metrics.csv",
    "configuration_summary_long.csv",
    "configuration_summary_formatted.csv",
    "run_quality.csv",
}


@dataclass(frozen=True)
class RunId:
    configuration: str
    run: int


def parse_run_id(path: Path) -> RunId:
    """Extract configuration and run number from a filename."""
    stem = path.stem.upper().replace("_", "-")

    run_match = re.search(r"RUN-?(\d+)", stem)
    if run_match is None:
        raise ValueError(
            f"Cannot find a run number in '{path.name}'. "
            "Use a name such as E0_exp_log_run1.csv."
        )
    run = int(run_match.group(1))

    config_patterns = [
        (r"(?:A-)?E3-?30", "A-E3-30"),
        (r"(?:A-)?E3-?50", "A-E3-50"),
        (r"(?:A-)?E0", "A-E0"),
        (r"(?:A-)?E1", "A-E1"),
        (r"(?:A-)?E2", "A-E2"),
        (r"(?:A-)?E4", "A-E4"),
    ]
    for pattern, normalized in config_patterns:
        if re.search(pattern, stem):
            return RunId(normalized, run)

    raise ValueError(
        f"Cannot determine the experiment from '{path.name}'. "
        "Use E0, E1, E2, E3-30, E3-50, or E4 in the filename."
    )


def read_numeric_csv(path: Path) -> tuple[pd.DataFrame, dict[str, object]]:
    """
    Read the CSV without deleting rows merely because one unused field is empty.

    We first use pandas' faster C parser. If the file is truncated, contains a
    malformed final record, or the C parser cannot read the source, we retry
    with the more tolerant Python parser.

    Values that cannot be parsed are converted to NaN. Metrics then discard
    only rows missing the columns required by that specific calculation.
    """
    parser_used = "c"

    try:
        raw = pd.read_csv(
            path,
            engine="c",
            low_memory=False,
            on_bad_lines="warn",
        )
    except (OSError, ValueError, pd.errors.ParserError) as first_error:
        parser_used = "python"
        print(
            f"WARNING: {path.name}: C parser failed ({first_error}). "
            "Retrying with the Python parser.",
            file=sys.stderr,
        )

        # Open explicitly as text so that decoding problems do not abort the
        # whole experiment analysis. Invalid bytes are replaced and reported
        # through the quality table via the selected parser.
        with path.open(
            "r",
            encoding="utf-8",
            errors="replace",
            newline="",
        ) as handle:
            raw = pd.read_csv(
                handle,
                engine="python",
                on_bad_lines="warn",
            )

    missing_columns = sorted(REQUIRED_COLUMNS.difference(raw.columns))
    if missing_columns:
        raise ValueError(
            f"{path.name} is missing required columns: {', '.join(missing_columns)}"
        )

    df = raw.copy()
    for column in df.columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")

    # Check timestamp monotonicity only on complete records. ArduPilot may
    # leave one partially written final CSV line when SITL terminates. Such a
    # line can contain a truncated timestamp (for example 2 or 96) and would
    # otherwise create a false non-monotonicity warning.
    complete_time_rows = df.dropna(
        subset=["NAV_INDEX", "WALL_TIME_US", "SIM_TIME_US"]
    )
    valid_wall = complete_time_rows["WALL_TIME_US"]
    valid_sim = complete_time_rows["SIM_TIME_US"]

    quality = {
        "csv_parser": parser_used,
        "rows_read": int(len(df)),
        "complete_time_rows": int(len(complete_time_rows)),
        "rows_with_missing_nav_or_time": int(
            df[["NAV_INDEX", "WALL_TIME_US", "SIM_TIME_US"]].isna().any(axis=1).sum()
        ),
        "wall_time_monotonic": bool(valid_wall.is_monotonic_increasing),
        "sim_time_monotonic": bool(valid_sim.is_monotonic_increasing),
    }
    return df, quality


def first_position(mask: pd.Series) -> int | None:
    positions = np.flatnonzero(mask.fillna(False).to_numpy())
    return int(positions[0]) if len(positions) else None


def mission_window(df: pd.DataFrame) -> tuple[pd.DataFrame, bool, int, int]:
    """
    Start: first row with NAV_INDEX == 1.
    End: first later row with NAV_INDEX == 4 and WP_DISTANCE_CM <= 100 cm.

    If the mission does not complete, retain the available trace until the
    final row and mark the run as incomplete.
    """
    start = first_position(df["NAV_INDEX"].eq(1))
    if start is None:
        raise ValueError("Cannot find mission start: NAV_INDEX == 1 is absent.")

    end_mask = (
        df["NAV_INDEX"].eq(4)
        & df["WP_DISTANCE_CM"].le(100)
    )
    end_positions = np.flatnonzero(end_mask.fillna(False).to_numpy())
    end_positions = end_positions[end_positions > start]

    completed = len(end_positions) > 0
    if completed:
        end = int(end_positions[0])
    else:
        valid_positions = np.flatnonzero(
            df[["WALL_TIME_US", "SIM_TIME_US"]].notna().all(axis=1).to_numpy()
        )
        later = valid_positions[valid_positions >= start]
        if len(later) == 0:
            raise ValueError("No valid timestamps after mission start.")
        end = int(later[-1])

    return df.iloc[start : end + 1].copy(), completed, start, end


def transition_2_to_3_position(df: pd.DataFrame) -> int:
    nav = df["NAV_INDEX"]
    transition = nav.eq(3) & nav.shift(1).eq(2)
    position = first_position(transition)
    if position is None:
        raise ValueError("Cannot find the NAV_INDEX transition 2 -> 3.")
    return position


def contiguous_nav3_window(df: pd.DataFrame, start: int) -> pd.DataFrame:
    end = start
    while end + 1 < len(df) and df["NAV_INDEX"].iloc[end + 1] == 3:
        end += 1
    return df.iloc[start : end + 1].copy()


def post_200_window(df: pd.DataFrame, start: int) -> pd.DataFrame:
    return df.iloc[start : min(start + 200, len(df))].copy()


def post_6s_window(df: pd.DataFrame, start: int) -> pd.DataFrame:
    start_wall = df["WALL_TIME_US"].iloc[start]
    if pd.isna(start_wall):
        raise ValueError("Missing WALL_TIME_US at transition 2 -> 3.")
    end_wall = start_wall + 6_000_000
    return df.iloc[start:].loc[
        df.iloc[start:]["WALL_TIME_US"].le(end_wall)
    ].copy()


def valid_series(window: pd.DataFrame, column: str) -> pd.Series:
    return pd.to_numeric(window[column], errors="coerce").dropna()


def duration_seconds(window: pd.DataFrame, column: str) -> float:
    values = valid_series(window, column)
    if len(values) < 2:
        return math.nan
    return float((values.iloc[-1] - values.iloc[0]) / 1_000_000.0)


def velocity_error(window: pd.DataFrame) -> pd.Series:
    cols = ["REF_VEL_N", "SIM_VEL_N", "REF_VEL_E", "SIM_VEL_E"]
    valid = window.dropna(subset=cols)
    if valid.empty:
        return pd.Series(dtype=float)

    error_n = valid["REF_VEL_N"] - valid["SIM_VEL_N"]
    error_e = valid["REF_VEL_E"] - valid["SIM_VEL_E"]
    return np.sqrt(error_n.pow(2) + error_e.pow(2))


def velocity_metrics(window: pd.DataFrame, prefix: str) -> dict[str, float]:
    error = velocity_error(window)
    if error.empty:
        return {
            f"{prefix}_velocity_rmse_cm_s": math.nan,
            f"{prefix}_velocity_max_error_cm_s": math.nan,
        }

    # RMSE_v = sqrt(mean(e_v(k)^2))
    return {
        f"{prefix}_velocity_rmse_cm_s": float(np.sqrt(np.mean(np.square(error)))),
        f"{prefix}_velocity_max_error_cm_s": float(error.max()),
    }


def mean_max_ms(
    window: pd.DataFrame,
    column: str,
    output_name: str,
    validity_mask: pd.Series | None = None,
) -> dict[str, float]:
    if validity_mask is None:
        values = valid_series(window, column)
    else:
        mask = validity_mask.reindex(window.index, fill_value=False)
        values = pd.to_numeric(window.loc[mask, column], errors="coerce").dropna()

    if values.empty:
        return {
            f"{output_name}_mean_ms": math.nan,
            f"{output_name}_max_ms": math.nan,
        }

    values_ms = values / 1000.0
    return {
        f"{output_name}_mean_ms": float(values_ms.mean()),
        f"{output_name}_max_ms": float(values_ms.max()),
    }


def timing_metrics(window: pd.DataFrame, prefix: str) -> dict[str, float]:
    result: dict[str, float] = {}

    result.update(
        mean_max_ms(
            window,
            "CONTROL_DT_WALL_US",
            f"{prefix}_control_interval",
        )
    )
    result.update(
        mean_max_ms(
            window,
            "WPNAV_ELAPSED_WALL_US",
            f"{prefix}_wpnav_elapsed",
        )
    )
    result.update(
        mean_max_ms(
            window,
            "STATE_AGE_AT_WPNAV_US",
            f"{prefix}_state_temporal_displacement",
        )
    )

    # Producer fields are meaningful only when a separated sensor thread
    # publishes a non-zero producer timestamp.
    producer_valid = window["PRODUCER_SIM_US"].gt(0)
    result.update(
        mean_max_ms(
            window,
            "AGE_SIM_US",
            f"{prefix}_producer_age",
            validity_mask=producer_valid,
        )
    )

    if "INS_UPDATED_DURING_WPNAV" in window.columns:
        unchanged = (
            pd.to_numeric(
                window["INS_UPDATED_DURING_WPNAV"], errors="coerce"
            )
            .dropna()
            .eq(0)
        )
        result[f"{prefix}_ins_unchanged_percent"] = (
            float(100.0 * unchanged.mean()) if len(unchanged) else math.nan
        )

    return result


def process_run(path: Path) -> tuple[dict[str, object], dict[str, object], pd.DataFrame]:
    run_id = parse_run_id(path)
    df, quality = read_numeric_csv(path)

    navigation, completed, nav_start, nav_end = mission_window(df)
    transition = transition_2_to_3_position(df)
    nav3 = contiguous_nav3_window(df, transition)
    post200 = post_200_window(df, transition)
    post6s = post_6s_window(df, transition)

    metrics: dict[str, object] = {
        "configuration": run_id.configuration,
        "run": run_id.run,
        "source_file": path.name,
        "mission_completed": int(completed),
        "navigation_samples": int(len(navigation)),
        "nav3_samples": int(len(nav3)),
        "post200_samples": int(len(post200)),
        "post6s_samples": int(len(post6s)),
        "navigation_duration_wall_s": duration_seconds(
            navigation, "WALL_TIME_US"
        ),
        "navigation_duration_sim_s": duration_seconds(
            navigation, "SIM_TIME_US"
        ),
    }

    metrics.update(velocity_metrics(navigation, "navigation"))
    metrics.update(velocity_metrics(nav3, "nav3"))
    metrics.update(velocity_metrics(post200, "post200"))
    metrics.update(velocity_metrics(post6s, "post6s"))

    metrics.update(timing_metrics(post200, "post200"))
    metrics.update(timing_metrics(post6s, "post6s"))

    quality_row = {
        "configuration": run_id.configuration,
        "run": run_id.run,
        "source_file": path.name,
        "mission_start_row": nav_start,
        "mission_end_row": nav_end,
        "transition_2_to_3_row": transition,
        **quality,
    }

    # Return the complete trace with derived velocity error for optional plots.
    trace = df.copy()
    trace["VELOCITY_ERROR_CM_S"] = np.nan
    err = velocity_error(df)
    trace.loc[err.index, "VELOCITY_ERROR_CM_S"] = err

    return metrics, quality_row, trace


def summarize(per_run: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    id_columns = {
        "configuration",
        "run",
        "source_file",
    }
    numeric_columns = [
        c for c in per_run.columns
        if c not in id_columns and pd.api.types.is_numeric_dtype(per_run[c])
    ]

    long_rows: list[dict[str, object]] = []
    formatted_rows: list[dict[str, object]] = []

    for configuration, group in per_run.groupby("configuration", sort=True):
        formatted: dict[str, object] = {
            "configuration": configuration,
            "run_count": int(group["run"].nunique()),
            "completed_runs": int(group["mission_completed"].sum()),
        }

        for column in numeric_columns:
            if column in {
                "mission_completed",
                "navigation_samples",
                "nav3_samples",
                "post200_samples",
                "post6s_samples",
            }:
                continue

            values = pd.to_numeric(group[column], errors="coerce").dropna()
            if values.empty:
                mean = std = minimum = maximum = math.nan
                count = 0
                formatted[column] = "N/A"
            else:
                count = int(len(values))
                mean = float(values.mean())
                std = float(values.std(ddof=1)) if len(values) > 1 else math.nan
                minimum = float(values.min())
                maximum = float(values.max())

                if math.isnan(std):
                    formatted[column] = f"{mean:.3f}"
                else:
                    formatted[column] = f"{mean:.3f} ± {std:.3f}"

            long_rows.append(
                {
                    "configuration": configuration,
                    "metric": column,
                    "run_count": count,
                    "mean": mean,
                    "sample_std": std,
                    "min": minimum,
                    "max": maximum,
                }
            )

        formatted_rows.append(formatted)

    return pd.DataFrame(long_rows), pd.DataFrame(formatted_rows)


def create_trace_plots(
    traces: list[tuple[RunId, pd.DataFrame]],
    output_dir: Path,
    trace_seconds: float,
) -> None:
    try:
        import matplotlib

        # Use a non-interactive backend so plots also work in a VM terminal
        # without an active graphical display.
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Plots requested but matplotlib is not installed."
        ) from exc

    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    grouped: dict[str, list[tuple[int, pd.DataFrame]]] = {}
    for run_id, trace in traces:
        grouped.setdefault(run_id.configuration, []).append((run_id.run, trace))

    safe_name = lambda value: value.replace("A-", "").replace("-", "_")

    def plot_xy(x_values: pd.Series, y_values: pd.Series, label: str) -> bool:
        """
        Convert pandas Series to NumPy arrays before calling matplotlib.

        This avoids the pandas 2.x / older matplotlib incompatibility that
        raises: "Multi-dimensional indexing is no longer supported".
        """
        x_array = pd.to_numeric(x_values, errors="coerce").to_numpy(dtype=float)
        y_array = pd.to_numeric(y_values, errors="coerce").to_numpy(dtype=float)

        finite = np.isfinite(x_array) & np.isfinite(y_array)
        if not finite.any():
            return False

        plt.plot(
            x_array[finite],
            y_array[finite],
            label=label,
            linewidth=1,
        )
        return True

    for configuration, runs in sorted(grouped.items()):
        # Velocity error after the 2 -> 3 transition
        plt.figure(figsize=(8, 4.5))
        plotted = False
        for run, trace in sorted(runs):
            try:
                transition = transition_2_to_3_position(trace)
            except ValueError:
                continue
            start_wall = trace["WALL_TIME_US"].iloc[transition]
            segment = trace.iloc[transition:].copy()
            segment = segment[
                segment["WALL_TIME_US"].le(start_wall + trace_seconds * 1_000_000)
            ]
            x = (segment["WALL_TIME_US"] - start_wall) / 1_000_000
            y = segment["VELOCITY_ERROR_CM_S"]
            plotted = plot_xy(x, y, label=f"run {run}") or plotted

        if plotted:
            plt.xlabel("Time after NAV_INDEX 2→3 transition (s)")
            plt.ylabel("Horizontal velocity error (cm/s)")
            plt.title(f"{configuration}: velocity error after transition")
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(
                plot_dir / f"{safe_name(configuration)}_velocity_error.png",
                dpi=160,
            )
        plt.close()

        # State temporal displacement after transition
        plt.figure(figsize=(8, 4.5))
        plotted = False
        for run, trace in sorted(runs):
            try:
                transition = transition_2_to_3_position(trace)
            except ValueError:
                continue
            start_wall = trace["WALL_TIME_US"].iloc[transition]
            segment = trace.iloc[transition:].copy()
            segment = segment[
                segment["WALL_TIME_US"].le(start_wall + trace_seconds * 1_000_000)
            ]
            x = (segment["WALL_TIME_US"] - start_wall) / 1_000_000
            y = segment["STATE_AGE_AT_WPNAV_US"] / 1000
            plotted = plot_xy(x, y, label=f"run {run}") or plotted

        if plotted:
            plt.xlabel("Time after NAV_INDEX 2→3 transition (s)")
            plt.ylabel("Navigation-state temporal displacement (ms)")
            plt.title(f"{configuration}: state age after transition")
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(
                plot_dir / f"{safe_name(configuration)}_state_age.png",
                dpi=160,
            )
        plt.close()

        # Producer age only for configurations with a separated sensor thread
        if any(trace["PRODUCER_SIM_US"].gt(0).any() for _, trace in runs):
            plt.figure(figsize=(8, 4.5))
            plotted = False
            for run, trace in sorted(runs):
                try:
                    transition = transition_2_to_3_position(trace)
                except ValueError:
                    continue
                start_wall = trace["WALL_TIME_US"].iloc[transition]
                segment = trace.iloc[transition:].copy()
                segment = segment[
                    segment["WALL_TIME_US"].le(
                        start_wall + trace_seconds * 1_000_000
                    )
                ]
                segment = segment[segment["PRODUCER_SIM_US"].gt(0)]
                x = (segment["WALL_TIME_US"] - start_wall) / 1_000_000
                y = segment["AGE_SIM_US"] / 1000
                plotted = plot_xy(x, y, label=f"run {run}") or plotted

            if plotted:
                plt.xlabel("Time after NAV_INDEX 2→3 transition (s)")
                plt.ylabel("Latest sensor-producer age (ms, simulation time)")
                plt.title(f"{configuration}: producer age after transition")
                plt.legend()
                plt.grid(True, alpha=0.3)
                plt.tight_layout()
                plt.savefig(
                    plot_dir / f"{safe_name(configuration)}_producer_age.png",
                    dpi=160,
                )
            plt.close()


def find_input_csvs(input_dir: Path, pattern: str) -> list[Path]:
    candidates = sorted(input_dir.rglob(pattern))
    return [
        path for path in candidates
        if path.name not in OUTPUT_FILENAMES
        and "summary" not in path.name.lower()
        and path.is_file()
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calculate the agreed ArduPilot TimeTrap metrics."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        required=True,
        help="Directory containing the experiment CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory in which results will be written.",
    )
    parser.add_argument(
        "--pattern",
        default="*.csv",
        help="Recursive CSV filename pattern (default: *.csv).",
    )
    parser.add_argument(
        "--plots",
        action="store_true",
        help="Create trace plots aligned to the NAV_INDEX 2->3 transition.",
    )
    parser.add_argument(
        "--trace-seconds",
        type=float,
        default=12.0,
        help="Seconds shown after the transition in trace plots (default: 12).",
    )
    args = parser.parse_args()

    if not args.input_dir.exists():
        print(f"ERROR: input directory does not exist: {args.input_dir}", file=sys.stderr)
        return 2

    csv_files = find_input_csvs(args.input_dir, args.pattern)
    if not csv_files:
        print("ERROR: no input CSV files found.", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)

    metric_rows: list[dict[str, object]] = []
    quality_rows: list[dict[str, object]] = []
    traces: list[tuple[RunId, pd.DataFrame]] = []
    failures: list[str] = []

    for path in csv_files:
        try:
            metrics, quality, trace = process_run(path)
            run_id = parse_run_id(path)
            metric_rows.append(metrics)
            quality_rows.append(quality)
            traces.append((run_id, trace))
            print(f"Processed {path.name}")
        except Exception as exc:
            failures.append(f"{path.name}: {exc}")
            print(f"WARNING: skipped {path.name}: {exc}", file=sys.stderr)

    if not metric_rows:
        print("ERROR: no valid experiment CSV was processed.", file=sys.stderr)
        return 1

    per_run = (
        pd.DataFrame(metric_rows)
        .sort_values(["configuration", "run"])
        .reset_index(drop=True)
    )
    quality = (
        pd.DataFrame(quality_rows)
        .sort_values(["configuration", "run"])
        .reset_index(drop=True)
    )
    summary_long, summary_formatted = summarize(per_run)

    per_run.to_csv(args.output_dir / "per_run_metrics.csv", index=False)
    quality.to_csv(args.output_dir / "run_quality.csv", index=False)
    summary_long.to_csv(
        args.output_dir / "configuration_summary_long.csv", index=False
    )
    summary_formatted.to_csv(
        args.output_dir / "configuration_summary_formatted.csv", index=False
    )

    if failures:
        (args.output_dir / "skipped_files.txt").write_text(
            "\n".join(failures) + "\n",
            encoding="utf-8",
        )

    if args.plots:
        create_trace_plots(traces, args.output_dir, args.trace_seconds)

    print()
    print(f"Processed runs: {len(per_run)}")
    print(f"Configurations: {', '.join(sorted(per_run['configuration'].unique()))}")
    print(f"Results directory: {args.output_dir}")

    # Warn when a configuration does not contain exactly three runs.
    counts = per_run.groupby("configuration")["run"].nunique()
    for configuration, count in counts.items():
        if count != 3:
            print(
                f"WARNING: {configuration} contains {count} run(s), expected 3.",
                file=sys.stderr,
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
