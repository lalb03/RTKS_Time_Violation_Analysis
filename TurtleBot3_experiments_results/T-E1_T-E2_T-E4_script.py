#!/usr/bin/env python3
"""
Rebuild a per-run master CSV for the TurtleBot3 experiments T-E1, T-E2,
and T-E4 from the raw ROS1 bag files and source-level timing CSV files in
results-per-injection-point.zip.

The script is designed for the exact archive layout used by the RTKS project,
but it also accepts the extracted directory.

Full run (requires ROS Noetic's Python rosbag module):

    source /opt/ros/noetic/setup.bash
    python3 build_la_all_runs_master_v6.py results-per-injection-point.zip \
        --output-dir analysis_output

Internal-timing-only test (does not require ROS):

    python3 build_la_all_runs_master_v6.py results-per-injection-point.zip \
        --output-dir analysis_output_internal --skip-bags

Outputs:
    all_runs_master.csv
        One row per run. This is the main file intended for analysis.
    condition_summary.csv
        Aggregations across runs for each experiment and delay condition.
    report_relevant_summary.csv
        Compact aggregation using the conventions used in the report:
        internal metrics are means across run-level means, while mission and
        ROS-bag metrics are mostly medians across runs.
    data_quality.csv
        Per-run parsing counts and warnings.
    analysis_warnings.txt
        Human-readable warnings.

Important experiment-specific conventions
-----------------------------------------
T-E1 (AMCL pose publication):
    The raw CSV has no header. Its first row is a legacy control/summary record
    with an explicit displacement in the third field; the following rows are
    callback-event records whose third field is the hook name
    ``amcl_pose_publish``. The first row duplicates the first callback event
    (same scan timestamp and delay fields, with publication time differing by
    at most a few milliseconds), so it is used only as a consistency check and
    excluded from statistical aggregation. Pose temporal displacement is
    reconstructed for every callback event as
    (publish_time_sec - scan_time_sec) * 1000.

T-E2 (AMCL TF publication):
    Rows whose requested_delay_us does not match the directory condition are
    excluded. This is necessary for te2_200ms_run-01, which contains records
    from other delay settings. The exclusion is recorded in data_quality.csv.

T-E4 (delay before local-planner execution):
    ROS bags are analysed in two passes: the first identifies the mission
    window from move_base status/result messages, and the second collects all
    data-topic messages whose timestamps lie inside that window. This avoids
    losing /cmd_vel messages only because they occur earlier in bag record
    order than the ACTIVE status message. Source-level timing rows are checked
    for numeric completeness, intra-row timestamp ordering, consistency between
    timestamp differences and the reported latency fields, and strictly
    increasing command-publication timestamps. A row failing one of these
    checks is excluded from every T-E4 statistic, rather than only from the
    interval calculation. Command-publication intervals are then differences
    between consecutive valid command_publish_time_sec values. The diagnostic nominal-period
    exceedance ratio is the fraction of command_publish_latency_ms values
    strictly greater than the nominal 100-ms controller period. It is a
    diagnostic indicator, not evidence of a formally specified deadline miss.

The script never fills missing measurements from values copied out of the
report. Every populated numeric field is computed from a raw bag or timing CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import statistics
import sys
import tempfile
import zipfile
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


NOMINAL_CONTROLLER_PERIOD_MS = 100.0
STARTUP_WINDOW_S = 5.0
LINEAR_ZERO_THRESHOLD = 0.01
ANGULAR_ZERO_THRESHOLD = 0.05
ODOM_JUMP_THRESHOLD_M = 0.5
REQUESTED_DELAY_TOLERANCE_US = 0.5
TIMESTAMP_ORDER_TOLERANCE_S = 1e-9
TIMESTAMP_METRIC_TOLERANCE_MS = 0.01

STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}

BAG_TOPICS = [
    "/move_base_simple/goal",
    "/move_base/result",
    "/move_base/status",
    "/odom",
    "/cmd_vel",
    "/scan",
    "/amcl_pose",
]

EXPERIMENTS = {
    "T-E1": {
        "logical_name": "amcl_pose_publish",
        "injection_point": "AMCL pose publication before pose_pub_.publish()",
        "relative_root": Path("results"),
        "delays": [0, 20, 50, 100, 200, 500, 1000],
        "parser": "te1",
    },
    "T-E2": {
        "logical_name": "amcl_sendTransform",
        "injection_point": "AMCL TF publication before tfb_->sendTransform()",
        "relative_root": Path("results_delay_sendTransofrm") / "amcl_sendTransofrm",
        "delays": [0, 20, 50, 100, 200],
        "parser": "te2",
    },
    "T-E4": {
        "logical_name": "move_base_computeVelocityCommands",
        "injection_point": (
            "MoveBase::executeCycle() before "
            "tc_->computeVelocityCommands(cmd_vel)"
        ),
        "relative_root": (
            Path("results_move_base_computeVelocityCommands")
            / "move_base_computeVelocityCommands"
        ),
        "delays": [0, 20, 50, 100, 200, 500, 1000],
        "parser": "te4",
    },
}


class AnalysisError(RuntimeError):
    """Raised for an unrecoverable input or parsing problem."""


def finite_number(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def mean_or_none(values: Sequence[float]) -> Optional[float]:
    return statistics.mean(values) if values else None


def median_or_none(values: Sequence[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def stdev_or_none(values: Sequence[float]) -> Optional[float]:
    return statistics.stdev(values) if len(values) >= 2 else None


def min_or_none(values: Sequence[float]) -> Optional[float]:
    return min(values) if values else None


def max_or_none(values: Sequence[float]) -> Optional[float]:
    return max(values) if values else None


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """Linear percentile using position (n - 1) * fraction."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def numeric_values(values: Iterable[Any], allow_negative: bool = True) -> List[float]:
    output: List[float] = []
    for value in values:
        number = finite_number(value)
        if number is None:
            continue
        if not allow_negative and number < 0.0:
            continue
        output.append(number)
    return output


def summarise(values: Sequence[float], prefix: str) -> Dict[str, Any]:
    values = list(values)
    return {
        f"{prefix}_n": len(values),
        f"{prefix}_mean": mean_or_none(values),
        f"{prefix}_median": median_or_none(values),
        f"{prefix}_stdev": stdev_or_none(values),
        f"{prefix}_min": min_or_none(values),
        f"{prefix}_p95": percentile(values, 0.95),
        f"{prefix}_max": max_or_none(values),
    }


def fmt(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.9f}".rstrip("0").rstrip(".")
    return value


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: fmt(row.get(name)) for name in fieldnames})


def read_dict_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def locate_archive_root(candidate: Path) -> Path:
    """Locate the directory containing results/, results_delay_..., and results_move_...."""
    candidate = candidate.resolve()
    required = [
        Path("results"),
        Path("results_delay_sendTransofrm"),
        Path("results_move_base_computeVelocityCommands"),
    ]

    def is_root(path: Path) -> bool:
        return all((path / item).exists() for item in required)

    if is_root(candidate):
        return candidate

    direct = candidate / "results-per-injection-point"
    if is_root(direct):
        return direct

    matches = []
    if candidate.is_dir():
        for path in candidate.rglob("results-per-injection-point"):
            if path.is_dir() and is_root(path):
                matches.append(path)
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise AnalysisError(
            f"Could not locate the archive root below {candidate}. Expected "
            "results/, results_delay_sendTransofrm/, and "
            "results_move_base_computeVelocityCommands/."
        )
    raise AnalysisError(f"Multiple possible archive roots found: {matches}")


def prepare_input(input_path: Path):
    """Context manager yielding an extracted or existing archive root."""
    class PreparedInput:
        def __init__(self, source: Path):
            self.source = source
            self.temp: Optional[tempfile.TemporaryDirectory] = None
            self.root: Optional[Path] = None

        def __enter__(self) -> Path:
            if self.source.is_file() and self.source.suffix.lower() == ".zip":
                self.temp = tempfile.TemporaryDirectory(prefix="rtks_results_")
                with zipfile.ZipFile(str(self.source), "r") as archive:
                    archive.extractall(self.temp.name)
                self.root = locate_archive_root(Path(self.temp.name))
            elif self.source.is_dir():
                self.root = locate_archive_root(self.source)
            else:
                raise AnalysisError(
                    f"Input must be the results ZIP or an extracted directory: {self.source}"
                )
            return self.root

        def __exit__(self, exc_type, exc, tb) -> None:
            if self.temp is not None:
                self.temp.cleanup()

    return PreparedInput(input_path)


def condition_folder(delay_ms: int) -> str:
    return "baseline" if delay_ms == 0 else f"delay{delay_ms}"


def expected_bag_name(delay_ms: int, run_number: int) -> str:
    if delay_ms == 0:
        return f"baseline_run-{run_number}.bag"
    return f"delay{delay_ms}_run-{run_number}.bag"


def expected_timing_name(experiment: str, delay_ms: int, run_number: int) -> str:
    if experiment == "T-E1" and delay_ms == 0:
        return f"baseline_results-{run_number}.csv"
    if experiment == "T-E4" and delay_ms == 0:
        return f"baseline_results-{run_number}.csv"
    return f"delay_results-{run_number}.csv"


def discover_expected_runs(root: Path, warnings: List[str]) -> List[Dict[str, Any]]:
    runs: List[Dict[str, Any]] = []
    for experiment, spec in EXPERIMENTS.items():
        experiment_root = root / spec["relative_root"]
        if not experiment_root.exists():
            raise AnalysisError(f"Missing experiment directory: {experiment_root}")

        for delay_ms in spec["delays"]:
            folder = experiment_root / condition_folder(delay_ms)
            for run_number in range(1, 6):
                bag_path = folder / expected_bag_name(delay_ms, run_number)
                timing_path = folder / expected_timing_name(
                    experiment, delay_ms, run_number
                )
                if not bag_path.exists():
                    warnings.append(f"Missing expected bag: {bag_path}")
                if not timing_path.exists():
                    warnings.append(f"Missing expected timing CSV: {timing_path}")
                if not bag_path.exists() or not timing_path.exists():
                    continue

                run_id = (
                    f"{experiment.lower().replace('-', '')}_"
                    f"{delay_ms}ms_run-{run_number:02d}"
                )
                logical_folder = condition_folder(delay_ms)
                runs.append({
                    "experiment": experiment,
                    "run_id": run_id,
                    "delay_ms": delay_ms,
                    "run_number": run_number,
                    "injection_point": spec["injection_point"],
                    "logical_name": spec["logical_name"],
                    "parser": spec["parser"],
                    "bag_path": bag_path,
                    "timing_path": timing_path,
                    "bag_file": (
                        f"{spec['logical_name']}/{logical_folder}/{bag_path.name}"
                    ),
                    "timing_file": (
                        f"{spec['logical_name']}/{logical_folder}/{timing_path.name}"
                    ),
                    "source_bag_path": str(bag_path.relative_to(root)),
                    "source_timing_path": str(timing_path.relative_to(root)),
                })

    runs.sort(key=lambda row: (
        row["experiment"], int(row["delay_ms"]), int(row["run_number"])
    ))
    return runs


def path_length(points: Sequence[Tuple[float, float, float]]) -> float:
    total = 0.0
    for first, second in zip(points, points[1:]):
        step = math.hypot(second[1] - first[1], second[2] - first[2])
        if step < ODOM_JUMP_THRESHOLD_M:
            total += step
    return total


def count_angular_sign_changes(commands: Sequence[Tuple[float, float, float]]) -> int:
    signs: List[int] = []
    for _, _, angular in commands:
        if abs(angular) < ANGULAR_ZERO_THRESHOLD:
            continue
        signs.append(1 if angular > 0 else -1)
    return sum(1 for first, second in zip(signs, signs[1:]) if first != second)


def message_age_ms(record_time: float, header_stamp: Any) -> Optional[float]:
    try:
        stamp = header_stamp.to_sec()
    except Exception:
        return None
    if stamp <= 0.0:
        return None
    return max(0.0, (record_time - stamp) * 1000.0)


def analyse_bag(run: Dict[str, Any], rosbag_module: Any, warnings: List[str]) -> Dict[str, Any]:
    """Extract mission-level metrics from one ROS bag using two passes.

    Pass 1 identifies the mission start, active goal ID, terminal outcome, and
    analysis end. Pass 2 reopens the bag and collects every data-topic message
    whose timestamp lies in that mission window. This makes the result
    independent of the inter-topic record order inside the bag.
    """
    bag_path: Path = run["bag_path"]
    simple_goal_time: Optional[float] = None
    goal_time: Optional[float] = None
    active_goal_id: Optional[str] = None
    result_time: Optional[float] = None
    result_status = "NO_RESULT"
    result_status_text = ""
    result_topic_candidates: List[Tuple[float, str, int, str]] = []
    terminal_statuses = {2, 3, 4, 5, 8, 9}

    # Pass 1: identify the mission window and outcome.
    try:
        bag = rosbag_module.Bag(str(bag_path), "r")
    except Exception as exc:
        raise AnalysisError(f"Cannot open ROS bag {bag_path}: {exc}") from exc

    bag_start = bag.get_start_time()
    bag_end = bag.get_end_time()
    try:
        for topic, message, bag_time in bag.read_messages(
            topics=[
                "/move_base_simple/goal",
                "/move_base/status",
                "/move_base/result",
            ]
        ):
            current_time = bag_time.to_sec()

            if topic == "/move_base_simple/goal":
                if simple_goal_time is None:
                    simple_goal_time = current_time
                continue

            if topic == "/move_base/result":
                status = message.status
                result_topic_candidates.append((
                    current_time,
                    str(getattr(getattr(status, "goal_id", None), "id", "") or ""),
                    int(status.status),
                    str(getattr(status, "text", "") or ""),
                ))
                continue

            statuses = list(getattr(message, "status_list", []) or [])
            if goal_time is None:
                active_statuses = [
                    status for status in statuses if int(status.status) == 1
                ]
                if active_statuses:
                    active = active_statuses[-1]
                    goal_time = current_time
                    active_goal_id = str(
                        getattr(getattr(active, "goal_id", None), "id", "") or ""
                    )

            if goal_time is None:
                continue

            terminal_candidates = []
            for status in statuses:
                status_code = int(status.status)
                status_goal_id = str(
                    getattr(getattr(status, "goal_id", None), "id", "") or ""
                )
                same_goal = (
                    status_goal_id == active_goal_id
                    if active_goal_id and status_goal_id
                    else True
                )
                if same_goal and status_code in terminal_statuses:
                    terminal_candidates.append(status)

            if terminal_candidates:
                terminal = terminal_candidates[-1]
                result_time = current_time
                status_code = int(terminal.status)
                result_status = STATUS_NAMES.get(
                    status_code, f"UNKNOWN_{status_code}"
                )
                result_status_text = str(getattr(terminal, "text", "") or "")
                break
    finally:
        bag.close()

    # ACTIVE is preferred. The simple-goal timestamp is only a fallback for a
    # bag without an ACTIVE status record.
    if goal_time is None:
        goal_time = simple_goal_time

    # If no terminal status was observed, use a matching action-result record.
    if result_time is None and goal_time is not None:
        for candidate_time, candidate_id, status_code, status_text in result_topic_candidates:
            if candidate_time < goal_time or status_code not in terminal_statuses:
                continue
            same_goal = (
                candidate_id == active_goal_id
                if active_goal_id and candidate_id
                else True
            )
            if not same_goal:
                continue
            result_time = candidate_time
            result_status = STATUS_NAMES.get(
                status_code, f"UNKNOWN_{status_code}"
            )
            result_status_text = status_text
            break

    if goal_time is None:
        warnings.append(
            f"{run['run_id']}: no ACTIVE move_base goal found in "
            "/move_base/status or /move_base_simple/goal"
        )

    if result_time is None and goal_time is not None:
        warnings.append(
            f"{run['run_id']}: no terminal move_base result/status found; "
            "using bag end as observation end"
        )

    observation_end = result_time if result_time is not None else bag_end

    odom_points: List[Tuple[float, float, float]] = []
    cmd_commands: List[Tuple[float, float, float]] = []
    scan_ages_ms: List[float] = []
    amcl_ages_ms: List[float] = []
    minimum_scan_m: Optional[float] = None
    counts = defaultdict(int)

    # Pass 2: collect data by timestamp, not by when the ACTIVE status happens
    # to appear in the bag's inter-topic record order.
    if goal_time is not None:
        try:
            bag = rosbag_module.Bag(str(bag_path), "r")
        except Exception as exc:
            raise AnalysisError(f"Cannot reopen ROS bag {bag_path}: {exc}") from exc
        try:
            for topic, message, bag_time in bag.read_messages(
                topics=["/odom", "/cmd_vel", "/scan", "/amcl_pose"]
            ):
                current_time = bag_time.to_sec()
                if current_time < goal_time or current_time > observation_end:
                    continue

                counts[topic] += 1

                if topic == "/odom":
                    position = message.pose.pose.position
                    odom_points.append((
                        current_time,
                        float(position.x),
                        float(position.y),
                    ))

                elif topic == "/cmd_vel":
                    cmd_commands.append((
                        current_time,
                        float(message.linear.x),
                        float(message.angular.z),
                    ))

                elif topic == "/scan":
                    valid_ranges = [
                        float(value)
                        for value in message.ranges
                        if math.isfinite(value)
                        and message.range_min <= value <= message.range_max
                    ]
                    if valid_ranges:
                        current_minimum = min(valid_ranges)
                        minimum_scan_m = (
                            current_minimum
                            if minimum_scan_m is None
                            else min(minimum_scan_m, current_minimum)
                        )
                    age = message_age_ms(current_time, message.header.stamp)
                    if age is not None:
                        scan_ages_ms.append(age)

                elif topic == "/amcl_pose":
                    age = message_age_ms(current_time, message.header.stamp)
                    if age is not None:
                        amcl_ages_ms.append(age)
        finally:
            bag.close()

    mission_duration_s = (
        result_time - goal_time
        if (
            goal_time is not None
            and result_time is not None
            and result_status == "SUCCEEDED"
        )
        else None
    )
    observation_duration_s = (
        observation_end - goal_time
        if goal_time is not None and result_status != "SUCCEEDED"
        else None
    )

    linear_abs = [abs(item[1]) for item in cmd_commands]
    angular_abs = [abs(item[2]) for item in cmd_commands]
    stop_count = sum(
        1
        for _, linear, angular in cmd_commands
        if abs(linear) < LINEAR_ZERO_THRESHOLD
        and abs(angular) < ANGULAR_ZERO_THRESHOLD
    )
    cmd_gaps_ms = [
        (second[0] - first[0]) * 1000.0
        for first, second in zip(cmd_commands, cmd_commands[1:])
        if second[0] >= first[0]
    ]

    startup_limit = goal_time + STARTUP_WINDOW_S if goal_time is not None else None
    startup_points = [
        point
        for point in odom_points
        if startup_limit is not None and point[0] <= startup_limit
    ]
    startup_commands = [
        command
        for command in cmd_commands
        if startup_limit is not None and command[0] <= startup_limit
    ]
    startup_path_m = path_length(startup_points)
    if len(startup_points) >= 2:
        startup_net_m = math.hypot(
            startup_points[-1][1] - startup_points[0][1],
            startup_points[-1][2] - startup_points[0][2],
        )
    else:
        startup_net_m = None
    startup_efficiency = (
        startup_net_m / startup_path_m
        if startup_net_m is not None and startup_path_m > 0.0
        else None
    )
    reverse_count = sum(
        1
        for _, linear, _ in startup_commands
        if linear < -LINEAR_ZERO_THRESHOLD
    )
    first_motion_time_s: Optional[float] = None
    if goal_time is not None:
        for command_time, linear, angular in cmd_commands:
            if (
                abs(linear) >= LINEAR_ZERO_THRESHOLD
                or abs(angular) >= ANGULAR_ZERO_THRESHOLD
            ):
                first_motion_time_s = command_time - goal_time
                break

    return {
        "bag_duration_s": bag_end - bag_start,
        "goal_ros_time_s": goal_time,
        "result_ros_time_s": result_time,
        "mission_duration_s": mission_duration_s,
        "observation_duration_s": observation_duration_s,
        "result_from_bag": result_status,
        "result_status_text": result_status_text,
        "recovery_indicated_by_status": (
            "yes" if re.search(r"recover", result_status_text, re.I) else "no"
        ),
        "odom_path_length_m": path_length(odom_points),
        "minimum_scan_m": minimum_scan_m,
        "mean_abs_linear_cmd_m_s": mean_or_none(linear_abs),
        "max_abs_linear_cmd_m_s": max_or_none(linear_abs),
        "mean_abs_angular_cmd_rad_s": mean_or_none(angular_abs),
        "max_abs_angular_cmd_rad_s": max_or_none(angular_abs),
        "stop_command_fraction": (
            stop_count / len(cmd_commands) if cmd_commands else None
        ),
        "mean_cmd_gap_ms": mean_or_none(cmd_gaps_ms),
        "median_cmd_gap_ms": median_or_none(cmd_gaps_ms),
        "p95_cmd_gap_ms": percentile(cmd_gaps_ms, 0.95),
        "max_cmd_gap_ms": max_or_none(cmd_gaps_ms),
        "mean_scan_age_ms": mean_or_none(scan_ages_ms),
        "max_scan_age_ms": max_or_none(scan_ages_ms),
        "mean_amcl_age_ms": mean_or_none(amcl_ages_ms),
        "max_amcl_age_ms": max_or_none(amcl_ages_ms),
        "startup_window_s": STARTUP_WINDOW_S,
        "startup_path_length_5s_m": startup_path_m,
        "startup_net_displacement_5s_m": startup_net_m,
        "startup_efficiency_5s": startup_efficiency,
        "startup_angular_sign_changes_5s": count_angular_sign_changes(startup_commands),
        "startup_reverse_cmd_count_5s": reverse_count,
        "startup_mean_abs_angular_cmd_rad_s": mean_or_none(
            [abs(item[2]) for item in startup_commands]
        ),
        "startup_max_abs_angular_cmd_rad_s": max_or_none(
            [abs(item[2]) for item in startup_commands]
        ),
        "time_to_first_motion_s": first_motion_time_s,
        "odom_messages_mission": len(odom_points),
        "scan_messages_mission": counts["/scan"],
        "amcl_messages_mission": counts["/amcl_pose"],
        "cmd_vel_messages_mission": len(cmd_commands),
    }


def matches_expected_delay(requested_us: Any, expected_delay_ms: int) -> bool:
    number = finite_number(requested_us)
    if number is None:
        return False
    return abs(number - expected_delay_ms * 1000.0) <= REQUESTED_DELAY_TOLERANCE_US


def delay_stats_from_rows(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    measured_ms = numeric_values(
        (
            finite_number(row.get("measured_delay_us")) / 1000.0
            if finite_number(row.get("measured_delay_us")) is not None
            else None
            for row in rows
        )
    )
    errors_ms = numeric_values(
        (
            finite_number(row.get("delay_error_us")) / 1000.0
            if finite_number(row.get("delay_error_us")) is not None
            else None
            for row in rows
        ),
        allow_negative=True,
    )
    requested_ms = numeric_values(
        (
            finite_number(row.get("requested_delay_us")) / 1000.0
            if finite_number(row.get("requested_delay_us")) is not None
            else None
            for row in rows
        )
    )
    output = {
        "requested_delay_ms_from_csv": median_or_none(requested_ms),
    }
    output.update(summarise(measured_ms, "measured_delay_ms"))
    output.update(summarise(errors_ms, "delay_error_ms"))
    return output


def parse_te1_timing(run: Dict[str, Any], warnings: List[str]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Parse T-E1 while excluding the duplicated legacy first record.

    The first row of every T-E1 timing CSV contains a numeric displacement in
    field 3. The following rows use the literal hook name
    ``amcl_pose_publish``. Inspection of all input files shows that the first
    row has the same scan timestamp and delay values as the first hook row and
    therefore represents a legacy control/summary record, not an independent
    callback observation. It is checked for consistency but not aggregated.
    """
    path: Path = run["timing_path"]
    raw_rows: List[List[str]] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        for row in csv.reader(handle):
            if len(row) < 6:
                continue
            raw_rows.append(row[:6])

    if not raw_rows:
        raise AnalysisError(f"Empty T-E1 timing file: {path}")

    condition_rows: List[Dict[str, Any]] = []
    displacement_ms: List[float] = []
    legacy_rows = 0
    legacy_check_mismatches = 0
    unknown_hook_rows = 0
    malformed_rows = 0
    other_delay_rows = 0

    for index, raw in enumerate(raw_rows):
        publish_time = finite_number(raw[0])
        scan_time = finite_number(raw[1])
        requested_us = finite_number(raw[3])
        measured_us = finite_number(raw[4])
        error_us = finite_number(raw[5])
        if None in (publish_time, scan_time, requested_us, measured_us, error_us):
            malformed_rows += 1
            continue
        if not matches_expected_delay(requested_us, int(run["delay_ms"])):
            other_delay_rows += 1
            continue

        displacement = (publish_time - scan_time) * 1000.0
        if not math.isfinite(displacement) or displacement < 0.0:
            malformed_rows += 1
            continue

        third_field = str(raw[2]).strip()
        explicit = finite_number(third_field)
        if explicit is not None:
            # Legacy first record: use only as a consistency check.
            legacy_rows += 1
            if abs(explicit - displacement) > 1.0:
                legacy_check_mismatches += 1
            continue

        if third_field != "amcl_pose_publish":
            unknown_hook_rows += 1
            continue

        condition_rows.append({
            "publish_time_sec": publish_time,
            "scan_time_sec": scan_time,
            "third_field": third_field,
            "requested_delay_us": requested_us,
            "measured_delay_us": measured_us,
            "delay_error_us": error_us,
            "source_line": index + 1,
        })
        displacement_ms.append(displacement)

    if not condition_rows:
        raise AnalysisError(f"No T-E1 callback rows match {run['delay_ms']} ms in {path}")

    warning_parts: List[str] = []
    if legacy_rows != 1:
        warning_parts.append(
            f"expected one legacy control row, found {legacy_rows}"
        )
    if legacy_check_mismatches:
        warning_parts.append(
            f"{legacy_check_mismatches}/{legacy_rows} legacy displacement checks differed "
            "from timestamp reconstruction by more than 1 ms"
        )
    if malformed_rows:
        warning_parts.append(f"excluded {malformed_rows} malformed/invalid row(s)")
    if other_delay_rows:
        warning_parts.append(f"excluded {other_delay_rows} row(s) from another delay")
    if unknown_hook_rows:
        warning_parts.append(f"excluded {unknown_hook_rows} row(s) with an unknown hook")

    warning_text = "; ".join(warning_parts)
    if warning_text:
        warnings.append(f"{run['run_id']}: {warning_text}")

    excluded = len(raw_rows) - len(condition_rows)
    result: Dict[str, Any] = {
        "timing_scope": "all_matching_callback_events_excluding_legacy_control_row",
        "timing_event_count_total": len(raw_rows),
        "timing_event_count_condition": len(condition_rows),
        "timing_event_count_used": len(displacement_ms),
        "pose_displacement_scope": (
            "all_amcl_pose_publish_events_from_publish_and_scan_timestamps; "
            "legacy_numeric_first_row_excluded"
        ),
        "pose_displacement_sample_count": len(displacement_ms),
        "te1_legacy_control_rows_excluded": legacy_rows,
        "te1_legacy_control_check_mismatches": legacy_check_mismatches,
        "te1_unknown_hook_rows_excluded": unknown_hook_rows,
    }
    result.update(delay_stats_from_rows(condition_rows))
    result.update(summarise(displacement_ms, "pose_temporal_displacement_ms"))
    result["internal_displacement_median_ms"] = median_or_none(displacement_ms)
    result["internal_displacement_max_ms"] = max_or_none(displacement_ms)

    quality = {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "delay_ms": run["delay_ms"],
        "timing_file": run["timing_file"],
        "timing_rows_total": len(raw_rows),
        "timing_rows_matching_condition": len(condition_rows),
        "timing_rows_excluded": excluded,
        "timing_rows_expected_legacy_excluded": legacy_rows,
        "timing_rows_invalid_excluded": malformed_rows + unknown_hook_rows,
        "timing_rows_other_delay_excluded": other_delay_rows,
        "timing_parser_note": (
            "T-E1 displacement reconstructed for every amcl_pose_publish event as "
            "(publish_time_sec - scan_time_sec) * 1000. The duplicated legacy "
            "numeric first row is checked but excluded from all statistics."
        ),
        "quality_status": "warning" if warning_text else "ok",
        "quality_warning": warning_text,
    }
    return result, quality


def parse_te2_timing(run: Dict[str, Any], warnings: List[str]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    path: Path = run["timing_path"]
    raw_rows = read_dict_rows(path)
    required = {
        "tf_send_time_sec",
        "scan_time_sec",
        "tf_stamp_sec",
        "temporal_displacement_ms",
        "tf_stamp_lag_ms",
        "requested_delay_us",
        "measured_delay_us",
        "delay_error_us",
    }
    if not raw_rows or not required.issubset(set(raw_rows[0].keys())):
        raise AnalysisError(f"Unrecognised T-E2 schema in {path}")

    selected = [
        row
        for row in raw_rows
        if matches_expected_delay(row.get("requested_delay_us"), int(run["delay_ms"]))
    ]
    if not selected:
        raise AnalysisError(f"No T-E2 rows match {run['delay_ms']} ms in {path}")

    displacement = numeric_values(
        (row.get("temporal_displacement_ms") for row in selected)
    )
    lag = numeric_values(
        (row.get("tf_stamp_lag_ms") for row in selected),
        allow_negative=True,
    )
    future_margin = [-value for value in lag]

    result: Dict[str, Any] = {
        "timing_scope": "condition_rows_all_recording",
        "timing_event_count_total": len(raw_rows),
        "timing_event_count_condition": len(selected),
        "timing_event_count_used": len(selected),
    }
    result.update(delay_stats_from_rows(selected))
    result.update(summarise(displacement, "tf_temporal_displacement_ms"))
    result.update(summarise(lag, "tf_timestamp_lag_ms"))
    result.update(summarise(future_margin, "tf_future_margin_ms"))
    result["internal_displacement_median_ms"] = median_or_none(displacement)
    result["internal_displacement_max_ms"] = max_or_none(displacement)

    excluded = len(raw_rows) - len(selected)
    warning_text = ""
    status = "ok"
    if excluded:
        status = "warning"
        warning_text = (
            f"Excluded {excluded}/{len(raw_rows)} rows whose requested delay "
            f"did not match {run['delay_ms']} ms"
        )
        warnings.append(f"{run['run_id']}: {warning_text}")

    quality = {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "delay_ms": run["delay_ms"],
        "timing_file": run["timing_file"],
        "timing_rows_total": len(raw_rows),
        "timing_rows_matching_condition": len(selected),
        "timing_rows_excluded": excluded,
        "timing_parser_note": "T-E2 rows filtered by requested_delay_us",
        "quality_status": status,
        "quality_warning": warning_text,
    }
    return result, quality


def parse_te4_timing(run: Dict[str, Any], warnings: List[str]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Parse T-E4 and reject internally inconsistent cycle records uniformly."""
    path: Path = run["timing_path"]
    raw_rows = read_dict_rows(path)
    required = {
        "cycle_start_time_sec",
        "planner_call_time_sec",
        "command_ready_time_sec",
        "command_publish_time_sec",
        "requested_delay_us",
        "measured_delay_us",
        "delay_error_us",
        "scheduling_displacement_ms",
        "planner_computation_us",
        "command_ready_latency_ms",
        "command_publish_latency_ms",
        "valid_command",
    }
    if not raw_rows or not required.issubset(set(raw_rows[0].keys())):
        raise AnalysisError(f"Unrecognised T-E4 schema in {path}")

    delay_selected = [
        row
        for row in raw_rows
        if matches_expected_delay(row.get("requested_delay_us"), int(run["delay_ms"]))
    ]
    if not delay_selected:
        raise AnalysisError(f"No T-E4 rows match {run['delay_ms']} ms in {path}")

    selected: List[Dict[str, Any]] = []
    malformed_rows = 0
    timestamp_order_rows = 0
    latency_consistency_rows = 0
    nonincreasing_publish_rows = 0
    previous_publish_time: Optional[float] = None

    for row in delay_selected:
        cycle_start = finite_number(row.get("cycle_start_time_sec"))
        planner_call = finite_number(row.get("planner_call_time_sec"))
        command_ready = finite_number(row.get("command_ready_time_sec"))
        command_publish = finite_number(row.get("command_publish_time_sec"))
        scheduling_reported = finite_number(row.get("scheduling_displacement_ms"))
        ready_reported = finite_number(row.get("command_ready_latency_ms"))
        publish_reported = finite_number(row.get("command_publish_latency_ms"))
        measured_us = finite_number(row.get("measured_delay_us"))
        error_us = finite_number(row.get("delay_error_us"))
        planner_us = finite_number(row.get("planner_computation_us"))
        valid_command = finite_number(row.get("valid_command"))

        numeric = (
            cycle_start,
            planner_call,
            command_ready,
            command_publish,
            scheduling_reported,
            ready_reported,
            publish_reported,
            measured_us,
            error_us,
            planner_us,
            valid_command,
        )
        if any(value is None for value in numeric):
            malformed_rows += 1
            continue

        assert cycle_start is not None
        assert planner_call is not None
        assert command_ready is not None
        assert command_publish is not None
        assert scheduling_reported is not None
        assert ready_reported is not None
        assert publish_reported is not None

        if not (
            cycle_start <= planner_call + TIMESTAMP_ORDER_TOLERANCE_S
            and planner_call <= command_ready + TIMESTAMP_ORDER_TOLERANCE_S
            and command_ready <= command_publish + TIMESTAMP_ORDER_TOLERANCE_S
        ):
            timestamp_order_rows += 1
            continue

        expected_scheduling = (planner_call - cycle_start) * 1000.0
        expected_ready = (command_ready - cycle_start) * 1000.0
        expected_publish = (command_publish - cycle_start) * 1000.0
        if (
            abs(expected_scheduling - scheduling_reported) > TIMESTAMP_METRIC_TOLERANCE_MS
            or abs(expected_ready - ready_reported) > TIMESTAMP_METRIC_TOLERANCE_MS
            or abs(expected_publish - publish_reported) > TIMESTAMP_METRIC_TOLERANCE_MS
        ):
            latency_consistency_rows += 1
            continue

        # Consecutive controller cycles must have strictly increasing publication
        # timestamps. A repeated or decreasing timestamp cannot define a valid
        # publication interval and is excluded from *all* T-E4 statistics.
        if (
            previous_publish_time is not None
            and command_publish <= previous_publish_time + TIMESTAMP_ORDER_TOLERANCE_S
        ):
            nonincreasing_publish_rows += 1
            continue

        selected.append(row)
        previous_publish_time = command_publish

    if not selected:
        raise AnalysisError(f"No valid T-E4 timing rows remain for {run['run_id']}")

    scheduling = numeric_values(
        (row.get("scheduling_displacement_ms") for row in selected)
    )
    planner_ms = numeric_values(
        (
            finite_number(row.get("planner_computation_us")) / 1000.0
            if finite_number(row.get("planner_computation_us")) is not None
            else None
            for row in selected
        )
    )
    ready_latency = numeric_values(
        (row.get("command_ready_latency_ms") for row in selected)
    )
    publish_latency = numeric_values(
        (row.get("command_publish_latency_ms") for row in selected)
    )
    valid_commands = numeric_values((row.get("valid_command") for row in selected))

    publish_times = numeric_values(
        (row.get("command_publish_time_sec") for row in selected)
    )
    publish_intervals = [
        (second - first) * 1000.0
        for first, second in zip(publish_times, publish_times[1:])
    ]
    # Strict increase was enforced above; keep a defensive filter for floating
    # point safety and expose any unexpected removal as a warning.
    unexpected_nonpositive_intervals = sum(1 for value in publish_intervals if value <= 0.0)
    publish_intervals = [value for value in publish_intervals if value > 0.0]
    publish_rates = [1000.0 / interval for interval in publish_intervals]

    exceedance_ratio = (
        sum(1 for value in publish_latency if value > NOMINAL_CONTROLLER_PERIOD_MS)
        / len(publish_latency)
        if publish_latency
        else None
    )
    valid_ratio = mean_or_none(valid_commands)

    result: Dict[str, Any] = {
        "timing_scope": "valid_condition_rows_all_recording",
        "timing_event_count_total": len(raw_rows),
        "timing_event_count_condition": len(delay_selected),
        "timing_event_count_used": len(selected),
        "nominal_controller_period_ms": NOMINAL_CONTROLLER_PERIOD_MS,
        "diagnostic_period_exceedance_ratio": exceedance_ratio,
        # Legacy-compatible name. The report treats this only as diagnostic.
        "deadline_miss_ratio": exceedance_ratio,
        "valid_command_ratio": valid_ratio,
        "zero_command_publish_intervals": 0,
        "negative_command_publish_intervals_excluded": 0,
        "te4_invalid_rows_excluded": len(delay_selected) - len(selected),
        "te4_nonincreasing_publish_rows_excluded": nonincreasing_publish_rows,
        "te4_timestamp_order_rows_excluded": timestamp_order_rows,
        "te4_latency_consistency_rows_excluded": latency_consistency_rows,
    }
    result.update(delay_stats_from_rows(selected))
    result.update(summarise(scheduling, "scheduling_displacement_ms"))
    result.update(summarise(planner_ms, "planner_computation_ms"))
    result.update(summarise(ready_latency, "command_ready_latency_ms"))
    result.update(summarise(publish_latency, "command_publish_latency_ms"))
    result.update(summarise(publish_intervals, "command_publish_interval_ms"))
    result.update(summarise(publish_rates, "effective_publish_rate_hz"))
    result["internal_interval_median_ms"] = median_or_none(publish_intervals)
    result["internal_interval_max_ms"] = max_or_none(publish_intervals)
    result["internal_displacement_median_ms"] = median_or_none(scheduling)
    result["internal_displacement_max_ms"] = max_or_none(scheduling)

    warning_parts: List[str] = []
    other_delay_rows = len(raw_rows) - len(delay_selected)
    if other_delay_rows:
        warning_parts.append(
            f"excluded {other_delay_rows}/{len(raw_rows)} rows with another requested delay"
        )
    if malformed_rows:
        warning_parts.append(f"excluded {malformed_rows} malformed row(s)")
    if timestamp_order_rows:
        warning_parts.append(
            f"excluded {timestamp_order_rows} row(s) with non-monotonic intra-cycle timestamps"
        )
    if latency_consistency_rows:
        warning_parts.append(
            f"excluded {latency_consistency_rows} row(s) whose reported latencies did not match timestamps"
        )
    if nonincreasing_publish_rows:
        warning_parts.append(
            f"excluded {nonincreasing_publish_rows} row(s) with repeated/decreasing publication timestamps from all T-E4 statistics"
        )
    if unexpected_nonpositive_intervals:
        warning_parts.append(
            f"defensively excluded {unexpected_nonpositive_intervals} non-positive interval(s) after row validation"
        )
    warning_text = "; ".join(warning_parts)
    if warning_text:
        warnings.append(f"{run['run_id']}: {warning_text}")

    quality = {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "delay_ms": run["delay_ms"],
        "timing_file": run["timing_file"],
        "timing_rows_total": len(raw_rows),
        "timing_rows_matching_condition": len(delay_selected),
        "timing_rows_excluded": len(raw_rows) - len(selected),
        "timing_rows_expected_legacy_excluded": 0,
        "timing_rows_invalid_excluded": len(delay_selected) - len(selected),
        "timing_rows_other_delay_excluded": other_delay_rows,
        "timing_parser_note": (
            "T-E4 rows validated before aggregation. Rows with missing values, "
            "non-monotonic intra-cycle timestamps, inconsistent reported latency, "
            "or non-increasing command publication timestamps are excluded from "
            "all timing statistics. Intervals use consecutive remaining publication timestamps."
        ),
        "quality_status": "warning" if warning_text else "ok",
        "quality_warning": warning_text,
    }
    return result, quality


def parse_timing(run: Dict[str, Any], warnings: List[str]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    parser_name = run["parser"]
    if parser_name == "te1":
        return parse_te1_timing(run, warnings)
    if parser_name == "te2":
        return parse_te2_timing(run, warnings)
    if parser_name == "te4":
        return parse_te4_timing(run, warnings)
    raise AnalysisError(f"Unknown timing parser: {parser_name}")


def blank_bag_metrics() -> Dict[str, Any]:
    return {name: None for name in BAG_FIELDS}


def success(row: Dict[str, Any]) -> bool:
    return str(row.get("result_from_bag") or "").upper() == "SUCCEEDED"


def condition_summary(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, int], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["experiment"]), int(row["delay_ms"]))].append(row)

    metrics = [
        "mission_duration_s",
        "observation_duration_s",
        "odom_path_length_m",
        "minimum_scan_m",
        "stop_command_fraction",
        "mean_cmd_gap_ms",
        "median_cmd_gap_ms",
        "p95_cmd_gap_ms",
        "max_cmd_gap_ms",
        "measured_delay_ms_mean",
        "pose_temporal_displacement_ms_mean",
        "tf_temporal_displacement_ms_mean",
        "tf_future_margin_ms_mean",
        "scheduling_displacement_ms_mean",
        "planner_computation_ms_mean",
        "command_publish_latency_ms_mean",
        "command_publish_interval_ms_mean",
        "effective_publish_rate_hz_mean",
        "diagnostic_period_exceedance_ratio",
        "valid_command_ratio",
    ]

    output: List[Dict[str, Any]] = []
    for (experiment, delay_ms), group in sorted(grouped.items()):
        item: Dict[str, Any] = {
            "experiment": experiment,
            "delay_ms": delay_ms,
            "n_runs": len(group),
            "success_count": sum(1 for row in group if success(row)),
            "aborted_count": sum(
                1 for row in group
                if str(row.get("result_from_bag") or "").upper() == "ABORTED"
            ),
            "no_result_count": sum(
                1 for row in group
                if str(row.get("result_from_bag") or "").upper() == "NO_RESULT"
            ),
        }
        for metric in metrics:
            values = numeric_values((row.get(metric) for row in group), allow_negative=True)
            item.update({
                f"{metric}__mean_across_runs": mean_or_none(values),
                f"{metric}__stdev_across_runs": stdev_or_none(values),
                f"{metric}__median_across_runs": median_or_none(values),
                f"{metric}__min_across_runs": min_or_none(values),
                f"{metric}__max_across_runs": max_or_none(values),
            })
        output.append(item)
    return output


def report_relevant_summary(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, int], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["experiment"]), int(row["delay_ms"]))].append(row)

    output: List[Dict[str, Any]] = []
    for (experiment, delay_ms), group in sorted(grouped.items()):
        successful = [row for row in group if success(row)]

        def vals(metric: str, source: Sequence[Dict[str, Any]] = group) -> List[float]:
            return numeric_values((row.get(metric) for row in source), allow_negative=True)

        output.append({
            "experiment": experiment,
            "delay_ms": delay_ms,
            "n_runs": len(group),
            "success_count": len(successful),
            "measured_delay_ms_mean_across_run_means": mean_or_none(
                vals("measured_delay_ms_mean")
            ),
            "pose_temporal_displacement_ms_mean_across_runs": mean_or_none(
                vals("pose_temporal_displacement_ms_mean")
            ),
            "tf_temporal_displacement_ms_mean_across_run_means": mean_or_none(
                vals("tf_temporal_displacement_ms_mean")
            ),
            "tf_future_margin_ms_mean_across_run_means": mean_or_none(
                vals("tf_future_margin_ms_mean")
            ),
            "scheduling_displacement_ms_mean_across_run_means": mean_or_none(
                vals("scheduling_displacement_ms_mean")
            ),
            "planner_computation_ms_mean_across_run_means": mean_or_none(
                vals("planner_computation_ms_mean")
            ),
            "command_publish_latency_ms_mean_across_run_means": mean_or_none(
                vals("command_publish_latency_ms_mean")
            ),
            "command_publish_interval_ms_mean_across_run_means": mean_or_none(
                vals("command_publish_interval_ms_mean")
            ),
            "effective_publish_rate_hz_mean_across_run_means": mean_or_none(
                vals("effective_publish_rate_hz_mean")
            ),
            "diagnostic_period_exceedance_ratio_mean_across_runs": mean_or_none(
                vals("diagnostic_period_exceedance_ratio")
            ),
            "median_cmd_gap_ms_condition_median": median_or_none(
                vals("median_cmd_gap_ms")
            ),
            "p95_cmd_gap_ms_condition_median": median_or_none(
                vals("p95_cmd_gap_ms")
            ),
            "stop_command_fraction_condition_median": median_or_none(
                vals("stop_command_fraction")
            ),
            "mission_duration_successful_median_s": median_or_none(
                vals("mission_duration_s", successful)
            ),
            "path_length_all_runs_median_m": median_or_none(
                vals("odom_path_length_m")
            ),
            "path_length_successful_median_m": median_or_none(
                vals("odom_path_length_m", successful)
            ),
            "minimum_scan_all_runs_median_m": median_or_none(
                vals("minimum_scan_m")
            ),
        })
    return output


CORE_FIELDS = [
    "experiment",
    "run_id",
    "delay_ms",
    "run_number",
    "injection_point",
    "bag_file",
    "timing_file",
    "source_bag_path",
    "source_timing_path",
]

BAG_FIELDS = [
    "bag_duration_s",
    "goal_ros_time_s",
    "result_ros_time_s",
    "mission_duration_s",
    "observation_duration_s",
    "result_from_bag",
    "result_status_text",
    "recovery_indicated_by_status",
    "odom_path_length_m",
    "minimum_scan_m",
    "mean_abs_linear_cmd_m_s",
    "max_abs_linear_cmd_m_s",
    "mean_abs_angular_cmd_rad_s",
    "max_abs_angular_cmd_rad_s",
    "stop_command_fraction",
    "mean_cmd_gap_ms",
    "median_cmd_gap_ms",
    "p95_cmd_gap_ms",
    "max_cmd_gap_ms",
    "mean_scan_age_ms",
    "max_scan_age_ms",
    "mean_amcl_age_ms",
    "max_amcl_age_ms",
    "startup_window_s",
    "startup_path_length_5s_m",
    "startup_net_displacement_5s_m",
    "startup_efficiency_5s",
    "startup_angular_sign_changes_5s",
    "startup_reverse_cmd_count_5s",
    "startup_mean_abs_angular_cmd_rad_s",
    "startup_max_abs_angular_cmd_rad_s",
    "time_to_first_motion_s",
    "odom_messages_mission",
    "scan_messages_mission",
    "amcl_messages_mission",
    "cmd_vel_messages_mission",
]

TIMING_COMMON_FIELDS = [
    "timing_scope",
    "timing_event_count_total",
    "timing_event_count_condition",
    "timing_event_count_used",
    "requested_delay_ms_from_csv",
    "measured_delay_ms_n",
    "measured_delay_ms_mean",
    "measured_delay_ms_median",
    "measured_delay_ms_stdev",
    "measured_delay_ms_min",
    "measured_delay_ms_p95",
    "measured_delay_ms_max",
    "delay_error_ms_n",
    "delay_error_ms_mean",
    "delay_error_ms_median",
    "delay_error_ms_stdev",
    "delay_error_ms_min",
    "delay_error_ms_p95",
    "delay_error_ms_max",
    "internal_interval_median_ms",
    "internal_interval_max_ms",
    "internal_displacement_median_ms",
    "internal_displacement_max_ms",
]

TE1_FIELDS = [
    "pose_displacement_scope",
    "pose_displacement_sample_count",
    "te1_legacy_control_rows_excluded",
    "te1_legacy_control_check_mismatches",
    "te1_unknown_hook_rows_excluded",
    "pose_temporal_displacement_ms_n",
    "pose_temporal_displacement_ms_mean",
    "pose_temporal_displacement_ms_median",
    "pose_temporal_displacement_ms_stdev",
    "pose_temporal_displacement_ms_min",
    "pose_temporal_displacement_ms_p95",
    "pose_temporal_displacement_ms_max",
]

TE2_FIELDS = [
    "tf_temporal_displacement_ms_n",
    "tf_temporal_displacement_ms_mean",
    "tf_temporal_displacement_ms_median",
    "tf_temporal_displacement_ms_stdev",
    "tf_temporal_displacement_ms_min",
    "tf_temporal_displacement_ms_p95",
    "tf_temporal_displacement_ms_max",
    "tf_timestamp_lag_ms_n",
    "tf_timestamp_lag_ms_mean",
    "tf_timestamp_lag_ms_median",
    "tf_timestamp_lag_ms_stdev",
    "tf_timestamp_lag_ms_min",
    "tf_timestamp_lag_ms_p95",
    "tf_timestamp_lag_ms_max",
    "tf_future_margin_ms_n",
    "tf_future_margin_ms_mean",
    "tf_future_margin_ms_median",
    "tf_future_margin_ms_stdev",
    "tf_future_margin_ms_min",
    "tf_future_margin_ms_p95",
    "tf_future_margin_ms_max",
]

TE4_FIELDS = [
    "nominal_controller_period_ms",
    "scheduling_displacement_ms_n",
    "scheduling_displacement_ms_mean",
    "scheduling_displacement_ms_median",
    "scheduling_displacement_ms_stdev",
    "scheduling_displacement_ms_min",
    "scheduling_displacement_ms_p95",
    "scheduling_displacement_ms_max",
    "planner_computation_ms_n",
    "planner_computation_ms_mean",
    "planner_computation_ms_median",
    "planner_computation_ms_stdev",
    "planner_computation_ms_min",
    "planner_computation_ms_p95",
    "planner_computation_ms_max",
    "command_ready_latency_ms_n",
    "command_ready_latency_ms_mean",
    "command_ready_latency_ms_median",
    "command_ready_latency_ms_stdev",
    "command_ready_latency_ms_min",
    "command_ready_latency_ms_p95",
    "command_ready_latency_ms_max",
    "command_publish_latency_ms_n",
    "command_publish_latency_ms_mean",
    "command_publish_latency_ms_median",
    "command_publish_latency_ms_stdev",
    "command_publish_latency_ms_min",
    "command_publish_latency_ms_p95",
    "command_publish_latency_ms_max",
    "command_publish_interval_ms_n",
    "command_publish_interval_ms_mean",
    "command_publish_interval_ms_median",
    "command_publish_interval_ms_stdev",
    "command_publish_interval_ms_min",
    "command_publish_interval_ms_p95",
    "command_publish_interval_ms_max",
    "effective_publish_rate_hz_n",
    "effective_publish_rate_hz_mean",
    "effective_publish_rate_hz_median",
    "effective_publish_rate_hz_stdev",
    "effective_publish_rate_hz_min",
    "effective_publish_rate_hz_p95",
    "effective_publish_rate_hz_max",
    "diagnostic_period_exceedance_ratio",
    "deadline_miss_ratio",
    "valid_command_ratio",
    "zero_command_publish_intervals",
    "negative_command_publish_intervals_excluded",
    "te4_invalid_rows_excluded",
    "te4_nonincreasing_publish_rows_excluded",
    "te4_timestamp_order_rows_excluded",
    "te4_latency_consistency_rows_excluded",
]

MASTER_FIELDS = CORE_FIELDS + BAG_FIELDS + TIMING_COMMON_FIELDS + TE1_FIELDS + TE2_FIELDS + TE4_FIELDS

QUALITY_FIELDS = [
    "experiment",
    "run_id",
    "delay_ms",
    "timing_file",
    "timing_rows_total",
    "timing_rows_matching_condition",
    "timing_rows_excluded",
    "timing_rows_expected_legacy_excluded",
    "timing_rows_invalid_excluded",
    "timing_rows_other_delay_excluded",
    "timing_parser_note",
    "quality_status",
    "quality_warning",
]


def import_rosbag_or_exit() -> Any:
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        raise AnalysisError(
            "Cannot import the ROS1 'rosbag' Python module. Run the script in "
            "the Ubuntu/ROS Noetic environment used for the experiments and "
            "source ROS first:\n"
            "  source /opt/ros/noetic/setup.bash\n"
            "Then run this script again. Use --skip-bags only for an internal "
            "timing test; that output is not a complete final master CSV."
        ) from exc
    return rosbag


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rebuild the T-E1/T-E2/T-E4 all-runs master CSV from raw data."
    )
    parser.add_argument(
        "input",
        type=Path,
        help="results-per-injection-point.zip or the extracted directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("analysis_output_rebuilt"),
        help="Output directory (default: analysis_output_rebuilt)",
    )
    parser.add_argument(
        "--skip-bags",
        action="store_true",
        help=(
            "Parse only timing CSVs. Useful to test internal metrics without ROS; "
            "the resulting master is incomplete."
        ),
    )
    parser.add_argument(
        "--copy-input-script",
        action="store_true",
        help="Copy this script into the output directory for provenance.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    warnings: List[str] = []
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rosbag_module = None if args.skip_bags else import_rosbag_or_exit()

    with prepare_input(args.input) as root:
        runs = discover_expected_runs(root, warnings)
        if len(runs) != 95:
            warnings.append(
                f"Expected 95 complete runs for T-E1/T-E2/T-E4, discovered {len(runs)}"
            )

        master_rows: List[Dict[str, Any]] = []
        quality_rows: List[Dict[str, Any]] = []

        for index, run in enumerate(runs, start=1):
            print(f"[{index:02d}/{len(runs)}] {run['run_id']}")
            row = {name: run.get(name) for name in CORE_FIELDS}

            if args.skip_bags:
                row.update(blank_bag_metrics())
            else:
                row.update(analyse_bag(run, rosbag_module, warnings))

            timing, quality = parse_timing(run, warnings)
            row.update(timing)
            master_rows.append(row)
            quality_rows.append(quality)

    master_path = args.output_dir / "all_runs_master.csv"
    write_csv(master_path, master_rows, MASTER_FIELDS)

    summary_rows = condition_summary(master_rows)
    summary_fields: List[str] = []
    for row in summary_rows:
        for key in row:
            if key not in summary_fields:
                summary_fields.append(key)
    write_csv(args.output_dir / "condition_summary.csv", summary_rows, summary_fields)

    report_rows = report_relevant_summary(master_rows)
    report_fields: List[str] = []
    for row in report_rows:
        for key in row:
            if key not in report_fields:
                report_fields.append(key)
    write_csv(
        args.output_dir / "report_relevant_summary.csv",
        report_rows,
        report_fields,
    )

    write_csv(args.output_dir / "data_quality.csv", quality_rows, QUALITY_FIELDS)

    warning_path = args.output_dir / "analysis_warnings.txt"
    with warning_path.open("w", encoding="utf-8") as handle:
        if warnings:
            for warning in warnings:
                handle.write(f"- {warning}\n")
        else:
            handle.write("No warnings.\n")

    if args.copy_input_script:
        script_path = Path(__file__).resolve()
        destination = args.output_dir / script_path.name
        if script_path != destination.resolve():
            shutil.copy2(str(script_path), str(destination))

    print(f"\nCreated: {master_path}")
    print(f"Rows: {len(master_rows)}")
    print(f"Warnings: {len(warnings)} (see {warning_path})")
    if args.skip_bags:
        print(
            "NOTE: --skip-bags was used. External ROS-bag metrics are blank; "
            "this output is not the final repository master."
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AnalysisError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
