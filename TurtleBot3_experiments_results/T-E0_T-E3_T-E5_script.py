#!/usr/bin/env python3
"""
Build a comparable analysis dataset for E1, E2 (DWA delay), and E3
(local-costmap delay) from ROS Noetic bag files and timing CSV files.

Run from a ROS Noetic shell, for example:
    source /opt/ros/noetic/setup.bash
    source ~/Desktop/project/catkin_ws/devel/setup.bash
    python3 analyze_all_experiments.py ~/Desktop/project/results

Only Python's standard library and ROS's rosbag module are required.
"""

import csv
import math
import os
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

try:
    import rosbag
except ImportError as exc:
    raise SystemExit(
        "Cannot import rosbag. Source ROS Noetic first:\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "  source ~/Desktop/project/catkin_ws/devel/setup.bash"
    ) from exc


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

TOPICS = [
    "/move_base_simple/goal",
    "/move_base/result",
    "/odom",
    "/cmd_vel",
    "/scan",
    "/amcl_pose",
]

STARTUP_WINDOW_S = 5.0
LINEAR_ZERO_THRESHOLD = 0.01
ANGULAR_ZERO_THRESHOLD = 0.05
ODOM_JUMP_THRESHOLD_M = 0.5


class AnalysisError(RuntimeError):
    pass


def finite_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def valid_nonnegative(values):
    result = []
    for value in values:
        number = finite_number(value)
        if number is not None and number >= 0.0:
            result.append(number)
    return result


def mean_or_none(values):
    return statistics.mean(values) if values else None


def median_or_none(values):
    return statistics.median(values) if values else None


def stdev_or_none(values):
    return statistics.stdev(values) if len(values) >= 2 else None


def min_or_none(values):
    return min(values) if values else None


def max_or_none(values):
    return max(values) if values else None


def percentile(values, fraction):
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


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return value


def write_csv(path, rows, fieldnames=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if fieldnames is None:
        fieldnames = []
        seen = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    seen.add(key)
                    fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key)) for key in fieldnames})


def read_csv_rows(path):
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def load_manual_runs(results_root, warnings):
    mapping = {}
    for path in [
        results_root / "baseline" / "runs.csv",
        results_root / "E2_DWA" / "runs.csv",
        results_root / "E3_COSTMAP" / "runs.csv",
    ]:
        if not path.exists():
            warnings.append(f"Missing manual runs file: {path}")
            continue
        for row in read_csv_rows(path):
            run_id = (row.get("run_id") or "").strip()
            if not run_id:
                warnings.append(f"Row without run_id in {path}")
                continue
            if run_id in mapping:
                warnings.append(f"Duplicate run_id in manual CSV files: {run_id}")
            mapping[run_id] = row
    return mapping


def identify_bag(path):
    stem = path.stem
    if re.fullmatch(r"baseline_run-\d+", stem):
        return "E1", stem, 0, "clean_baseline"
    match = re.fullmatch(r"dwa_(\d+)ms_run-\d+", stem)
    if match:
        return "E2", stem, int(match.group(1)), "dwa_compute_velocity_commands"
    match = re.fullmatch(r"costmap_(\d+)ms_run-\d+", stem)
    if match:
        return "E3", stem, int(match.group(1)), "local_costmap_update_loop"
    return None


def discover_runs(results_root, warnings):
    runs = []
    seen_run_ids = set()
    for bag_path in sorted(results_root.rglob("*.bag")):
        identified = identify_bag(bag_path)
        if not identified:
            continue
        experiment, run_id, delay_ms, injection_point = identified
        if run_id in seen_run_ids:
            warnings.append(f"Duplicate bag run_id: {run_id} ({bag_path})")
            continue
        seen_run_ids.add(run_id)
        runs.append({
            "experiment": experiment,
            "run_id": run_id,
            "delay_ms": delay_ms,
            "injection_point": injection_point,
            "bag_path": bag_path,
        })
    return runs


def find_timing_csv(run, results_root, warnings):
    if run["experiment"] == "E1":
        return None
    bag_dir = run["bag_path"].parent
    run_id = run["run_id"]
    direct_candidates = [
        bag_dir / f"{run_id}_timing.csv",
        bag_dir / f"{run_id}.csv",
    ]
    for candidate in direct_candidates:
        if candidate.exists():
            return candidate

    experiment_root = results_root / ("E2_DWA" if run["experiment"] == "E2" else "E3_COSTMAP")
    candidates = list(experiment_root.rglob(f"{run_id}_timing.csv"))
    candidates += list(experiment_root.rglob(f"{run_id}.csv"))
    candidates = sorted(set(candidates))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        warnings.append(f"Missing timing CSV for {run_id}")
    else:
        warnings.append(f"Multiple timing CSV candidates for {run_id}: {candidates}")
    return None


def find_log(run, results_root, warnings):
    if run["experiment"] == "E1":
        return None
    bag_dir = run["bag_path"].parent
    run_id = run["run_id"]
    candidate = bag_dir / f"{run_id}_navigation.log"
    if candidate.exists():
        return candidate
    experiment_root = results_root / ("E2_DWA" if run["experiment"] == "E2" else "E3_COSTMAP")
    candidates = sorted(experiment_root.rglob(f"{run_id}_navigation.log"))
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        warnings.append(f"Multiple log candidates for {run_id}: {candidates}")
    return None


def message_age_ms(record_time, header_stamp):
    stamp = header_stamp.to_sec()
    if stamp <= 0.0:
        return None
    return max(0.0, (record_time - stamp) * 1000.0)


def path_length(points):
    total = 0.0
    for first, second in zip(points, points[1:]):
        step = math.hypot(second[1] - first[1], second[2] - first[2])
        if step < ODOM_JUMP_THRESHOLD_M:
            total += step
    return total


def count_angular_sign_changes(commands):
    signs = []
    for _, _, angular in commands:
        if abs(angular) < ANGULAR_ZERO_THRESHOLD:
            continue
        signs.append(1 if angular > 0 else -1)
    return sum(1 for first, second in zip(signs, signs[1:]) if first != second)


def analyse_bag(run, warnings):
    bag_path = run["bag_path"]
    goal_time = None
    result_time = None
    result_status = "NO_RESULT"
    odom_points = []
    cmd_commands = []
    scan_ages_ms = []
    amcl_ages_ms = []
    minimum_scan_m = None
    counts = defaultdict(int)

    try:
        bag = rosbag.Bag(str(bag_path), "r")
    except Exception as exc:
        raise AnalysisError(f"Cannot open bag {bag_path}: {exc}") from exc

    bag_start = bag.get_start_time()
    bag_end = bag.get_end_time()

    try:
        for topic, message, bag_time in bag.read_messages(topics=TOPICS):
            current_time = bag_time.to_sec()

            if topic == "/move_base_simple/goal" and goal_time is None:
                goal_time = current_time
                continue

            if goal_time is None:
                continue

            if topic == "/move_base/result":
                result_time = current_time
                result_status = STATUS_NAMES.get(
                    message.status.status,
                    f"UNKNOWN_{message.status.status}",
                )
                break

            counts[topic] += 1

            if topic == "/odom":
                position = message.pose.pose.position
                odom_points.append((current_time, position.x, position.y))

            elif topic == "/cmd_vel":
                cmd_commands.append((
                    current_time,
                    float(message.linear.x),
                    float(message.angular.z),
                ))

            elif topic == "/scan":
                valid_ranges = [
                    value for value in message.ranges
                    if math.isfinite(value)
                    and message.range_min <= value <= message.range_max
                ]
                if valid_ranges:
                    current_minimum = min(valid_ranges)
                    minimum_scan_m = (
                        current_minimum if minimum_scan_m is None
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

    if goal_time is None:
        warnings.append(f"No goal found in {run['run_id']}")

    if result_time is None:
        warnings.append(
            f"No move_base result found in {run['run_id']}"
        )

    # Vera durata della missione: disponibile soltanto
    # quando il bag contiene un risultato.
    mission_duration_s = (
        result_time - goal_time
        if goal_time is not None and result_time is not None
        else None
    )

    # Per i run senza risultato indica per quanto tempo
    # il robot è stato osservato dopo l'invio del goal.
    observation_duration_s = (
        bag_end - goal_time
        if goal_time is not None and result_time is None
        else None
    )


    linear_abs = [abs(item[1]) for item in cmd_commands]
    angular_abs = [abs(item[2]) for item in cmd_commands]
    stop_count = sum(
        1 for _, linear, angular in cmd_commands
        if abs(linear) < LINEAR_ZERO_THRESHOLD
        and abs(angular) < ANGULAR_ZERO_THRESHOLD
    )
    cmd_gaps_ms = [
        (second[0] - first[0]) * 1000.0
        for first, second in zip(cmd_commands, cmd_commands[1:])
    ]

    startup_limit = (goal_time + STARTUP_WINDOW_S) if goal_time is not None else None
    startup_points = [
        point for point in odom_points
        if startup_limit is not None and point[0] <= startup_limit
    ]
    startup_commands = [
        command for command in cmd_commands
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
        1 for _, linear, _ in startup_commands
        if linear < -LINEAR_ZERO_THRESHOLD
    )
    first_motion_time_s = None
    if goal_time is not None:
        for command_time, linear, angular in cmd_commands:
            if abs(linear) >= LINEAR_ZERO_THRESHOLD or abs(angular) >= ANGULAR_ZERO_THRESHOLD:
                first_motion_time_s = command_time - goal_time
                break

    return {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "delay_ms": run["delay_ms"],
        "injection_point": run["injection_point"],
        "bag_file": str(bag_path.relative_to(bag_path.parents[2])) if len(bag_path.parents) >= 3 else str(bag_path),
        "bag_duration_s": bag_end - bag_start,
        "goal_ros_time_s": goal_time,
        "result_ros_time_s": result_time,
        "mission_duration_s": mission_duration_s,
	"observation_duration_s": observation_duration_s,
	"result_from_bag": result_status,
        "odom_path_length_m": path_length(odom_points),
        "minimum_scan_m": minimum_scan_m,
        "mean_abs_linear_cmd_m_s": mean_or_none(linear_abs),
        "max_abs_linear_cmd_m_s": max_or_none(linear_abs),
        "mean_abs_angular_cmd_rad_s": mean_or_none(angular_abs),
        "max_abs_angular_cmd_rad_s": max_or_none(angular_abs),
        "stop_command_fraction": (stop_count / len(cmd_commands)) if cmd_commands else None,
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
        "startup_mean_abs_angular_cmd_rad_s": mean_or_none([abs(item[2]) for item in startup_commands]),
        "startup_max_abs_angular_cmd_rad_s": max_or_none([abs(item[2]) for item in startup_commands]),
        "time_to_first_motion_s": first_motion_time_s,
        "odom_messages_mission": len(odom_points),
        "scan_messages_mission": counts["/scan"],
        "amcl_messages_mission": counts["/amcl_pose"],
        "cmd_vel_messages_mission": len(cmd_commands),
    }


def summarise_numeric(values, prefix):
    values = valid_nonnegative(values)
    return {
        f"{prefix}_mean": mean_or_none(values),
        f"{prefix}_median": median_or_none(values),
        f"{prefix}_stdev": stdev_or_none(values),
        f"{prefix}_min": min_or_none(values),
        f"{prefix}_p95": percentile(values, 0.95),
        f"{prefix}_max": max_or_none(values),
    }


def aggregate_timing(run, timing_path, goal_time, result_time, warnings):
    if timing_path is None:
        return None
    try:
        rows = read_csv_rows(timing_path)
    except Exception as exc:
        warnings.append(f"Cannot read timing CSV for {run['run_id']}: {exc}")
        return None
    if not rows:
        warnings.append(f"Empty timing CSV for {run['run_id']}: {timing_path}")
        return None

    fieldnames = set(rows[0].keys())
    is_e2 = "cycle_interval_ms" in fieldnames
    is_e3 = "update_completion_interval_ms" in fieldnames
    if not is_e2 and not is_e3:
        warnings.append(
            f"Unrecognised timing schema for {run['run_id']}: {sorted(fieldnames)}"
        )
        return None

    timestamp_key = None
    for candidate in [
        "ros_before_sleep_sec",
        "ros_update_start_sec",
        "ros_update_complete_sec",
    ]:
        if candidate in fieldnames:
            timestamp_key = candidate
            break

    selected = rows
    scope = "all_rows"
    if timestamp_key and goal_time is not None and result_time is not None:
        mission_rows = []
        for row in rows:
            timestamp = finite_number(row.get(timestamp_key))
            if timestamp is not None and goal_time <= timestamp <= result_time:
                mission_rows.append(row)
        if mission_rows:
            selected = mission_rows
            scope = "goal_to_result"
        else:
            warnings.append(
                f"No timing rows aligned with goal-result window for {run['run_id']}; using all rows"
            )
            scope = "all_rows_fallback"

    measured_delay_ms = [
        value / 1000.0
        for value in valid_nonnegative(row.get("measured_delay_us") for row in selected)
    ]
    delay_error_ms = [
        value / 1000.0
        for value in [finite_number(row.get("delay_error_us")) for row in selected]
        if value is not None
    ]
    requested_values_us = valid_nonnegative(row.get("requested_delay_us") for row in selected)
    requested_delay_ms_csv = median_or_none([value / 1000.0 for value in requested_values_us])

    result = {
        "experiment": run["experiment"],
        "run_id": run["run_id"],
        "delay_ms": run["delay_ms"],
        "timing_file": str(timing_path),
        "timing_scope": scope,
        "timing_event_count_total": len(rows),
        "timing_event_count_used": len(selected),
        "requested_delay_ms_from_csv": requested_delay_ms_csv,
    }
    result.update(summarise_numeric(measured_delay_ms, "measured_delay_ms"))
    # Error can legitimately be slightly negative, so do not use nonnegative filter.
    error_values = [value for value in delay_error_ms if math.isfinite(value)]
    result.update({
        "delay_error_ms_mean": mean_or_none(error_values),
        "delay_error_ms_median": median_or_none(error_values),
        "delay_error_ms_min": min_or_none(error_values),
        "delay_error_ms_max": max_or_none(error_values),
    })

    if is_e2:
        intervals = valid_nonnegative(row.get("cycle_interval_ms") for row in selected)
        displacements = [
            value for value in [finite_number(row.get("cycle_displacement_ms")) for row in selected]
            if value is not None and value != -1.0
        ]
        result.update(summarise_numeric(intervals, "cycle_interval_ms"))
        result.update({
            "cycle_displacement_ms_mean": mean_or_none(displacements),
            "cycle_displacement_ms_median": median_or_none(displacements),
            "cycle_displacement_ms_stdev": stdev_or_none(displacements),
            "cycle_displacement_ms_min": min_or_none(displacements),
            "cycle_displacement_ms_p95": percentile(displacements, 0.95),
            "cycle_displacement_ms_max": max_or_none(displacements),
        })
        result["internal_interval_median_ms"] = median_or_none(intervals)
        result["internal_interval_max_ms"] = max_or_none(intervals)
        result["internal_displacement_median_ms"] = median_or_none(displacements)
        result["internal_displacement_max_ms"] = max_or_none(displacements)

    if is_e3:
        update_duration = valid_nonnegative(row.get("update_duration_ms") for row in selected)
        intervals = valid_nonnegative(row.get("update_completion_interval_ms") for row in selected)
        displacements = [
            value for value in [finite_number(row.get("update_temporal_displacement_ms")) for row in selected]
            if value is not None and value != -1.0
        ]
        previous_age = valid_nonnegative(
            row.get("previous_update_age_before_refresh_ms") for row in selected
        )
        result.update(summarise_numeric(update_duration, "update_duration_ms"))
        result.update(summarise_numeric(intervals, "update_completion_interval_ms"))
        result.update({
            "update_displacement_ms_mean": mean_or_none(displacements),
            "update_displacement_ms_median": median_or_none(displacements),
            "update_displacement_ms_stdev": stdev_or_none(displacements),
            "update_displacement_ms_min": min_or_none(displacements),
            "update_displacement_ms_p95": percentile(displacements, 0.95),
            "update_displacement_ms_max": max_or_none(displacements),
        })
        result.update(summarise_numeric(previous_age, "previous_update_age_ms"))
        result["internal_interval_median_ms"] = median_or_none(intervals)
        result["internal_interval_max_ms"] = max_or_none(intervals)
        result["internal_displacement_median_ms"] = median_or_none(displacements)
        result["internal_displacement_max_ms"] = max_or_none(displacements)

    return result


def parse_log(log_path):
    if log_path is None or not log_path.exists():
        return {
            "log_available": "no",
            "control_loop_warning_count": None,
            "map_update_warning_count": None,
            "total_warn_error_lines": None,
            "max_reported_loop_time_s": None,
        }

    # I file esistono, ma la registrazione si interrompe prima
    # degli eventi di interesse. Non vengono usati per
    # quantificare warning o deadline misses.
    return {
        "log_available": "incomplete",
        "control_loop_warning_count": None,
        "map_update_warning_count": None,
        "total_warn_error_lines": None,
        "max_reported_loop_time_s": None,
    }


def merge_rows(navigation, timing, manual, log_info):
    row = dict(navigation)
    if timing:
        row.update(timing)
    if manual:
        for key, value in manual.items():
            if key == "run_id":
                continue
            row[f"manual_{key}"] = value
    row.update(log_info)
    return row


def is_yes(value):
    return str(value or "").strip().lower() in {"yes", "true", "1", "y"}


def result_is_success(row):
    bag_result = str(row.get("result_from_bag") or "").upper()
    manual_result = str(row.get("manual_result") or "").upper()
    return bag_result == "SUCCEEDED" or manual_result in {"SUCCESS", "SUCCEEDED"}


def summarise_condition(rows):
    first = rows[0]
    summary = {
        "experiment": first["experiment"],
        "delay_ms": first["delay_ms"],
        "n": len(rows),
        "success_count": sum(1 for row in rows if result_is_success(row)),
        "collision_count": sum(1 for row in rows if is_yes(row.get("manual_collision"))),
        "recovery_count": sum(1 for row in rows if is_yes(row.get("manual_recovery_observed"))),
        "timeout_count": sum(1 for row in rows if is_yes(row.get("manual_timeout"))),
        "log_available_count": sum(1 for row in rows if row.get("log_available") == "yes"),
    }
    metric_names = [
        "mission_duration_s",
        "observation_duration_s",
        "odom_path_length_m",
        "minimum_scan_m",
        "mean_abs_linear_cmd_m_s",
        "mean_abs_angular_cmd_rad_s",
        "stop_command_fraction",
        "median_cmd_gap_ms",
        "p95_cmd_gap_ms",
        "max_cmd_gap_ms",
        "startup_path_length_5s_m",
        "startup_net_displacement_5s_m",
        "startup_efficiency_5s",
        "startup_angular_sign_changes_5s",
        "startup_reverse_cmd_count_5s",
        "startup_mean_abs_angular_cmd_rad_s",
        "time_to_first_motion_s",
        "measured_delay_ms_median",
        "internal_interval_median_ms",
        "internal_displacement_median_ms",
        "previous_update_age_ms_median",
        "control_loop_warning_count",
        "map_update_warning_count",
    ]
    for metric in metric_names:
        values = [
            number for number in (finite_number(row.get(metric)) for row in rows)
            if number is not None
        ]
        
        summary[f"{metric}__n"] = len(values)
        summary[f"{metric}__median"] = median_or_none(values)
        summary[f"{metric}__min"] = min_or_none(values)
        summary[f"{metric}__max"] = max_or_none(values)
        summary[f"{metric}__mean"] = mean_or_none(values)
        summary[f"{metric}__stdev"] = stdev_or_none(values)
    return summary


def build_vs_zero(condition_rows):
    by_experiment = defaultdict(dict)
    for row in condition_rows:
        by_experiment[row["experiment"]][int(row["delay_ms"])] = row
    output = []
    metrics = [
        "mission_duration_s__median",
        "odom_path_length_m__median",
        "minimum_scan_m__median",
        "stop_command_fraction__median",
        "median_cmd_gap_ms__median",
        "max_cmd_gap_ms__median",
        "startup_efficiency_5s__median",
        "startup_angular_sign_changes_5s__median",
        "startup_reverse_cmd_count_5s__median",
    ]
    for experiment, delay_map in sorted(by_experiment.items()):
        if experiment == "E1" or 0 not in delay_map:
            continue
        zero = delay_map[0]
        for delay, row in sorted(delay_map.items()):
            comparison = {
                "experiment": experiment,
                "delay_ms": delay,
                "n": row["n"],
                "reference_delay_ms": 0,
            }
            for metric in metrics:
                value = finite_number(row.get(metric))
                reference = finite_number(zero.get(metric))
                base_name = metric.replace("__median", "")
                comparison[f"{base_name}_absolute_change"] = (
                    value - reference if value is not None and reference is not None else None
                )
                comparison[f"{base_name}_percent_change"] = (
                    ((value - reference) / reference) * 100.0
                    if value is not None and reference not in (None, 0.0)
                    else None
                )
            output.append(comparison)
    return output


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            "Usage: python3 analyze_all_experiments.py ~/Desktop/project/results"
        )

    results_root = Path(os.path.expanduser(sys.argv[1])).resolve()
    if not results_root.exists():
        raise SystemExit(f"Results directory does not exist: {results_root}")

    analysis_dir = results_root / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    warnings = []

    manual_runs = load_manual_runs(results_root, warnings)
    runs = discover_runs(results_root, warnings)
    if not runs:
        raise SystemExit("No official bag files were discovered.")

    navigation_rows = []
    e2_timing_rows = []
    e3_timing_rows = []
    master_rows = []

    for index, run in enumerate(runs, start=1):
        print(f"[{index}/{len(runs)}] {run['run_id']}")
        try:
            navigation = analyse_bag(run, warnings)
        except AnalysisError as exc:
            warnings.append(str(exc))
            continue

        timing_path = find_timing_csv(run, results_root, warnings)
        timing = aggregate_timing(
            run,
            timing_path,
            navigation.get("goal_ros_time_s"),
            navigation.get("result_ros_time_s"),
            warnings,
        )
        log_path = find_log(run, results_root, warnings)
        log_info = parse_log(log_path)
        manual = manual_runs.get(run["run_id"])
        if manual is None:
            warnings.append(f"No manual runs.csv row for {run['run_id']}")

        navigation_rows.append(navigation)
        if timing and run["experiment"] == "E2":
            e2_timing_rows.append(timing)
        if timing and run["experiment"] == "E3":
            e3_timing_rows.append(timing)
        master_rows.append(merge_rows(navigation, timing, manual, log_info))

    condition_groups = defaultdict(list)
    for row in master_rows:
        condition_groups[(row["experiment"], int(row["delay_ms"]))].append(row)
    condition_rows = [
        summarise_condition(rows)
        for _, rows in sorted(condition_groups.items())
    ]
    vs_zero_rows = build_vs_zero(condition_rows)

    write_csv(analysis_dir / "navigation_all_runs.csv", navigation_rows)
    write_csv(analysis_dir / "E2_timing_runs.csv", e2_timing_rows)
    write_csv(analysis_dir / "E3_timing_runs.csv", e3_timing_rows)
    write_csv(analysis_dir / "all_runs_master.csv", master_rows)
    write_csv(analysis_dir / "condition_summary.csv", condition_rows)
    write_csv(analysis_dir / "condition_vs_zero.csv", vs_zero_rows)

    warning_path = analysis_dir / "analysis_warnings.txt"
    warning_path.write_text(
        "\n".join(warnings) + ("\n" if warnings else "No warnings.\n"),
        encoding="utf-8",
    )

    print("\nCreated:")
    for name in [
        "navigation_all_runs.csv",
        "E2_timing_runs.csv",
        "E3_timing_runs.csv",
        "all_runs_master.csv",
        "condition_summary.csv",
        "condition_vs_zero.csv",
        "analysis_warnings.txt",
    ]:
        print(f"  {analysis_dir / name}")
    print(f"\nAnalysed {len(master_rows)} official runs.")
    if warnings:
        print(f"Warnings: {len(warnings)} (inspect analysis_warnings.txt)")


if __name__ == "__main__":
    main()
