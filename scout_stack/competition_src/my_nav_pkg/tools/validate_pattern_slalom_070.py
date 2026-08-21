#!/usr/bin/env python3
"""Deterministic 2-D validation for the 0.50-0.70 m/s official slalom.

This exercises the same ROS-independent planner and ray-cast closed-loop plant
used by the unit tests. It deliberately separates:

* exact-course trials: the two official layouts, 2.50 m clear box gap;
* placement-stress trials: randomized small box placement errors;
* boundary-stress trials: deterministic combinations chosen to push the
  inflated box and road-boundary margins in both steering directions.

The reported percentages are simulation scenario pass rates, not measured
real-vehicle probabilities.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import random
import statistics
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
TEST_FILE = ROOT / "test" / "test_vfh_closed_loop.py"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

spec = importlib.util.spec_from_file_location("vfh_closed_loop_support", TEST_FILE)
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load {TEST_FILE}")
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
ClosedLoopHarness = module.ClosedLoopHarness
SimulationResult = module.SimulationResult

Point = Tuple[float, float]

LANE_DIVIDER_Y = 0.625
BOX_WIDTH = 0.90
BOX_HALF_WIDTH = 0.5 * BOX_WIDTH
UPPER_FLUSH_Y = LANE_DIVIDER_Y + BOX_HALF_WIDTH
LOWER_FLUSH_Y = LANE_DIVIDER_Y - BOX_HALF_WIDTH
# Worst nominal lateral placement: every box touches the lane divider. The
# diagram's optional 0.20 m outward offset would increase clearance.
PATTERNS: Dict[str, Sequence[float]] = {
    "ULU": (UPPER_FLUSH_Y, LOWER_FLUSH_Y, UPPER_FLUSH_Y),
    "LUL": (LOWER_FLUSH_Y, UPPER_FLUSH_Y, LOWER_FLUSH_Y),
}
BASE_X = (3.0, 6.0, 9.0)  # 3.00 m centre spacing = 2.50 m clear + 0.50 m depth


def _boxes(
    pattern: Sequence[float],
    rng: random.Random,
    placement_stress: bool,
) -> List[Point]:
    if not placement_stress:
        return list(zip(BASE_X, pattern))
    return [
        (
            x + rng.uniform(-0.08, 0.08),
            y + rng.uniform(-0.04, 0.04),
        )
        for x, y in zip(BASE_X, pattern)
    ]


def _scenario_parameters(
    rng: random.Random,
    placement_stress: bool,
) -> Dict[str, float]:
    if placement_stress:
        return {
            "initial_lateral": rng.uniform(-0.08, 0.08),
            "initial_yaw": rng.uniform(math.radians(-3.0), math.radians(3.0)),
            "actual_yaw_gain": rng.uniform(1.10, 1.52),
            "angular_tau": rng.uniform(0.05, 0.22),
            "linear_tau": rng.uniform(0.05, 0.20),
            "command_delay": rng.choice((0.0, 0.05, 0.10)),
            "linear_scale": rng.uniform(0.95, 1.05),
            "pose_lateral_bias": rng.uniform(-0.03, 0.03),
            "pose_yaw_bias": rng.uniform(
                math.radians(-1.0), math.radians(1.0)
            ),
        }
    return {
        "initial_lateral": rng.uniform(-0.06, 0.06),
        "initial_yaw": rng.uniform(math.radians(-2.5), math.radians(2.5)),
        "actual_yaw_gain": rng.uniform(1.15, 1.47),
        "angular_tau": rng.uniform(0.05, 0.18),
        "linear_tau": rng.uniform(0.05, 0.18),
        "command_delay": rng.choice((0.0, 0.05, 0.10)),
        "linear_scale": rng.uniform(0.96, 1.04),
        "pose_lateral_bias": rng.uniform(-0.02, 0.02),
        "pose_yaw_bias": rng.uniform(
            math.radians(-0.8), math.radians(0.8)
        ),
    }


def _boundary_stress_cases(
    pattern: Sequence[float],
) -> List[Tuple[List[Point], Dict[str, float]]]:
    """Deterministic harsh cases for both obstacle and road margins.

    The obstacle rows are shifted toward the selected pass lines by 4 cm.
    Longitudinal offsets alternate by 8 cm so the controller cannot rely on
    perfectly uniform placement. Plant profiles span low/high yaw response,
    actuator lag, 100 ms command delay, speed-scale error, and pose bias.
    These are engineering stress assumptions, not official placement
    tolerances or measured probability distributions.
    """
    shifted_pattern = [
        y - 0.04 if y > 0.625 else y + 0.04
        for y in pattern
    ]
    longitudinal_profiles = (
        (0.0, 0.0, 0.0),
        (0.08, -0.08, 0.08),
        (-0.08, 0.08, -0.08),
    )
    plant_profiles = (
        (-0.08, -3.0, 1.10, 0.22, 0.20, 0.10, 1.05, 0.03, 1.0),
        (0.08, 3.0, 1.52, 0.22, 0.20, 0.10, 1.05, -0.03, -1.0),
        (-0.08, 3.0, 1.52, 0.05, 0.05, 0.10, 0.95, 0.03, -1.0),
        (0.08, -3.0, 1.10, 0.05, 0.05, 0.10, 0.95, -0.03, 1.0),
        (0.0, -3.0, 1.10, 0.22, 0.12, 0.10, 1.00, 0.03, 1.0),
        (0.0, 3.0, 1.52, 0.22, 0.12, 0.10, 1.00, -0.03, -1.0),
    )
    cases: List[Tuple[List[Point], Dict[str, float]]] = []
    for offsets in longitudinal_profiles:
        boxes = [
            (x + dx, y)
            for x, dx, y in zip(BASE_X, offsets, shifted_pattern)
        ]
        for (
            initial_lateral,
            initial_yaw_deg,
            actual_yaw_gain,
            angular_tau,
            linear_tau,
            command_delay,
            linear_scale,
            pose_lateral_bias,
            pose_yaw_bias_deg,
        ) in plant_profiles:
            cases.append(
                (
                    list(boxes),
                    {
                        "initial_lateral": initial_lateral,
                        "initial_yaw": math.radians(initial_yaw_deg),
                        "actual_yaw_gain": actual_yaw_gain,
                        "angular_tau": angular_tau,
                        "linear_tau": linear_tau,
                        "command_delay": command_delay,
                        "linear_scale": linear_scale,
                        "pose_lateral_bias": pose_lateral_bias,
                        "pose_yaw_bias": math.radians(pose_yaw_bias_deg),
                    },
                )
            )
    return cases


def _failure_reasons(result: SimulationResult) -> List[str]:
    reasons: List[str] = []
    if not result.completed:
        reasons.append("incomplete")
    if result.side_none_count:
        reasons.append("target_none")
    if result.planner_none_count:
        reasons.append("planner_none")
    if result.physical_collision_count:
        reasons.append("physical_collision")
    if result.inflated_collision_count:
        reasons.append("inflated_collision")
    if result.corridor_violation_count:
        reasons.append("corridor_violation")
    if result.pattern_min_speed and result.pattern_min_speed < 0.50 - 1e-6:
        reasons.append("below_0.50")
    if result.pattern_max_speed > 0.70 + 1e-6:
        reasons.append("above_0.70")
    if result.pattern_max_abs_angular > 1.00 + 1e-6:
        reasons.append("above_1.00_radps")
    return reasons


def _run_one(
    harness: Any,
    pattern_name: str,
    group: str,
    scenario_index: int,
    boxes: Sequence[Point],
    params: Dict[str, float],
    trajectory_log: List[Tuple[float, float, float]] | None = None,
) -> Dict[str, Any]:
    result = harness.run(
        boxes,
        initial_pose=(0.0, params["initial_lateral"], params["initial_yaw"]),
        actual_yaw_response_gain=params["actual_yaw_gain"],
        angular_time_constant_sec=params["angular_tau"],
        linear_time_constant_sec=params["linear_tau"],
        command_delay_sec=params["command_delay"],
        linear_speed_scale=params["linear_scale"],
        scan_period_sec=0.10,
        pose_lateral_bias=params["pose_lateral_bias"],
        pose_yaw_bias=params["pose_yaw_bias"],
        trajectory_log=trajectory_log,
        max_duration_sec=35.0,
    )
    reasons = _failure_reasons(result)
    row: Dict[str, Any] = {
        "group": group,
        "pattern": pattern_name,
        "scenario_index": scenario_index,
        "passed": not reasons,
        "failure_reasons": ";".join(reasons),
        "boxes": json.dumps(boxes),
        **params,
        **asdict(result),
    }
    return row


def _quantile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _summarize(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    summary: Dict[str, Any] = {}
    for group in ("nominal", "exact_uncertainty", "placement_stress", "boundary_stress"):
        selected = [row for row in rows if row["group"] == group]
        if not selected:
            continue
        passed = [row for row in selected if row["passed"]]
        failure_counts: Dict[str, int] = {}
        for row in selected:
            for reason in str(row["failure_reasons"]).split(";"):
                if reason:
                    failure_counts[reason] = failure_counts.get(reason, 0) + 1
        margins = [float(row["minimum_inflated_sat_margin"]) for row in selected]
        corridor = [float(row["minimum_corridor_margin"]) for row in selected]
        averages = [float(row["pattern_average_speed"]) for row in selected]
        summary[group] = {
            "trials": len(selected),
            "passes": len(passed),
            "failures": len(selected) - len(passed),
            "scenario_pass_rate": len(passed) / len(selected),
            "failure_counts": failure_counts,
            "inflated_sat_margin_min_m": min(margins),
            "inflated_sat_margin_p05_m": _quantile(margins, 0.05),
            "corridor_margin_min_m": min(corridor),
            "corridor_margin_p05_m": _quantile(corridor, 0.05),
            "average_command_speed_mean_mps": statistics.fmean(averages),
            "average_command_speed_p05_mps": _quantile(averages, 0.05),
        }
    return summary


def _plot_nominal(
    output_dir: Path,
    pattern_name: str,
    boxes: Sequence[Point],
    trajectory: Sequence[Tuple[float, float, float]],
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception:
        return

    figure, axis = plt.subplots(figsize=(10, 4.5))
    if trajectory:
        axis.plot(
            [pose[0] for pose in trajectory],
            [pose[1] for pose in trajectory],
            label="vehicle centre",
        )
    for index, (box_x, box_y) in enumerate(boxes, start=1):
        axis.add_patch(
            Rectangle(
                (box_x - 0.25, box_y - 0.45),
                0.50,
                0.90,
                fill=False,
                label="0.50 x 0.90 m box" if index == 1 else None,
            )
        )
    axis.axhline(2.025, linestyle="--", label="left road boundary")
    axis.axhline(-0.675, linestyle="--", label="right road boundary")
    axis.axhline(0.0, linestyle=":", label="recorded waypoint path")
    axis.axhline(
        LANE_DIVIDER_Y,
        linestyle="-.",
        label="lane divider (boxes flush in nominal model)",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(-0.5, 13.0)
    axis.set_ylim(-1.0, 2.35)
    axis.set_xlabel("forward in path frame (m)")
    axis.set_ylabel("lateral in path frame (m)")
    axis.set_title(f"2-D nominal official slalom: {pattern_name}")
    axis.grid(True)
    axis.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(output_dir / f"pattern_slalom_nominal_{pattern_name}.png", dpi=180)
    plt.close(figure)


def _write_markdown(
    path: Path,
    summary: Dict[str, Any],
    seed: int,
    samples_per_pattern: int,
    rows: Sequence[Dict[str, Any]],
) -> None:
    lines = [
        "# Pattern Slalom 0.70 m/s — 2-D Validation",
        "",
        f"- Random seed: `{seed}`",
        f"- Random samples per pattern and group: `{samples_per_pattern}`",
        "- Course geometry: 0.50 m box depth, 2.50 m clear gap, therefore 3.00 m centre spacing.",
        "- Lateral worst case: 0.90 m boxes touch the lane divider; no 0.20 m outward gap is assumed.",
        "- Vehicle envelope used by planner and SAT checks: 1.40 m × 0.65 m.",
        "- LiDAR FOV: 200°, scan update: 10 Hz, controller update: 20 Hz.",
        "- Command range after pattern lock: 0.50–0.70 m/s and |ω| ≤ 1.00 rad/s.",
        "",
        "> The pass rate below is a deterministic simulation-scenario result, not a measured real-vehicle probability.",
        "",
        "## Results",
        "",
        "| Group | Trials | Passes | Failures | Scenario pass rate | Min inflated SAT margin | P05 inflated margin | Min road margin | Mean commanded speed |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for group, data in summary.items():
        lines.append(
            "| {group} | {trials} | {passes} | {failures} | {rate:.1%} | "
            "{sat_min:.3f} m | {sat_p05:.3f} m | {road_min:.3f} m | "
            "{speed:.3f} m/s |".format(
                group=group,
                trials=data["trials"],
                passes=data["passes"],
                failures=data["failures"],
                rate=data["scenario_pass_rate"],
                sat_min=data["inflated_sat_margin_min_m"],
                sat_p05=data["inflated_sat_margin_p05_m"],
                road_min=data["corridor_margin_min_m"],
                speed=data["average_command_speed_mean_mps"],
            )
        )
    lines.extend(["", "## Assumed uncertainty envelope", ""])
    lines.extend(
        [
            "Exact-course trials keep both layouts and every box at the measured coordinates, while varying initial lateral/yaw error, actual yaw response, actuator lag, 0–100 ms command delay, speed scale, and small localization bias.",
            "",
            "Placement-stress trials additionally perturb each box by up to ±0.08 m longitudinally and ±0.04 m laterally. This is a stress envelope; the official source only fixes the two layouts and nominal spacing, not these tolerances.",
            "",
            "Boundary-stress trials deterministically move both obstacle rows 4 cm toward their pass lines, alternate longitudinal placement by 8 cm, and combine the endpoints of the modeled yaw response, lag, delay, speed-scale, and pose-bias ranges. They are designed to expose low-margin cases rather than estimate frequency.",
            "",
            "## Failure examples",
            "",
        ]
    )
    failures = [row for row in rows if not row["passed"]][:10]
    if not failures:
        lines.append("No failures occurred in the configured scenario set.")
    else:
        for row in failures:
            lines.append(
                "- `{group}/{pattern}/{scenario_index}`: {failure_reasons}; "
                "inflated margin={minimum_inflated_sat_margin:.3f} m, "
                "road margin={minimum_corridor_margin:.3f} m, "
                "yaw gain={actual_yaw_gain:.2f}, lag={angular_tau:.3f}s, "
                "delay={command_delay:.2f}s".format(**row)
            )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "Passing nominal and exact-course uncertainty trials supports the geometry and controller coupling at 0.70 m/s. Placement-stress failures, when present, identify sensitivity to real box placement or lane-frame measurement rather than proof that the official nominal layout fails.",
            "",
            "A real vehicle still adds tire scrub, surface-dependent yaw gain, timestamp skew, LiDAR clustering bias, GNSS/heading error, chassis dimensions, and actuator saturation. Field deployment should therefore progress through 0.50, 0.60, then 0.70 m/s, retaining the same path and measuring minimum box/road margin on every run.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples-per-pattern", type=int, default=120)
    parser.add_argument("--seed", type=int, default=20260821)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "validation",
    )
    args = parser.parse_args()
    if args.samples_per_pattern < 1:
        parser.error("--samples-per-pattern must be positive")
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    harness = ClosedLoopHarness(
        fov_deg=200.0,
        road_upper=2.025,
        road_lower=-0.675,
        use_asymmetric_config=True,
        use_pattern_slalom=True,
    )
    rng = random.Random(args.seed)
    rows: List[Dict[str, Any]] = []

    nominal_params = {
        "initial_lateral": 0.0,
        "initial_yaw": 0.0,
        "actual_yaw_gain": 1.31,
        "angular_tau": 0.10,
        "linear_tau": 0.12,
        "command_delay": 0.05,
        "linear_scale": 1.0,
        "pose_lateral_bias": 0.0,
        "pose_yaw_bias": 0.0,
    }
    print("[validation] nominal trials", flush=True)
    for pattern_name, pattern in PATTERNS.items():
        boxes = list(zip(BASE_X, pattern))
        trajectory: List[Tuple[float, float, float]] = []
        rows.append(
            _run_one(
                harness,
                pattern_name,
                "nominal",
                0,
                boxes,
                nominal_params,
                trajectory,
            )
        )
        _plot_nominal(output_dir, pattern_name, boxes, trajectory)

    for group, placement_stress in (
        ("exact_uncertainty", False),
        ("placement_stress", True),
    ):
        print(f"[validation] {group} trials", flush=True)
        for pattern_name, pattern in PATTERNS.items():
            for scenario_index in range(args.samples_per_pattern):
                if scenario_index % 10 == 0:
                    print(
                        f"[validation] {group}/{pattern_name} "
                        f"{scenario_index}/{args.samples_per_pattern}",
                        flush=True,
                    )
                params = _scenario_parameters(rng, placement_stress)
                boxes = _boxes(pattern, rng, placement_stress)
                rows.append(
                    _run_one(
                        harness,
                        pattern_name,
                        group,
                        scenario_index,
                        boxes,
                        params,
                    )
                )

    print("[validation] boundary_stress trials", flush=True)
    for pattern_name, pattern in PATTERNS.items():
        for scenario_index, (boxes, params) in enumerate(
            _boundary_stress_cases(pattern)
        ):
            if scenario_index % 6 == 0:
                print(
                    f"[validation] boundary_stress/{pattern_name} "
                    f"{scenario_index}/18",
                    flush=True,
                )
            rows.append(
                _run_one(
                    harness,
                    pattern_name,
                    "boundary_stress",
                    scenario_index,
                    boxes,
                    params,
                )
            )

    print("[validation] writing reports", flush=True)
    summary = _summarize(rows)
    summary_payload = {
        "seed": args.seed,
        "samples_per_pattern": args.samples_per_pattern,
        "geometry": {
            "clear_gap_m": 2.50,
            "box_depth_m": 0.50,
            "centre_spacing_m": 3.00,
            "box_width_m": BOX_WIDTH,
            "lane_divider_y_m": LANE_DIVIDER_Y,
            "box_to_lane_divider_gap_m": 0.0,
            "vehicle_length_m": 1.40,
            "vehicle_width_m": 0.65,
            "road_left_m": 2.025,
            "road_right_m": 0.675,
        },
        "limits": {
            "minimum_speed_mps": 0.50,
            "maximum_speed_mps": 0.70,
            "maximum_angular_speed_radps": 1.00,
        },
        "summary": summary,
    }
    (output_dir / "pattern_slalom_2d_summary.json").write_text(
        json.dumps(summary_payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    csv_path = output_dir / "pattern_slalom_2d_scenarios.csv"
    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    _write_markdown(
        output_dir / "pattern_slalom_2d_report.md",
        summary,
        args.seed,
        args.samples_per_pattern,
        rows,
    )

    print(json.dumps(summary_payload, indent=2, ensure_ascii=False))
    return 0 if all(
        data["failures"] == 0
        for name, data in summary.items()
        if name in ("nominal", "exact_uncertainty", "boundary_stress")
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
