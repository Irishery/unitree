#!/usr/bin/env python3
"""Run the reproducible RGB-D bimanual tabletop benchmark and write JSON logs."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import re
import subprocess
import sys


TRIALS = [
    (0.380, 0.000, 0.00), (0.390, 0.000, 0.00), (0.400, 0.000, 0.00),
    (0.385, 0.005, 0.01), (0.395, -0.005, -0.01),
    (0.380, 0.010, 0.02), (0.390, -0.010, -0.02),
    (0.390, 0.015, 0.03), (0.395, -0.015, -0.03),
    (0.400, 0.000, 0.05),
]


def values(line):
    return [float(value) for value in re.findall(r"[-+]?\d+(?:\.\d+)?", line)]


def run_one(index, config, root):
    x, y, yaw = config
    directory = root / f"trial_{index:02d}"
    directory.mkdir(parents=True, exist_ok=True)
    probe = Path(__file__).with_name("probe_grasp_frames.py")
    command = [sys.executable, str(probe), "-0.035", "0.02", "0.16", "0", "1.0",
               str(yaw), str(x), str(y)]
    environment = os.environ.copy()
    environment["G1_PROBE_OUT"] = str(directory / "frames")
    process = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, env=environment, check=False)
    (directory / "run.log").write_text(process.stdout, encoding="utf-8")
    result = {"trial": index, "x": x, "y": y, "yaw": yaw,
              "returncode": process.returncode, "success": False}
    try:
        rgb = next(line for line in process.stdout.splitlines() if line.startswith("RGBD detected"))
        hold = next(line for line in process.stdout.splitlines() if line.startswith("HOLD box"))
        final = next(line for line in process.stdout.splitlines() if line.startswith("FINAL box"))
        rv, hv, fv = values(rgb), values(hold), values(final)
        result.update({
            "position_error_m": rv[-2], "yaw_error_deg": rv[-1],
            "hold_start_z": hv[0], "hold_min_z": hv[1], "hold_end_z": hv[2],
            "hold_slip_m": hv[3], "hold_tilt_max_deg": hv[4],
            "hold_tilt_end_deg": hv[5], "max_z": fv[0], "placed_z": fv[2],
        })
        failures = []
        if fv[0] < 0.900: failures.append("lift_below_10cm")
        if hv[1] < 0.900: failures.append("hold_below_10cm")
        if hv[3] > 0.040: failures.append("visible_slip_over_4cm")
        if hv[4] > 8.0: failures.append("box_tilt_over_8deg")
        if abs(fv[2] - 0.800) > 0.012: failures.append("not_stably_placed")
        result["failures"] = failures
        result["success"] = not failures
    except (StopIteration, IndexError, ValueError) as error:
        detail = process.stdout.strip().splitlines()[-1] if process.stdout.strip() else str(error)
        result["failures"] = [f"runner_error:{detail}"]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("/ws/src/g1_mujoco/artifacts/pick_trials"))
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {pool.submit(run_one, index, config, args.output): index
                   for index, config in enumerate(TRIALS, 1)}
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(json.dumps(result, sort_keys=True), flush=True)
    results.sort(key=lambda item: item["trial"])
    summary = {"trials": len(results), "successes": sum(r["success"] for r in results),
               "results": results}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"SUMMARY {summary['successes']}/{summary['trials']} -> {args.output / 'summary.json'}")
    return 0 if summary["successes"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
