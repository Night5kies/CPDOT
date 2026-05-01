#!/usr/bin/env python3
"""
Run the extension ablation as one ROS process per trial.

Default settings are chosen to match the paper's comprehensive-analysis
benchmark as closely as this extension study allows:
- 60 m x 60 m world with formation-center start/goal fixed in topologic_pp
- one configurable obstacle count, default 60
- 100 random instances per configuration
- 180 second per-instance timeout
- four extension configurations: baseline, A, B, and A+B

Usage:
    python3 run_extension_ablation.py
    python3 run_extension_ablation.py --n-trials 100 --num-obs 60 --timeout-s 180
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple


CONFIGS: List[Tuple[str, str, str]] = [
    ("baseline", "0", "0"),
    ("ext_A", "1", "0"),
    ("ext_B", "0", "1"),
    ("ext_AB", "1", "1"),
]

FIELDNAMES = [
    "num_robot",
    "opti_t",
    "opti_w_a",
    "opti_w_omega",
    "opti_w_penalty0",
    "opti_w_formation",
    "opti_w_topo",
    "opti_w_sfc",
    "seed",
    "trial",
    "trial_success",
    "solve_success",
    "success_strict",
    "success_any_solution",
    "success_feasible",
    "final_infeasibility",
    "n_solutions",
    "time_topo_s",
    "time_filter_s",
    "time_coarse_s",
    "time_to_s",
    "time_total_s",
    "outer_iters",
    "inner_iters",
    "tf_s",
    "distance_m",
    "config_timed_out",
]

DEFAULT_ROSPARAMS = {
    "n_trials": 1,
    "opti_t": 1.0,
    "opti_w_a": 1.0,
    "opti_w_omega": 1.0,
    "opti_w_penalty0": 10000.0,
    "opti_w_formation": 1.0,
    "opti_w_topo": 1.0,
    "opti_w_sfc": 1.0,
    "opti_inner_iter_max": 30,
    "topo_prm/max_sample_num": 1500,
    "solver_initial_noise_stddev": 1e-3,
}


def make_failure_row(seed: int, trial: int, timed_out: bool) -> Dict[str, object]:
    return {
        "num_robot": "?",
        "opti_t": 1.0,
        "opti_w_a": 1.0,
        "opti_w_omega": 1.0,
        "opti_w_penalty0": 10000.0,
        "opti_w_formation": 1.0,
        "opti_w_topo": 1.0,
        "opti_w_sfc": 1.0,
        "seed": seed,
        "trial": trial,
        "trial_success": 0,
        "solve_success": 0,
        "success_strict": 0,
        "success_any_solution": 0,
        "success_feasible": 0,
        "final_infeasibility": "nan",
        "n_solutions": 0,
        "time_topo_s": -1,
        "time_filter_s": -1,
        "time_coarse_s": -1,
        "time_to_s": -1,
        "time_total_s": -1,
        "outer_iters": -1,
        "inner_iters": -1,
        "tf_s": -1,
        "distance_m": -1,
        "config_timed_out": int(timed_out),
    }


def set_rosparam(name: str, value: object) -> None:
    subprocess.run(
        ["rosparam", "set", f"/{name}", str(value)],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def ros_master_running() -> bool:
    probe = subprocess.run(
        ["rosparam", "get", "/rosversion"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return probe.returncode == 0


def wait_for_ros_master(timeout_s: int) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ros_master_running():
            return True
        time.sleep(0.2)
    return False


def ensure_roscore() -> Optional[subprocess.Popen]:
    if ros_master_running():
        return None

    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    if wait_for_ros_master(timeout_s=10):
        return proc

    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    raise RuntimeError("Timed out waiting for roscore to start.")


def stop_process_group(proc: subprocess.Popen, sig: signal.Signals) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except ProcessLookupError:
        pass


def run_trial(
    package_csv: Path,
    analytical_lift: str,
    failure_pruning: str,
    seed: int,
    num_obs: int,
    timeout_s: int,
) -> Tuple[Dict[str, object], int, str]:
    if package_csv.exists():
        package_csv.unlink()

    for name, value in DEFAULT_ROSPARAMS.items():
        set_rosparam(name, value)
    set_rosparam("num_obs", num_obs)
    set_rosparam("random_seed", seed)
    set_rosparam("topo_prm_seed", seed)

    env = os.environ.copy()
    env["CPDOT_ANALYTICAL_LIFT"] = analytical_lift
    env["CPDOT_FAILURE_PRUNING"] = failure_pruning

    proc = subprocess.Popen(
        ["rosrun", "formation_planner", "topologic_pp"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        preexec_fn=os.setsid,
    )

    timed_out = False
    try:
        stdout, _ = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        stop_process_group(proc, signal.SIGTERM)
        try:
            stdout, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            stop_process_group(proc, signal.SIGKILL)
            stdout, _ = proc.communicate()

    row: Dict[str, object]
    if package_csv.exists():
        with open(package_csv, newline="") as csv_file:
            rows = list(csv.DictReader(csv_file))
        if rows:
            row = rows[-1]
            row["config_timed_out"] = int(timed_out)
        else:
            row = make_failure_row(seed=seed, trial=0, timed_out=timed_out)
    else:
        row = make_failure_row(seed=seed, trial=0, timed_out=timed_out)

    return row, proc.returncode, stdout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the CPDOT extension study on the paper's comprehensive-analysis world."
    )
    parser.add_argument("--n-trials", type=int, default=100, help="Problem instances per extension configuration.")
    parser.add_argument("--num-obs", type=int, default=60, help="Cuboid obstacle count for the comprehensive-analysis world.")
    parser.add_argument("--timeout-s", type=int, default=180, help="Per-trial timeout in seconds.")
    parser.add_argument("--base-seed", type=int, default=277, help="Seed for trial 0; later trials use base_seed + trial.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    pkg_dir = Path(__file__).resolve().parents[1]
    result_dir = pkg_dir / "traj_result"
    package_csv = result_dir / "sweep_results.csv"
    roscore_proc = ensure_roscore()
    try:
        print(
            f"[ablation] {len(CONFIGS)} configs x {args.n_trials} trials, "
            f"num_obs={args.num_obs}, timeout={args.timeout_s}s"
        )

        for name, analytical_lift, failure_pruning in CONFIGS:
            output_csv = result_dir / f"{name}.csv"
            if output_csv.exists():
                output_csv.unlink()

            successes = 0
            print(
                f"\n[ablation] {name}: "
                f"CPDOT_ANALYTICAL_LIFT={analytical_lift} "
                f"CPDOT_FAILURE_PRUNING={failure_pruning}"
            )

            with open(output_csv, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES, extrasaction="ignore")
                writer.writeheader()

                for trial in range(args.n_trials):
                    seed = args.base_seed + trial
                    row, return_code, stdout = run_trial(
                        package_csv=package_csv,
                        analytical_lift=analytical_lift,
                        failure_pruning=failure_pruning,
                        seed=seed,
                        num_obs=args.num_obs,
                        timeout_s=args.timeout_s,
                    )
                    row["seed"] = seed
                    row["trial"] = trial
                    writer.writerow(row)
                    csv_file.flush()

                    trial_success = int(row.get("success_feasible", row.get("trial_success", 0)))
                    successes += trial_success
                    timed_out = bool(int(row.get("config_timed_out", 0)))
                    status = "timeout" if timed_out else ("ok" if return_code == 0 else f"exit={return_code}")
                    print(
                        f"  trial {trial + 1}/{args.n_trials}: seed={seed} "
                        f"success={trial_success} status={status}"
                    )
                    if return_code not in (0, None) and not timed_out:
                        tail = "\n".join(stdout.strip().splitlines()[-10:])
                        if tail:
                            print("  recent output:")
                            print(tail)

            rate = 100.0 * successes / args.n_trials if args.n_trials else 0.0
            print(f"[ablation] {name} complete: {successes}/{args.n_trials} feasible ({rate:.1f}%) -> {output_csv}")

        print("\n[ablation] Done.")
    finally:
        if roscore_proc is not None:
            stop_process_group(roscore_proc, signal.SIGTERM)
    return 0


if __name__ == "__main__":
    sys.exit(main())
