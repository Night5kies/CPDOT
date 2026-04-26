#!/usr/bin/env python3
"""
Parameter sweep driver for the formation planner.

Usage:
    python3 sweep.py sweep_config.yaml

Requires: roscore already running in a separate terminal.
Each configuration launches topologic_pp once; the node executes the
requested number of trials internally and writes one CSV row per trial.
"""

import csv
import itertools
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


FAILURE_COLUMNS = [
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


def set_rosparam(name: str, value):
    subprocess.run(
        ["rosparam", "set", f"/{name}", str(value)],
        check=True,
        capture_output=True,
    )


def expand_configs(cfg: dict) -> List[Dict]:
    if "ablation_one_at_a_time" in cfg:
        held = dict(cfg.get("held_constant", {}))
        configs = []
        for sweep_item in cfg["ablation_one_at_a_time"]:
            if len(sweep_item) != 1:
                raise ValueError(
                    "Each ablation_one_at_a_time entry must contain exactly one parameter"
                )
            param_name, values = next(iter(sweep_item.items()))
            for value in values:
                params = dict(held)
                params[param_name] = value
                configs.append(params)
        return configs

    if "param_grid" in cfg:
        grid = cfg["param_grid"]
        keys = list(grid.keys())
        values = list(grid.values())
        return [dict(zip(keys, combo)) for combo in itertools.product(*values)]

    raise ValueError("Config must contain either 'ablation_one_at_a_time' or 'param_grid'")


def make_failure_rows(params: dict, n_trials: int, timed_out: bool) -> List[Dict]:
    seed = params.get("random_seed", 42)
    rows = []
    for trial in range(n_trials):
        row = {
            "num_robot": params.get("num_robot", "?"),
            "opti_t": params.get("opti_t", 1.0),
            "opti_w_a": params.get("opti_w_a", 1.0),
            "opti_w_omega": params.get("opti_w_omega", 1.0),
            "opti_w_penalty0": params.get("opti_w_penalty0", 10000.0),
            "opti_w_formation": params.get("opti_w_formation", 1.0),
            "opti_w_topo": params.get("opti_w_topo", 1.0),
            "opti_w_sfc": params.get("opti_w_sfc", 1.0),
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
        rows.append(row)
    return rows


def launch_config(params: dict, n_trials: int, timeout_s: int, pkg_csv: Path) -> Tuple[List[Dict], bool]:
    if pkg_csv.exists():
        pkg_csv.unlink()

    set_rosparam("n_trials", n_trials)
    for key, value in params.items():
        set_rosparam(key, value)

    proc = subprocess.Popen(
        ["rosrun", "formation_planner", "topologic_pp"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )

    deadline = time.time() + timeout_s
    timed_out = False

    while proc.poll() is None:
        line = proc.stdout.readline()
        if line:
            print("  |", line.rstrip())
        if time.time() > deadline:
            print(f"  [sweep] TIMEOUT after {timeout_s}s — killing")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            timed_out = True
            break
        time.sleep(0.05)

    for line in proc.stdout:
        print("  |", line.rstrip())

    rows = []
    if pkg_csv.exists():
        with open(pkg_csv, newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                row["config_timed_out"] = int(timed_out)
                rows.append(row)
    else:
        print("  [sweep] No CSV output from node — recording failures")
        rows = make_failure_rows(params, n_trials, timed_out=True)

    return rows, timed_out


def write_rows(master_writer, buffered_rows: List[Dict], rows: List[Dict], master_file):
    for row in rows:
        if master_writer is None:
            buffered_rows.append(row)
        else:
            master_writer.writerow(row)
    master_file.flush()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 sweep.py sweep_config.yaml")
        sys.exit(1)

    config_path = Path(sys.argv[1])
    with open(config_path) as config_file:
        cfg = yaml.safe_load(config_file)

    n_trials = cfg.get("n_trials_per_config", 5)
    timeout_s = cfg.get("timeout_per_trial_s", 60) * n_trials + 30
    pkg_result_dir = Path(cfg["pkg_result_dir"])
    pkg_csv = pkg_result_dir / "sweep_results.csv"
    topo_prm_seed = int(cfg.get("topo_prm_seed", 42))
    seed_stride_per_config = int(cfg.get("seed_stride_per_config", 1000))
    offset_random_seed_per_config = bool(cfg.get("offset_random_seed_per_config", True))

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(cfg.get("output_dir", "."))
    output_dir.mkdir(parents=True, exist_ok=True)
    master_path = output_dir / f"sweep_{timestamp}.csv"

    configs = expand_configs(cfg)

    print(
        f"[sweep] {len(configs)} configurations × {n_trials} trials = "
        f"{len(configs) * n_trials} total trials"
    )
    print(f"[sweep] Output → {master_path}")
    print(f"[sweep] Per-config timeout: {timeout_s}s\n")

    total_success = 0
    total_trials = 0

    with open(master_path, "w", newline="") as master_file:
        buffered_rows = []
        fieldnames = None
        master_writer = None

        for config_index, params in enumerate(configs, start=1):
            launch_params = dict(params)
            base_seed = int(launch_params.get("random_seed", 42))
            if offset_random_seed_per_config:
                launch_params["random_seed"] = base_seed + (config_index - 1) * seed_stride_per_config
            else:
                launch_params["random_seed"] = base_seed
            launch_params["topo_prm_seed"] = topo_prm_seed

            label = ", ".join(f"{key}={value}" for key, value in launch_params.items())
            print(f"\n[sweep] Config {config_index}/{len(configs)}: {label}")

            rows, _timed_out = launch_config(launch_params, n_trials, timeout_s, pkg_csv)
            config_trials = len(rows)
            config_successes = sum(
                int(row.get("success_feasible", row.get("trial_success", 0)))
                for row in rows
            )

            if fieldnames is None:
                if rows:
                    fieldnames = list(rows[0].keys())
                else:
                    fieldnames = list(FAILURE_COLUMNS)
                master_writer = csv.DictWriter(
                    master_file, fieldnames=fieldnames, extrasaction="ignore"
                )
                master_writer.writeheader()
                for buffered_row in buffered_rows:
                    master_writer.writerow(buffered_row)
                buffered_rows.clear()

            write_rows(master_writer, buffered_rows, rows, master_file)

            total_success += config_successes
            total_trials += config_trials
            rate = 100.0 * total_success / total_trials if total_trials else 0.0
            print(
                f"  [sweep] Config result: {config_successes}/{config_trials} feasible  "
                f"(running total: {total_success}/{total_trials} = {rate:.1f}%)"
            )

    print(f"\n[sweep] DONE. {total_success}/{total_trials} feasible trials.")
    print(f"[sweep] Results saved to: {master_path}")


if __name__ == "__main__":
    main()
