#!/usr/bin/env python3
"""
Parameter sweep driver for the formation planner.

Usage:
    python3 sweep.py sweep_config.yaml

Requires: roscore already running in a separate terminal.
Each trial launches topologic_pp via rosrun, waits for it to exit
(it exits itself after n_trials runs), then aggregates results.
"""

import subprocess
import csv
import time
import os
import signal
import sys
import yaml
import itertools
import shutil
from pathlib import Path
from datetime import datetime


# ── helpers ────────────────────────────────────────────────────────────────

def set_rosparam(name: str, value):
    subprocess.run(
        ["rosparam", "set", f"/{name}", str(value)],
        check=True, capture_output=True
    )


def run_one_config(params: dict, n_trials: int, timeout_s: int,
                   pkg_csv: Path, master_csv_path: Path,
                   master_writer, success_counts: list) -> int:
    """
    Set rosparams, launch topologic_pp, wait for it to finish or timeout.
    Returns number of successful trials written to master CSV.
    """
    # 1. push all params
    set_rosparam("n_trials",         n_trials)
    for k, v in params.items():
        set_rosparam(k, v)

    # 2. delete the per-run CSV so we can detect new rows cleanly
    if pkg_csv.exists():
        pkg_csv.unlink()

    # 3. launch node
    proc = subprocess.Popen(
        ["rosrun", "formation_planner", "topologic_pp"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )

    # 4. stream stdout so the running success rate prints live
    deadline = time.time() + timeout_s
    timed_out = False
    while proc.poll() is None:
        line = proc.stdout.readline()
        if line:
            print("  |", line.rstrip())
        if time.time() > deadline:
            print(f"  [sweep] TIMEOUT after {timeout_s}s — killing")
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            timed_out = True
            break
        time.sleep(0.05)

    # drain remaining stdout
    for line in proc.stdout:
        print("  |", line.rstrip())

    # 5. read new rows from pkg_csv and append to master
    new_successes = 0
    if pkg_csv.exists():
        with open(pkg_csv) as f:
            reader = csv.DictReader(f)
            for row in reader:
                # inject sweep params that aren't in the C++ CSV
                row["timed_out"] = int(timed_out)
                master_writer.writerow(row)
                if int(row.get("success", 0)):
                    new_successes += 1
        master_csv_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        # node timed out before writing anything — write n_trials failure rows
        print("  [sweep] No CSV output — recording as failures")
        for i in range(n_trials):
            row = {k: v for k, v in params.items()}
            row.update({
                "num_robot": params.get("num_robot", "?"),
                "seed": params.get("random_seed", 42),
                "trial": i,
                "success": 0,
                "time_topo_s": -1, "time_filter_s": -1,
                "time_coarse_s": -1, "time_to_s": -1, "time_total_s": -1,
                "outer_iters": -1, "inner_iters": -1,
                "tf_s": -1, "distance_m": -1,
                "timed_out": 1,
            })
            master_writer.writerow(row)

    return new_successes


# ── main ───────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: sweep.py sweep_config.yaml")
        sys.exit(1)

    config_path = Path(sys.argv[1])
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    n_trials    = cfg.get("n_trials_per_config", 5)
    timeout_s   = cfg.get("timeout_per_trial_s", 60) * n_trials + 30
    pkg_result  = Path(cfg["pkg_result_dir"])          # e.g. .../traj_result
    pkg_csv     = pkg_result / "sweep_results.csv"     # written by C++ node

    timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
    master_path = Path(cfg.get("output_dir", ".")) / f"sweep_{timestamp}.csv"
    master_path.parent.mkdir(parents=True, exist_ok=True)

    # build parameter grid
    grid = cfg["param_grid"]                           # dict of name → list
    keys = list(grid.keys())
    values = list(grid.values())
    configs = [dict(zip(keys, combo)) for combo in itertools.product(*values)]

    print(f"[sweep] {len(configs)} configurations × {n_trials} trials = "
          f"{len(configs) * n_trials} total trials")
    print(f"[sweep] Output → {master_path}")
    print(f"[sweep] Per-trial timeout: {timeout_s}s total per config\n")

    total_success = 0
    total_trials  = 0

    with open(master_path, "w", newline="") as master_f:
        # We don't know the fieldnames until the first C++ row arrives;
        # use a list to buffer and write header after first config.
        buffered_rows = []
        fieldnames = None
        master_writer = None

        for ci, params in enumerate(configs):
            label = ", ".join(f"{k}={v}" for k, v in params.items())
            print(f"\n[sweep] Config {ci+1}/{len(configs)}: {label}")

            # delete stale csv
            if pkg_csv.exists():
                pkg_csv.unlink()

            # push params
            set_rosparam("n_trials", n_trials)
            for k, v in params.items():
                set_rosparam(k, v)

            # launch
            proc = subprocess.Popen(
                ["rosrun", "formation_planner", "topologic_pp"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid, text=True,
            )

            deadline = time.time() + timeout_s
            timed_out = False
            while proc.poll() is None:
                line = proc.stdout.readline()
                if line:
                    print("  |", line.rstrip())
                if time.time() > deadline:
                    print(f"  [sweep] TIMEOUT — killing")
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                    timed_out = True
                    break
                time.sleep(0.05)
            for line in proc.stdout:
                print("  |", line.rstrip())

            # read results
            config_successes = 0
            config_trials    = 0
            new_rows = []
            if pkg_csv.exists():
                with open(pkg_csv) as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        row["timed_out"] = int(timed_out)
                        new_rows.append(row)
                        config_trials += 1
                        if int(row.get("trial_success", row.get("success", 0))):
                            config_successes += 1

                if fieldnames is None and new_rows:
                    fieldnames = list(new_rows[0].keys())
                    master_writer = csv.DictWriter(master_f, fieldnames=fieldnames,
                                                   extrasaction="ignore")
                    master_writer.writeheader()
                    for r in buffered_rows:
                        master_writer.writerow(r)

                for r in new_rows:
                    if master_writer:
                        master_writer.writerow(r)
                    else:
                        buffered_rows.append(r)
                master_f.flush()
            else:
                print("  [sweep] No CSV output from node — all failures")
                for i in range(n_trials):
                    row = dict(params)
                    row.update({
                        "trial": i, "trial_success": 0, "timed_out": 1,
                        "time_topo_s": -1, "time_filter_s": -1,
                        "time_coarse_s": -1, "time_to_s": -1, "time_total_s": -1,
                        "outer_iters": -1, "inner_iters": -1, "tf_s": -1, "distance_m": -1,
                    })
                    if master_writer:
                        master_writer.writerow(row)
                    else:
                        buffered_rows.append(row)
                config_trials = n_trials

            total_success += config_successes
            total_trials  += config_trials
            rate = 100.0 * total_success / total_trials if total_trials else 0
            print(f"  [sweep] Config result: {config_successes}/{config_trials} succeeded  "
                  f"(running total: {total_success}/{total_trials} = {rate:.1f}%)")

    print(f"\n[sweep] DONE. {total_success}/{total_trials} trials succeeded.")
    print(f"[sweep] Results saved to: {master_path}")


if __name__ == "__main__":
    main()
