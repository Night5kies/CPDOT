# Reproducing Final Results

This page covers how to rebuild the planner, run the ablation studies, and locate the final CSV outputs.

All commands below assume the workspace root is:

```bash
~/CPDOT2
```

## 1. Environment

The current codebase was run in a ROS Noetic workspace on Ubuntu 20.04 with MOSEK available for the formation optimizer.

At minimum you will need:

- ROS Noetic
- a built catkin workspace
- a working MOSEK installation and license

## 2. Build the Workspace

If the workspace has not been built yet, run:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
catkin_make --cmake-args -DMOSEK_DIR=$HOME/tools/mosek/11.1
source devel/setup.bash
```

If the workspace is already configured and you only want to rebuild the planner package:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build formation_planner
```

## 3. Important Output Paths

The sweep driver writes to two places:

- temporary per-config file:
  `~/CPDOT2/src/CPDOT2/formation_planner/traj_result/sweep_results.csv`
- final timestamped output:
  `~/CPDOT2/src/CPDOT2/formation_planner/traj_result/sweep_outputs/sweep_<timestamp>.csv`

Keep the timestamped file for analysis. The temporary `sweep_results.csv` is reused across runs.

If you want a clean run, remove the temporary file before starting:

```bash
rm -f ~/CPDOT2/src/CPDOT2/formation_planner/traj_result/sweep_results.csv
```

## 4. Start ROS

The sweep script expects `roscore` to already be running.

Terminal 1:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

## 5. Reproduce Extension A: Feasibility Weights

This sweep reproduces Extension A, the feasibility-weight study over:

- `opti_w_topo`
- `opti_w_sfc`
- `opti_w_formation`

- [sweep_config.yaml](formation_planner/scripts/sweep_config.yaml)

It runs:

- `13` configurations
- `30` trials per configuration
- `390` total trials

Terminal 2:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/CPDOT2/formation_planner/scripts/sweep.py \
  src/CPDOT2/formation_planner/scripts/sweep_config.yaml
```

- [sweep_20260426_213231.csv](formation_planner/traj_result/sweep_outputs/sweep_20260426_213231.csv)

## 6. Reproduce Extensions B and C

This sweep evaluates the four extension settings:

1. no extensions
2. Extension C only
3. Extension B only
4. Extension B + C

- [sweep_extensions_ablation.yaml](formation_planner/scripts/sweep_extensions_ablation.yaml)

It runs:

- `4` configurations
- `100` trials per configuration
- `400` total trials

Terminal 2:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/CPDOT2/formation_planner/scripts/sweep.py \
  src/CPDOT2/formation_planner/scripts/sweep_extensions_ablation.yaml
```

The extension toggles are controlled through environment variables inside the sweep:

- `CPDOT_ANALYTICAL_LIFT=1` enables Extension C
- `CPDOT_FAILURE_PRUNING=1` enables Extension B

The current four-way ordering in the config is:

1. `failure_pruning=0`, `analytical_lift=0`
2. `failure_pruning=0`, `analytical_lift=1`
3. `failure_pruning=1`, `analytical_lift=0`
4. `failure_pruning=1`, `analytical_lift=1`

## 7. Run Only the B and B+C Cases

If the first two extension cases have already been run and you only want the remaining Extension B cases, use:

- [sweep_extensions_ablation_from_B.yaml](formation_planner/scripts/sweep_extensions_ablation_from_B.yaml)

This runs:

1. Extension B only
2. Extension B + C

Command:

```bash
cd ~/CPDOT2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/CPDOT2/formation_planner/scripts/sweep.py \
  src/CPDOT2/formation_planner/scripts/sweep_extensions_ablation_from_B.yaml
```

## 8. Existing Final CSVs

The repository already contains the final extension CSVs used for analysis:

- [sweep_ext_baseline.csv](formation_planner/traj_result/sweep_outputs/sweep_ext_baseline.csv)
- [sweep_ext_A.csv](formation_planner/traj_result/sweep_outputs/sweep_ext_A.csv)
- [sweep_ext_B.csv](formation_planner/traj_result/sweep_outputs/sweep_ext_B.csv)
- [sweep_ext_AB.csv](formation_planner/traj_result/sweep_outputs/sweep_ext_AB.csv)

These correspond to:

- baseline: no extensions
- `sweep_ext_A.csv`: legacy filename for Extension C only
- `sweep_ext_B.csv`: Extension B only
- `sweep_ext_AB.csv`: legacy filename for Extension B + C

## 9. Useful CSV Columns

The most important columns are:

- `trial_success`
- `success_feasible`
- `final_infeasibility`
- `time_total_s`
- `time_to_s`
- `outer_iters`
- `inner_iters`

For the extension sweep, the new diagnostic columns are:

- `analytical_lift_enabled`
- `failure_pruning_enabled`
- `outer_candidates_considered`
- `outer_candidates_pruned`
- `outer_ocp_attempts`
- `uncrossable_failure`

## 10. Minimal Aggregation Example

The following script summarizes success rate and average timing by extension setting:

```bash
python3 - <<'PY'
import csv
from collections import defaultdict
from pathlib import Path

path = Path.home() / "CPDOT2" / "src/CPDOT2/formation_planner/traj_result/sweep_outputs/sweep_ext_AB.csv"
groups = defaultdict(list)

with open(path) as f:
    for row in csv.DictReader(f):
        key = (
            row.get("failure_pruning_enabled", "0"),
            row.get("analytical_lift_enabled", "0"),
        )
        groups[key].append(row)

for key, rows in sorted(groups.items()):
    n = len(rows)
    feasible = sum(int(r["success_feasible"]) for r in rows)
    mean_total = sum(float(r["time_total_s"]) for r in rows) / n
    mean_inner = sum(float(r["inner_iters"]) for r in rows) / n
    print(key, n, feasible / n, mean_total, mean_inner)
PY
```

Replace `path` with any timestamped sweep output you want to analyze.

## 11. Notes

- `sweep.py` requires `roscore`; it will not start ROS for you.
- Each configuration launches `rosrun formation_planner topologic_pp`.
- The sweep script offsets the random seed by configuration by default.
- If a run is interrupted, keep the timestamped CSV if it was already written, but remove `traj_result/sweep_results.csv` before restarting a fresh sweep.
