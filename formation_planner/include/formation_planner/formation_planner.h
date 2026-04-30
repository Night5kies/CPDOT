/**
 * file formation_planner.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief head file for formation planning
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <memory>
#include "optimizer_interface.h"
#include "planner_config.h"
#include "environment.h"
#include "coarse_path_planner.h"
#include <vector>
#include <set>
#include "iris/iris.h"

#ifndef SRC_formation_planner_H
#define SRC_formation_planner_H

namespace formation_planner {

// Extension B: failure-aware combination pruning.
// Input fields are read by Plan_fm; output fields are written when Plan_fm
// returns false. The caller maintains blocked_obstacles across combinations
// and consults failure_reason / failed_obstacle_idx to decide what to add.
struct PruningContext {
  static constexpr int FAIL_NONE          = 0;
  static constexpr int FAIL_WARM_START    = 1;
  static constexpr int FAIL_INFEAS_LIMIT  = 2;
  static constexpr int FAIL_MAX_FORMATION = 3;
  static constexpr int FAIL_REFINEMENT    = 4;
  static constexpr int FAIL_PRUNED        = 5;

  bool enable = false;
  std::set<int> blocked_obstacles;
  int failed_obstacle_idx = -1;
  int failure_reason = FAIL_NONE;
};

class FormationPlanner {
public:
  explicit FormationPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

  void set_global_path(const std::vector<math::Pose> &path) {
    global_path_ = path;
  }

  bool Plan(const FullStates &prev_solution, const TrajectoryPoint &start, const TrajectoryPoint &goal, iris::IRISProblem &iris_problem, FullStates &result, double& coarse_time, double& solve_time, bool show_cr, const std::vector<std::vector<std::vector<double>>>& hyperparam_set);
  bool Plan_car_like(const FullStates &prev_solution, const double offset, FullStates &result, double& solve);
  bool Plan_diff_drive(const FullStates &guess, const FullStates &prev_solution, const TrajectoryPoint &start, const TrajectoryPoint &goal, FullStates &result, const int robot_index, double& max_error, double& solve_time);
  bool Plan_car_like_replan(const FullStates &guess, const FullStates &prev_solution, FullStates &result, const int robot_index, double& infeasible, double& solution_car_like);
  bool Plan_fm(
  const std::vector<FullStates> &prev_sol, 
  const std::vector<TrajectoryPoint> &start_set, 
  const std::vector<TrajectoryPoint> &goal_set, 
  iris::IRISProblem &iris_problem, 
  std::vector<FullStates> &result, 
  std::vector<double>& coarse_time_set, 
  std::vector<double>& solve_time_set, 
  bool show_cr, 
  const std::vector<math::Polygon2d>& polys_inflat,
  const std::vector<std::vector<std::vector<std::vector<double>>>>& hyperparam_sets,
  std::vector<ros::Publisher> path_pub_set,
  int& warm_start,
  double& cost,
  int& solve_success,
  double& e_max, double& e_avg,
  double& avg, double& std,
  double& final_infeasibility,
  double initial_guess_noise_stddev = 0.0,
  unsigned int initial_guess_noise_seed = 0u,
  PruningContext* pruning = nullptr);
  void GenerateHeightCons(const std::vector<FullStates>& guess, std::shared_ptr<Environment> env, std::vector<double>& height_cons);
  void GenerateHeightConsWithIdx(const std::vector<FullStates>& guess, std::shared_ptr<Environment> env,
                                 std::vector<double>& height_cons, std::vector<int>& obstacle_idx);

private:
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<IOptimizer> problem_;

  std::vector<math::Pose> global_path_;

  CoarsePathPlanner coarse_path_planner_;

  bool CheckGuessFeasibility(const FullStates &guess);

  FullStates StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start);

  FullStates GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start, const int step_num, bool ratio);

  bool CheckCarKinematic(const FullStates &current_result, const std::vector<double> offset_car);

  bool CheckDiffDriveKinematic(const FullStates &current_result, const std::vector<double> offset, const std::vector<double> rephi);

  FullStates ResamplePath(const std::vector<math::Pose> &path, const int step_num, bool ratio) const;

  std::vector<double> GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const;

  bool CheckHeightCons(const std::vector<formation_planner::FullStates> traj_set, const std::vector<double> height_cons, const std::vector<std::vector<double>> vertice_set, std::vector<double>& height);
  void DeriveHeight(const std::vector<formation_planner::FullStates> traj_set, const std::vector<std::vector<double>> vertice_set,
  std::vector<double> &height);
};

}

#endif //SRC_formation_planner_H
