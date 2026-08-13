/*********************************************************************
 * Single fixed configuration for this differential-drive ROS 2 port.
 *
 * The upstream ROS 1 planner carried a ~600 line HATebConfig class fed by
 * dynamic_reconfigure. This port does not need runtime retuning, so the values
 * live here instead - but they must live in exactly ONE place. Previously every
 * g2o edge header kept its own private copy of the same constant, and the
 * copies had already drifted apart (inflation_dist was 0.6 in optimal_planner.h
 * and 0.1 in edge_obstacle.h; changing max_vel_x needed edits in two files).
 *
 * Everything below is a plain default. The handful of values that the node
 * genuinely reads from YAML (velocities, radii) are overwritten once at startup
 * in HATEBPlanningFramework's constructor, before any planner or edge is built.
 *
 * ponytail: mutable process-wide singleton, not a per-instance config. That is
 * exactly right while one process runs one planner for one robot. If this ever
 * hosts several robots in one process, pass a HATebParams& into the planner and
 * the edges instead of calling params().
 *********************************************************************/

#ifndef HATEB_PARAMS_H_
#define HATEB_PARAMS_H_

#include <cmath>

namespace hateb_local_planner
{

  struct HATebParams
  {
    // ================= Robot (differential drive) =================
    // max_vel_y / acc_lim_y are zero by design: no strafing. Several code
    // paths key off max_vel_y == 0 to pick the nonholonomic branch
    // (AddEdgesVelocity, extractVelocity), so leave them at zero.
    double max_vel_x = 0.5;
    double max_vel_y = 0.0;
    double max_vel_theta = 1.2;
    // Upstream default. Forward-only (0.0) is NOT supported here: pose 0 of the
    // band is pinned to the robot's current position AND orientation, so when
    // the path leaves from behind the robot the optimizer's only options are to
    // reverse or turn in place. Forbidding reverse leaves it oscillating in yaw
    // instead of committing, because single-band TEB has no
    // deletePlansDetouringBackwards() filter to reject a backward start (that
    // lived in the homotopy planner, which is gone).
    double max_vel_x_backwards = 0.2;
    double acc_lim_x = 0.5;
    double acc_lim_y = 0.0;
    double acc_lim_theta = 0.5;
    double min_turning_radius = 0.0; // diff drive turns in place
    bool exact_arc_length = false;

    // ================= Trajectory =================
    bool teb_autosize = true;
    double dt_ref = 0.3;
    double dt_hysteresis = 0.1;
    int min_samples = 3;
    int agent_min_samples = 3;
    double teb_init_skip_dist = 0.4;
    double force_reinit_new_goal_dist = 1.0;
    double force_reinit_new_goal_angular = 0.5 * M_PI;
    bool disable_warm_start = false;
    bool via_points_ordered = false;
    double min_resolution_collision_check_angular = M_PI;
    int feasibility_check_no_poses = 5;
    int control_look_ahead_poses = 1;
    // <= 0 disables the explicit cap, leaving the horizon bounded by the local
    // costmap size (~3.6 m for the 4 m benchmark costmap).
    double max_global_plan_lookahead_dist = 0.0;
    double global_plan_prune_distance = 1.0;
    // Via-points are the main thing holding the band inside a corridor, and they
    // are only subsampled from the global plan, never interpolated. Too sparse
    // and a narrow passage gets one lonely point with a long unguided gap.
    // The esc planner emits a 0.2 m path, so keep this close to that.
    double global_plan_viapoint_sep = 0.3;
    int planning_mode = 1;

    // ================= Optimization =================
    bool optimization_activate = true;
    bool optimization_verbose = false;
    bool publish_feedback = false;
    int no_inner_iterations = 8;
    int no_outer_iterations = 4;
    double penalty_epsilon = 0.1;
    double weight_adapt_factor = 2.0;

    double weight_max_vel_x = 2.0;
    double weight_max_vel_y = 2.0;
    double weight_max_vel_theta = 1.0;
    double weight_acc_lim_x = 1.0;
    double weight_acc_lim_y = 1.0;
    double weight_acc_lim_theta = 1.0;

    // weight_kinematics_nh enforces the nonholonomic constraint: it is what
    // stops the optimizer producing sideways motion between poses. It has to
    // dominate the soft costs, hence 1000 rather than an O(1) value.
    double weight_kinematics_nh = 1000.0;
    // Upstream default. Prefers forward motion without forbidding a reversal.
    // Raising this to force forward-only made the optimizer fight itself when
    // the goal was behind the robot; see max_vel_x_backwards above.
    double weight_kinematics_forward_drive = 1.0;
    double weight_kinematics_turning_radius = 1.0;

    double weight_optimaltime = 1.0;
    double weight_shortest_path = 0.0;
    double weight_obstacle = 50.0;
    double weight_inflation = 0.1;
    double weight_dynamic_obstacle = 50.0;
    double weight_dynamic_obstacle_inflation = 0.1;
    double weight_viapoint = 1.0;
    double weight_prefer_rotdir = 50.0;
    double weight_invisible_human = 1.0;

    // ================= Obstacles =================
    // Clearance demanded BEYOND the footprint radius. This is the parameter that
    // decides whether a narrow passage is solvable at all: the optimizer needs
    // (robot_radius + min_obstacle_dist) of half-width on BOTH sides, so a 0.3 m
    // robot with 0.3 m here cannot thread anything narrower than ~1.2 m and will
    // sit and oscillate instead. Overridable from YAML per environment.
    double min_obstacle_dist = 0.15;
    // Must stay > min_obstacle_dist for EdgeInflatedObstacle to be selected; the
    // gap between them is the soft push-away zone. Keep the gap small in tight
    // spaces or both walls push the band at once and it never settles.
    double inflation_dist = 0.25;
    double dynamic_obstacle_inflation_dist = 0.6;
    bool include_dynamic_obstacles = true;
    int obstacle_poses_affected = 25;
    bool legacy_obstacle_association = false;
    double obstacle_association_force_inclusion_factor = 1.5;
    double obstacle_association_cutoff_factor = 5.0;
    double costmap_obstacles_behind_robot_dist = 1.5;
    // Must stay 1.0 unless penalty_epsilon < min_obstacle_dist. The cost is
    // min_obstacle_dist * (err/min_obstacle_dist)^exponent, which only
    // attenuates while that ratio is <= 1; above it the term explodes.
    double obstacle_cost_exponent = 1.0;

    // ================= Agents (humans) =================
    double agent_radius = 0.3;
    double agent_max_vel_x = 1.3;
    double agent_nominal_vel_x = 1.1;
    double agent_max_vel_y = 0.4;
    double agent_max_vel_x_backwards = 0.0;
    double agent_max_vel_theta = 1.1;
    double agent_acc_lim_x = 0.6;
    // Upstream declared agent.acc_lim_y but never gave it a default, so the
    // holonomic agent edges read uninitialised memory. Mirror acc_lim_x.
    double agent_acc_lim_y = 0.6;
    double agent_acc_lim_theta = 0.8;
    int num_moving_avg = 5;
    bool use_agent_elastic_vel = true;
    double fov = 90.0; // degrees

    double weight_max_agent_vel_x = 2.0;
    double weight_max_agent_vel_y = 2.0;
    double weight_max_agent_vel_theta = 2.0;
    double weight_nominal_agent_vel_x = 2.0;
    double weight_agent_acc_lim_x = 1.0;
    double weight_agent_acc_lim_y = 1.0;
    double weight_agent_acc_lim_theta = 1.0;
    double weight_agent_optimaltime = 1.0;
    double weight_agent_viapoint = 1.0;

    // ================= Human-aware constraints =================
    double min_agent_robot_dist = 0.6;
    double min_agent_agent_dist = 0.2;
    bool use_agent_robot_safety_c = true;
    bool use_agent_agent_safety_c = true;
    bool use_agent_robot_ttc_c = true;
    bool use_agent_robot_ttcplus_c = false;
    bool use_agent_robot_rel_vel_c = true;
    bool use_agent_robot_visi_c = true;
    bool scale_agent_robot_ttc_c = true;
    bool scale_agent_robot_ttcplus_c = true;
    double ttc_threshold = 5.0;
    double ttcplus_threshold = 5.0;
    double ttcplus_timer = 5.0;
    double agent_robot_ttc_scale_alpha = 1.0;
    double agent_robot_ttcplus_scale_alpha = 1.0;
    double rel_vel_cost_threshold = 5.0;
    double visibility_cost_threshold = 5.0;
    double invisible_human_threshold = 5.0;
    // The invisible-humans feature needs the invisible_humans_detection package,
    // which was not ported. Keep off until it is.
    bool add_invisible_humans = false;

    double weight_agent_robot_safety = 20.0;
    double weight_agent_agent_safety = 20.0;
    double weight_agent_robot_ttc = 20.0;
    double weight_agent_robot_ttcplus = 20.0;
    double weight_agent_robot_rel_vel = 20.0;
    double weight_agent_robot_visibility = 20.0;

    // ================= Recovery =================
    double omega_chage_time_seperation = 1.0;
    bool shrink_horizon_backup = true;
    double shrink_horizon_min_duration = 10.0;
    bool oscillation_recovery = true;
    double oscillation_v_eps = 0.1;
    double oscillation_omega_eps = 0.1;
    double oscillation_recovery_min_duration = 10.0;
    double oscillation_filter_duration = 10.0;

  };

  //! Process-wide configuration. Single definition across translation units
  //! via the function-local static (C++14 has no inline variables).
  inline HATebParams &params()
  {
    static HATebParams p;
    return p;
  }

} // namespace hateb_local_planner

#endif /* HATEB_PARAMS_H_ */
