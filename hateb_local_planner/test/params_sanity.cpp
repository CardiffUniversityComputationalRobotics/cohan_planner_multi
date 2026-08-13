// Guards the few configuration invariants that are silently catastrophic
// rather than loud. Each one of these was actually violated at some point
// during the ROS 1 -> ROS 2 migration.
//
// Header-only: no gtest, no ROS. Build and run it, exit code 0 means fine.

#include <cassert>
#include <cmath>
#include <cstdio>

#include <hateb_params.h>

int main()
{
  const auto &p = hateb_local_planner::params();

  // --- Differential drive ------------------------------------------------
  // AddEdgesVelocity() and extractVelocity() both branch on max_vel_y == 0 to
  // pick the nonholonomic path. Anything else silently turns the robot into a
  // holonomic one and the extracted (v, omega) stops matching the plan.
  assert(p.max_vel_y == 0.0);
  assert(p.acc_lim_y == 0.0);
  assert(p.min_turning_radius == 0.0);

  // The nonholonomic constraint has to dominate the soft costs. At O(1) the
  // optimizer happily plans sideways motion a diff-drive base cannot execute.
  assert(p.weight_kinematics_nh >= 100.0);

  // --- Obstacle cost shape -----------------------------------------------
  // EdgeObstacle computes  min_obstacle_dist * (err/min_obstacle_dist)^exponent
  // where err = penaltyBoundFromBelow(dist, min_obstacle_dist, penalty_epsilon)
  // peaks at (min_obstacle_dist + penalty_epsilon). The exponent only
  // attenuates while that ratio stays <= 1; above it the term explodes.
  assert(p.min_obstacle_dist > 0.0);
  const double worst_ratio = (p.min_obstacle_dist + p.penalty_epsilon) / p.min_obstacle_dist;
  assert(p.obstacle_cost_exponent == 1.0 || worst_ratio <= 1.0);

  // Whatever the exponent, the worst-case unweighted cost must stay sane.
  const double worst_cost = p.min_obstacle_dist * std::pow(worst_ratio, p.obstacle_cost_exponent);
  assert(std::isfinite(worst_cost) && worst_cost < 100.0);

  // --- Band resolution ----------------------------------------------------
  // initTEBtoGoal thins the global plan to teb_init_skip_dist spacing, so the
  // first segment is that long. autoResize() only subdivides a segment when its
  // time diff exceeds dt_ref + dt_hysteresis. If the estimated time for that
  // segment does not clear that bar, the band is never refined and keeps a
  // coarse spacing that implies far more than max_vel_x.
  const double first_segment_dt = p.teb_init_skip_dist / p.max_vel_x;
  assert(first_segment_dt > p.dt_ref + p.dt_hysteresis);

  // --- Agent (human) bands ------------------------------------------------
  // Zero weights leave the predicted human trajectories unconstrained, so the
  // robot plans around motion no person could produce.
  assert(p.weight_max_agent_vel_x > 0.0);
  assert(p.weight_nominal_agent_vel_x > 0.0);
  assert(p.weight_agent_acc_lim_x > 0.0);

  // --- Forward-only motion ------------------------------------------------
  // The optimizer's reverse penalties are soft, so forward-only only actually
  // holds if the hard clamp in saturateVelocity() is armed (max_vel_x_backwards
  // <= 0) AND the forward-drive term is heavy enough to shape the band.
  if (p.max_vel_x_backwards <= 0.0)
    assert(p.weight_kinematics_forward_drive >= 100.0);

  // --- Homotopy class planning -------------------------------------------
  // With fewer than 2 classes there is nothing to choose between and the
  // planner degenerates to the single band that stalls in narrow passages.
  if (p.enable_homotopy_class_planning)
    assert(p.max_number_classes >= 2);
  // HSignature asserts this at runtime; catch it here instead of mid-run.
  assert(p.h_signature_prescaler > 0.1 && p.h_signature_prescaler <= 1.0);

  std::printf("hateb params sanity: OK (worst obstacle cost %.4f, min passage %.2f m, hcp %s)\n",
              worst_cost, 2.0 * (0.3 + p.min_obstacle_dist),
              p.enable_homotopy_class_planning ? "on" : "off");
  return 0;
}
