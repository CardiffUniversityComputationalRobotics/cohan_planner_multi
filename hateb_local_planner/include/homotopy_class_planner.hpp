/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 * Minor Modifications by: Phani Teja Singamaneni
 *********************************************************************/

#include <homotopy_class_planner.h>
#include <h_signature.h>

namespace hateb_local_planner
{

  template <typename BidirIter, typename Fun>
  EquivalenceClassPtr HomotopyClassPlanner::calculateEquivalenceClass(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer *obstacles,
                                                                      boost::optional<TimeDiffSequence::iterator> timediff_start, boost::optional<TimeDiffSequence::iterator> timediff_end)
  {
    if (params().include_dynamic_obstacles)
    {
      HSignature3d *H = new HSignature3d();
      H->calculateHSignature(path_start, path_end, fun_cplx_point, obstacles, timediff_start, timediff_end);
      return EquivalenceClassPtr(H);
    }
    else
    {
      HSignature *H = new HSignature();
      H->calculateHSignature(path_start, path_end, fun_cplx_point, obstacles);
      return EquivalenceClassPtr(H);
    }
  }

  template <typename BidirIter, typename Fun>
  TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position, double start_orientation, double goal_orientation, const geometry_msgs::msg::Twist *start_velocity, double dt_ref)
  {
    // Pass the full context, as the other two addAndInitNewTeb overloads do.
    // Constructing with only (obstacles, robot_model) leaves via_points_,
    // agents_via_points_map_ and agent_model_ at their defaults, so graph-search
    // candidates would silently ignore via-points and the agent constraints.
    TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr(new TebOptimalPlanner(obstacles_, robot_model_, visualization_, via_points_, agent_model_, agents_via_points_map_));

    // candidate->teb().initTrajectoryToGoal(path_start, path_end, fun_position, params().max_vel_x, params().max_vel_theta,
    // params().acc_lim_x, params().acc_lim_theta, start_orientation, goal_orientation, params().min_samples,
    // params().allow_init_with_backwards_motion);
    candidate->teb().initTEBtoGoal(path_start, path_end, fun_position, params().max_vel_x, params().max_vel_theta,
                                   params().acc_lim_x, params().acc_lim_theta, start_orientation, goal_orientation, params().min_samples);

    if (start_velocity)
      candidate->setVelocityStart(*start_velocity);

    EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                      candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

    if (addEquivalenceClassIfNew(H))
    {
      tebs_.push_back(candidate);
      return tebs_.back();
    }

    // If the candidate constitutes no new equivalence class, return a null pointer
    return TebOptimalPlannerPtr();
  }

} // namespace hateb_local_planner
