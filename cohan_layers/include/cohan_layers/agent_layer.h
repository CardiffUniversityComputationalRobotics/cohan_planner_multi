/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
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
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 * ROS 2 / nav2 port.
 *********************************************************************/

#ifndef AGENT_LAYER_H
#define AGENT_LAYER_H

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/state_array.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace cohan_layers
{
class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer() {}

  void onInitialize() override;
  void updateBounds(double origin_x, double origin_y, double origin_yaw, double * min_x, double * min_y,
                    double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override = 0;

  virtual void updateBoundsFromAgents(double * min_x, double * min_y, double * max_x, double * max_y) = 0;

  // nav2 requires these; the layer holds no persistent grid of its own, it is
  // rewritten from the agent topics every cycle.
  void reset() override
  {
    first_time_ = true;
  }

  bool isClearable() override
  {
    return false;
  }

  bool isDiscretized()
  {
    return false;
  }

protected:
  struct AgentPoseVel
  {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
    // transformed_agents_ is a filtered subset of agents_.agents, so its index
    // is NOT an index into agents_ or states_. Carry identity explicitly rather
    // than indexing those by position (which upstream did, and which also got
    // shadowed by the cell loop counters in updateCosts).
    uint64_t track_id = 0;
    int type = 0;
    int state = -1;
  };

  void agentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr agents);

  void statesCB(const cohan_msgs::msg::StateArray::SharedPtr states);

  void shutdownCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  //! State for the agent at position `idx` in agents_.agents, or -1 if unknown.
  //! TrackedAgents and StateArray are published together and built in the same
  //! order, so position is the correct key. track_id is NOT: the ids come from
  //! the tracker (pedsim ids are 0-based and the robot consumes one), so
  //! states_[track_id - 1] reads a different person, or underflows for id 0.
  int stateForAgentIndex(std::size_t idx) const
  {
    if (idx >= states_.states.size())
      return -1;
    return static_cast<int>(states_.states[idx]);
  }

  double Guassian1D(double x, double x0, double A, double varx)
  {
    double dx = x - x0;
    return A * exp(-pow(dx, 2.0) / (2.0 * varx));
  }

  double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d * cos(theta), Y = d * sin(theta);
    return A / std::max(d, 1.0) * Guassian1D(X, 0.0, 1.0, varx) * Guassian1D(Y, 0.0, 1.0, vary);
  }

  double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d * cos(theta - skew_ang), Y = d * sin(theta - skew_ang);
    return A / std::max(d, 1.0) * Guassian1D(X, 0.0, 1.0, varx) * Guassian1D(Y, 0.0, 1.0, vary);
  }

  double getRadius(double cutoff, double A, double var)
  {
    return sqrt(-2 * var * log(cutoff / A));
  }

  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr agents_sub_;
  rclcpp::Subscription<cohan_msgs::msg::StateArray>::SharedPtr agents_states_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopmap_srv_;
  // clock_ and logger_ are inherited from nav2_costmap_2d::Layer; declaring
  // them again here would shadow the ones the costmap already set up.

  cohan_msgs::msg::TrackedAgents agents_;
  cohan_msgs::msg::StateArray states_;
  std::vector<AgentPoseVel> transformed_agents_;
  std::recursive_mutex lock_;
  bool first_time_ = true, reset_ = true, shutdown_ = false;
  rclcpp::Time last_time_;
  double last_min_x_ = 0.0, last_min_y_ = 0.0, last_max_x_ = 0.0, last_max_y_ = 0.0;
  double radius_ = 1.5, amplitude_ = 150.0, covar_ = 0.25, cutoff_ = 10.0;
  double lethal_radius_ = 0.3;
  double robot_radius_ = 0.46, agent_radius_ = 0.31;
  std::string tracked_agents_topic_, agents_states_topic_;
};
}  // namespace cohan_layers

#endif  // AGENT_LAYER_H
