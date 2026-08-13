/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved. (BSD-3, see package LICENSE)
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 * ROS 2 / nav2 port.
 *********************************************************************/

#include <cohan_layers/agent_layer.h>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

#define DEFAULT_AGENT_PART cohan_msgs::msg::TrackedSegmentType::TORSO

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cohan_layers
{
void AgentLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error{"cohan_layers::AgentLayer: failed to lock node"};

  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_time_ = clock_->now();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tracked_agents_topic", rclcpp::ParameterValue(std::string("/tracked_agents")));
  declareParameter("agents_states_topic", rclcpp::ParameterValue(std::string("/agents_states")));
  declareParameter("robot_radius", rclcpp::ParameterValue(0.46));
  declareParameter("agent_radius", rclcpp::ParameterValue(0.31));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "tracked_agents_topic", tracked_agents_topic_);
  node->get_parameter(name_ + "." + "agents_states_topic", agents_states_topic_);
  node->get_parameter(name_ + "." + "robot_radius", robot_radius_);
  node->get_parameter(name_ + "." + "agent_radius", agent_radius_);

  // The subscriptions MUST go into the costmap's callback group. Costmap2DROS
  // spins only that group on its own thread; the node's default callback group
  // is spun by nobody here (main() spins the planner node, not the costmap).
  // Without this the subscriptions exist, the topic publishes, and the
  // callbacks are simply never invoked.
  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = callback_group_;

  agents_sub_ = node->create_subscription<cohan_msgs::msg::TrackedAgents>(
      tracked_agents_topic_, rclcpp::QoS(10),
      std::bind(&AgentLayer::agentsCB, this, std::placeholders::_1), sub_opt);

  agents_states_sub_ = node->create_subscription<cohan_msgs::msg::StateArray>(
      agents_states_topic_, rclcpp::QoS(10),
      std::bind(&AgentLayer::statesCB, this, std::placeholders::_1), sub_opt);

  stopmap_srv_ = node->create_service<std_srvs::srv::SetBool>(
      name_ + "/shutdown_layer",
      std::bind(&AgentLayer::shutdownCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

  current_ = true;
  first_time_ = true;
  shutdown_ = false;

  RCLCPP_INFO(logger_, "cohan_layers::%s subscribing to %s and %s", name_.c_str(),
              tracked_agents_topic_.c_str(), agents_states_topic_.c_str());
}

void AgentLayer::agentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr agents)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  agents_ = *agents;
}

void AgentLayer::statesCB(const cohan_msgs::msg::StateArray::SharedPtr states)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  states_ = *states;
  reset_ = false;
  last_time_ = clock_->now();
}

void AgentLayer::shutdownCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  shutdown_ = req->data;
  res->success = true;
  res->message = shutdown_ ? "Shutting down the agent layer costmaps.." : "Agent layer is switched on !";
}

void AgentLayer::updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/, double * min_x,
                              double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_agents_.clear();

  if ((clock_->now() - last_time_).seconds() > 1.0)
  {
    reset_ = true;
    states_.states.clear();
  }

  for (std::size_t idx = 0; idx < agents_.agents.size(); ++idx)
  {
    auto & agent = agents_.agents[idx];
    for (auto & segment : agent.segments)
    {
      if (segment.type != DEFAULT_AGENT_PART || shutdown_)
        continue;

      // Mark every tracked agent. Upstream skipped anyone whose state was 0,
      // but in this integration NO_STATE(0) is precisely what a person standing
      // still gets - so that filter dropped exactly the people the costmap most
      // needs to show. The state is carried through instead and only decides
      // whether the derived layer paints a soft Gaussian or a lethal disc.
      AgentPoseVel agent_pose_vel;
      agent_pose_vel.header.frame_id = agents_.header.frame_id;
      agent_pose_vel.header.stamp = agents_.header.stamp;
      agent_pose_vel.track_id = agent.track_id;
      agent_pose_vel.type = static_cast<int>(agent.type);
      agent_pose_vel.state = stateForAgentIndex(idx);
      geometry_msgs::msg::PoseStamped before_pose, after_pose;

      try
      {
        before_pose.pose = segment.pose.pose;
        before_pose.header.frame_id = agents_.header.frame_id;
        before_pose.header.stamp = agents_.header.stamp;
        tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
        agent_pose_vel.pose = after_pose.pose;

        before_pose.pose.position.x += segment.twist.twist.linear.x;
        before_pose.pose.position.y += segment.twist.twist.linear.y;
        auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
        tf2::Quaternion quat;
        quat.setEuler(segment.twist.twist.angular.z + hb_yaw, 0.0, 0.0);
        tf2::convert(before_pose.pose.orientation, quat);
        tf_->transform(before_pose, after_pose, global_frame, tf2::durationFromSec(0.5));
        agent_pose_vel.velocity.linear.x = after_pose.pose.position.x - agent_pose_vel.pose.position.x;
        agent_pose_vel.velocity.linear.y = after_pose.pose.position.y - agent_pose_vel.pose.position.y;
        agent_pose_vel.velocity.angular.z = angles::shortest_angular_distance(
            tf2::getYaw(after_pose.pose.orientation), tf2::getYaw(agent_pose_vel.pose.orientation));

        transformed_agents_.push_back(agent_pose_vel);
      }
      catch (const tf2::TransformException & ex)
      {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000, "cohan_layers: transform error: %s", ex.what());
        continue;
      }
    }
  }

  // Answers "is this layer doing anything" without a debugger: how many agents
  // arrived, how many survived the tf transform, and into which frame.
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "cohan_layers::%s: %zu tracked agents -> %zu marked (frame %s)",
                       name_.c_str(), agents_.agents.size(), transformed_agents_.size(), global_frame.c_str());

  updateBoundsFromAgents(min_x, min_y, max_x, max_y);

  if (first_time_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else
  {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}
}  // namespace cohan_layers
