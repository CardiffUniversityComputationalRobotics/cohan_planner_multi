/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Copyright (c) 2020 LAAS/CNRS
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
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#define GLOBAL_PLAN_TOPIC "global_plan"
#define LOCAL_PLAN_TOPIC "local_plan"
#define LOCAL_TRAJ_TOPIC "local_traj"
#define LOCAL_PLAN_POSES_TOPIC "local_plan_poses"
#define LOCAL_PLAN_FP_POSES_TOPIC "local_plan_fp_poses"
#define AGENT_GLOBAL_PLANS_TOPIC "agents_global_plans"
#define AGENT_LOCAL_PLANS_TOPIC "agents_local_plans"
#define AGENT_LOCAL_TRAJS_TOPIC "agents_local_trajs"
#define AGENT_LOCAL_PLANS_POSES_TOPIC "agents_local_plans_poses"
#define AGENT_LOCAL_PLANS_FP_POSES_TOPIC "agents_local_plans_fp_poses"
#define CLEARING_TIMER_DURATION 1.0 // seconds
#define ROBOT_FP_POSES_NS "robot_fp_poses"
#define AGENT_FP_POSES_NS "agents_fp_poses"
#define ROBOT_TRAJ_TIME_TOPIC "traj_time"
#define ROBOT_PATH_TIME_TOPIC "plan_time"
#define AGENT_TRAJS_TIME_TOPIC "agents_trajs_time"
#define AGENT_PATHS_TIME_TOPIC "agents_plans_time"
#define DEFAUTL_SEGMENT_TYPE cohan_msgs::msg::TrackedSegmentType::TORSO

#include <optimal_planner.h>
#include <visualization.h>
#include <cohan_msgs/msg/feedback_msg.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>

namespace hateb_local_planner
{

  // TebVisualization::TebVisualization() : initialized_(false)
  // {
  // }

  TebVisualization::TebVisualization(const rclcpp::Node::SharedPtr &node) : node_(node), tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())), initialized_(false)
  {
    initialize();
  }

  void TebVisualization::initialize()
  {
    if (initialized_)
      RCLCPP_WARN(node_->get_logger(), "TebVisualization already initialized. Reinitializing...");

    // // set config
    // cfg_ = &cfg;

    // register topics
    global_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(GLOBAL_PLAN_TOPIC, 1);
    local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(LOCAL_PLAN_TOPIC, 1);
    local_traj_pub_ = node_->create_publisher<cohan_msgs::msg::Trajectory>(LOCAL_TRAJ_TOPIC, 1);
    teb_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(LOCAL_PLAN_POSES_TOPIC, 1);
    teb_fp_poses_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(LOCAL_PLAN_FP_POSES_TOPIC, 1);
    agents_global_plans_pub_ = node_->create_publisher<cohan_msgs::msg::AgentPathArray>(AGENT_GLOBAL_PLANS_TOPIC, 1);
    agents_local_plans_pub_ = node_->create_publisher<cohan_msgs::msg::AgentPathArray>(AGENT_LOCAL_PLANS_TOPIC, 1);
    agents_local_trajs_pub_ = node_->create_publisher<cohan_msgs::msg::AgentTrajectoryArray>(AGENT_LOCAL_TRAJS_TOPIC, 1);
    agents_tebs_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(AGENT_LOCAL_PLANS_POSES_TOPIC, 1);
    agents_tebs_fp_poses_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(AGENT_LOCAL_PLANS_FP_POSES_TOPIC, 1);
    teb_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("teb_markers", 1000);
    feedback_pub_ = node_->create_publisher<cohan_msgs::msg::FeedbackMsg>("teb_feedback", 10);
    robot_traj_time_pub_ = node_->create_publisher<cohan_msgs::msg::AgentTimeToGoal>(ROBOT_TRAJ_TIME_TOPIC, 1);
    robot_path_time_pub_ = node_->create_publisher<cohan_msgs::msg::AgentTimeToGoal>(ROBOT_PATH_TIME_TOPIC, 1);
    agent_trajs_time_pub_ = node_->create_publisher<cohan_msgs::msg::AgentTimeToGoalArray>(AGENT_TRAJS_TIME_TOPIC, 1);
    agent_paths_time_pub_ = node_->create_publisher<cohan_msgs::msg::AgentTimeToGoalArray>(AGENT_PATHS_TIME_TOPIC, 1);
    agent_marker_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("agent_marker", 1);
    agent_arrow_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("agent_arrow", 1);
    mode_text_pub = node_->create_publisher<visualization_msgs::msg::Marker>("mode_text", 1);
    robot_next_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("robot_next_pose", 1);
    agent_next_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("agent_next_pose", 1);
    ttg_pub_ = node_->create_publisher<std_msgs::msg::Float32>("time_to_goal", 10);

    last_publish_robot_global_plan_ =
        publish_robot_global_plan_;
    last_publish_robot_local_plan_ = publish_robot_local_plan_;
    last_publish_robot_local_plan_poses_ =
        publish_robot_local_plan_poses_;
    last_publish_robot_local_plan_fp_poses_ =
        publish_robot_local_plan_fp_poses_;
    last_publish_agents_global_plans_ =
        publish_agents_global_plans_;
    last_publish_agents_local_plans_ =
        publish_agents_local_plans_;
    last_publish_agents_local_plan_poses_ =
        publish_agents_local_plan_poses_;
    last_publish_agents_local_plan_fp_poses_ =
        publish_agents_local_plan_fp_poses_;

    tracked_agents_sub_ = node_->create_subscription<cohan_msgs::msg::TrackedAgents>(
        "/tracked_agents", 10,
        std::bind(&TebVisualization::publishTrackedAgents, this, std::placeholders::_1));

    clearing_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(CLEARING_TIMER_DURATION),
        std::bind(&TebVisualization::clearingTimerCB, this));

    // node_->declare_parameter<std::string>("ns", "");
    // node_->get_parameter("ns", ns_);

    ns_ = "";

    last_robot_fp_poses_idx_ = 0;
    last_agent_fp_poses_idx_ = 0;
    initialized_ = true;
  }

  void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan) const
  {
    if (printErrorWhenNotInitialized() ||
        !publish_robot_global_plan_)
      return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node_->now(); // or rclcpp::Clock().now()
    path_msg.header.frame_id = map_frame_;
    path_msg.poses = global_plan;

    global_plan_pub_->publish(path_msg);
  }

  void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &local_plan) const
  {
    if (printErrorWhenNotInitialized())
      return;
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node_->now(); // or rclcpp::Clock().now()
    path_msg.header.frame_id = map_frame_;
    path_msg.poses = local_plan;

    local_plan_pub_->publish(path_msg);
  }

  void TebVisualization::publishAgentGlobalPlans(
      const std::vector<AgentPlanCombined> &agents_plans) const
  {
    if (printErrorWhenNotInitialized() ||
        !publish_agents_global_plans_ || agents_plans.empty())
    {
      return;
    }

    auto now = node_->now();
    auto frame_id = map_frame_;

    cohan_msgs::msg::AgentPathArray agent_path_array;
    agent_path_array.header.stamp = now;
    agent_path_array.header.frame_id = frame_id;

    for (auto &agent_plan_combined : agents_plans)
    {
      auto total_size = agent_plan_combined.plan_before.size() +
                        agent_plan_combined.plan_to_optimize.size() +
                        agent_plan_combined.plan_after.size();
      if (total_size == 0)
      {
        continue;
      }
      nav_msgs::msg::Path path;
      path.header.stamp = now;
      path.header.frame_id = frame_id;

      path.poses.resize(total_size);
      size_t index = 0;
      for (size_t i = 0; i < agent_plan_combined.plan_before.size(); ++i)
      {
        path.poses[i] = agent_plan_combined.plan_before[i];
      }
      index += agent_plan_combined.plan_before.size();
      for (size_t i = 0; i < agent_plan_combined.plan_to_optimize.size(); ++i)
      {
        path.poses[i + index] = agent_plan_combined.plan_to_optimize[i];
      }
      index += agent_plan_combined.plan_to_optimize.size();
      for (size_t i = 0; i < agent_plan_combined.plan_after.size(); ++i)
      {
        path.poses[i + index] = agent_plan_combined.plan_after[i];
      }

      cohan_msgs::msg::AgentPath agent_path;
      agent_path.header.stamp = now;
      agent_path.header.frame_id = frame_id;
      agent_path.id = agent_plan_combined.id;
      agent_path.path = path;

      agent_path_array.paths.push_back(agent_path);
    }
    if (!agent_path_array.paths.empty())
    {
      agents_global_plans_pub_->publish(agent_path_array);
    }
  }
  void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand &teb, const BaseRobotFootprintModel &robot_model, const double fp_size, const std_msgs::msg::ColorRGBA &color)
  {
    if (printErrorWhenNotInitialized() || (!publish_robot_local_plan_ && !publish_robot_local_plan_poses_ && !publish_robot_local_plan_fp_poses_))
      return;

    auto frame_id = map_frame_;
    auto now = node_->now();

    // create path msg
    nav_msgs::msg::Path teb_path;
    teb_path.header.frame_id = frame_id;
    teb_path.header.stamp = now;

    // create pose_array (along trajectory)
    geometry_msgs::msg::PoseArray teb_poses;
    teb_poses.header.frame_id = frame_id;
    teb_poses.header.stamp = now;

    // fill path msgs with teb configurations
    double pose_time = 0.0;
    // std::vector<double> pose_times;
    for (int i = 0; i < teb.sizePoses(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = now;
      pose.pose.position.x = teb.Pose(i).x();
      pose.pose.position.y = teb.Pose(i).y();
      // pose_times.push_back(teb.TimeDiff(i));
      pose.pose.position.z = 0; // visualize_with_time_as_z_axis_scale_*teb.getSumOfTimeDiffsUpToIdx(i);
      tf2::Quaternion q;
      q.setRPY(0, 0, teb.Pose(i).theta());
      pose.pose.orientation = tf2::toMsg(q);
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);

      if (i == 0)
      {
        robot_next_pose_pub_->publish(pose);
      }

      if (i < (teb.sizePoses() - 1))
      {
        pose_time += teb.TimeDiff(i);
      }
    }

    // publish robot local plans
    if (!teb_path.poses.empty() && publish_robot_local_plan_)
    {
      local_plan_pub_->publish(teb_path);
    }

    // publish robot local plan poses and footprint
    if (!teb_poses.poses.empty())
    {
      if (publish_robot_local_plan_poses_)
      {
        teb_poses_pub_->publish(teb_poses);
      }

      if (publish_robot_local_plan_fp_poses_)
      {
        visualization_msgs::msg::MarkerArray teb_fp_poses;
        int idx = 0;
        // double fp_size = teb_poses.poses.size();
        for (auto &pose : teb_poses.poses)
        {
          std::vector<visualization_msgs::msg::Marker> fp_markers;
          robot_model.visualizeRobot(pose, fp_markers, color);
          for (auto &marker : fp_markers)
          {
            marker.header.frame_id = map_frame_;
            marker.header.stamp = now;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.ns = ROBOT_FP_POSES_NS;
            marker.pose.position.z = vel_robot[idx] / 2;
            marker.scale.z = std::max(vel_robot[idx], 0.00001);
            marker.id = idx++;
            marker.color.a = 0.5;
            setMarkerColour(marker, (double)idx, fp_size);
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            teb_fp_poses.markers.push_back(marker);
          }
        }
        while (idx < last_robot_fp_poses_idx_)
        {
          visualization_msgs::msg::Marker clean_fp_marker;
          clean_fp_marker.header.frame_id = map_frame_;
          clean_fp_marker.header.stamp = now;
          clean_fp_marker.action = visualization_msgs::msg::Marker::DELETE;
          clean_fp_marker.id = idx++;
          clean_fp_marker.ns = ROBOT_FP_POSES_NS;
          teb_fp_poses.markers.push_back(clean_fp_marker);
        }
        last_robot_fp_poses_idx_ = idx;

        teb_fp_poses_pub_->publish(teb_fp_poses);
      }
    }
  }

  void TebVisualization::publishTrajectory(
      const PlanTrajCombined &plan_traj_combined)
  {
    if (printErrorWhenNotInitialized() ||
        !publish_robot_local_plan_)
    {
      return;
    }
    vel_robot.clear();

    auto frame_id = map_frame_;
    auto now = node_->now();

    cohan_msgs::msg::Trajectory trajectory;
    trajectory.header.frame_id = frame_id;
    trajectory.header.stamp = now;

    cohan_msgs::msg::AgentTimeToGoal robot_time_to_goal;
    robot_time_to_goal.header.frame_id = frame_id;
    robot_time_to_goal.header.stamp = now;

    cohan_msgs::msg::AgentTimeToGoal robot_time_to_goal_full;
    robot_time_to_goal_full.header.frame_id = frame_id;
    robot_time_to_goal_full.header.stamp = now;

    for (auto &pose : plan_traj_combined.plan_before)
    {
      cohan_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point.transform.translation.x = pose.pose.position.x;
      trajectory_point.transform.translation.y = pose.pose.position.y;
      trajectory_point.transform.translation.z = pose.pose.position.z;
      trajectory_point.transform.rotation = pose.pose.orientation;
      trajectory_point.time_from_start = rclcpp::Duration::from_seconds(-1.0);
      trajectory.points.push_back(trajectory_point);
    }

    for (auto &traj_point : plan_traj_combined.optimized_trajectory)
    {
      cohan_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point.transform.translation.x = traj_point.pose.position.x;
      trajectory_point.transform.translation.y = traj_point.pose.position.y;
      trajectory_point.transform.translation.z = traj_point.pose.position.z;
      trajectory_point.transform.rotation = traj_point.pose.orientation;
      trajectory_point.velocity = traj_point.velocity;
      auto r_vel = std::hypot(traj_point.velocity.linear.x, traj_point.velocity.linear.y);

      vel_robot.push_back(r_vel);
      trajectory_point.time_from_start = traj_point.time_from_start;
      trajectory.points.push_back(trajectory_point);
    }

    if (plan_traj_combined.optimized_trajectory.size() > 0)
    {
      robot_time_to_goal.time_to_goal = trajectory.points.back().time_from_start;
    }
    else
    {
      robot_time_to_goal.time_to_goal = rclcpp::Duration::from_seconds(0);
    }

    double remaining_path_dist = 0.0;
    const geometry_msgs::msg::PoseStamped *previous_pose = nullptr;
    for (auto &pose : plan_traj_combined.plan_after)
    {
      cohan_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point.transform.translation.x = pose.pose.position.x;
      trajectory_point.transform.translation.y = pose.pose.position.y;
      trajectory_point.transform.translation.z = pose.pose.position.z;
      trajectory_point.transform.rotation = pose.pose.orientation;
      trajectory_point.time_from_start = rclcpp::Duration::from_seconds(-1.0);
      trajectory.points.push_back(trajectory_point);

      if (previous_pose != nullptr)
      {
        remaining_path_dist +=
            std::hypot(pose.pose.position.x - previous_pose->pose.position.x,
                       pose.pose.position.y - previous_pose->pose.position.y);
      }
      previous_pose = &pose;
    }

    rclcpp::Duration time_to_goal_duration = rclcpp::Duration(robot_time_to_goal.time_to_goal);
    rclcpp::Duration extra_duration = rclcpp::Duration::from_seconds(remaining_path_dist / max_vel_x_);
    robot_time_to_goal_full.time_to_goal = time_to_goal_duration + extra_duration;

    std_msgs::msg::Float32 ttg_msg;
    ttg_msg.data = rclcpp::Duration(robot_time_to_goal_full.time_to_goal).seconds();
    ttg_pub_->publish(ttg_msg);

    if (!trajectory.points.empty())
    {
      local_traj_pub_->publish(trajectory);
      robot_traj_time_pub_->publish(robot_time_to_goal);
      robot_path_time_pub_->publish(robot_time_to_goal_full);
    }
  }

  void TebVisualization::publishAgentLocalPlansAndPoses(
      const std::map<uint64_t, TimedElasticBand> &agents_tebs_map,
      const BaseRobotFootprintModel &agent_model, const double fp_size, const std_msgs::msg::ColorRGBA &color)
  {
    if (printErrorWhenNotInitialized() || agents_tebs_map.empty() ||
        (!publish_agents_local_plans_ &&
         !publish_agents_local_plan_poses_ &&
         !publish_agents_local_plan_fp_poses_))
    {
      return;
    }

    auto now = node_->now();
    auto frame_id = map_frame_;

    // create pose array for all agents
    geometry_msgs::msg::PoseArray agents_teb_poses;
    agents_teb_poses.header.frame_id = frame_id;
    agents_teb_poses.header.stamp = now;

    cohan_msgs::msg::AgentPathArray agent_path_array;
    agent_path_array.header.stamp = now;
    agent_path_array.header.frame_id = frame_id;
    for (auto &agent_teb_kv : agents_tebs_map)
    {
      auto &agent_id = agent_teb_kv.first;
      auto &agent_teb = agent_teb_kv.second;

      cohan_msgs::msg::AgentPath path;
      path.header.stamp = now;
      path.header.frame_id = frame_id;

      if (agent_teb.sizePoses() == 0)
      {
        continue;
      }

      double pose_time = 0.0;
      for (unsigned int i = 0; i < agent_teb.sizePoses(); i++)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = frame_id;
        pose.pose.position.x = agent_teb.Pose(i).x();
        pose.pose.position.y = agent_teb.Pose(i).y();
        pose.pose.position.z = pose_time * pose_array_z_scale_;
        tf2::Quaternion q;
        q.setRPY(0, 0, agent_teb.Pose(i).theta());
        pose.pose.orientation = tf2::toMsg(q);
        agents_teb_poses.poses.push_back(pose.pose);
        pose.pose.position.z = 0;
        path.path.poses.push_back(pose);
        if (i < (agent_teb.sizePoses() - 1))
        {
          pose_time += agent_teb.TimeDiff(i);
        }
        if (i == 0)
        {
          agent_next_pose_pub_->publish(pose);
        }
      }
      agent_path_array.paths.push_back(path);
    }

    // if (!teb_path.poses.empty() && publish_robot_local_plan_) {
    //   local_plan_pub_->publish(teb_path);
    // }

    if (!agents_teb_poses.poses.empty())
    {
      if (publish_agents_local_plan_poses_)
      {
        agents_tebs_poses_pub_->publish(agents_teb_poses);
      }

      if (publish_agents_local_plans_)
      {
        agents_local_plans_pub_->publish(agent_path_array);
      }

      if (publish_agents_local_plan_fp_poses_)
      {
        visualization_msgs::msg::MarkerArray agents_teb_fp_poses;
        int idx = 0;
        // double fp_size =agents_teb_poses.poses.size();
        // auto now = ros::Time::now();
        for (auto &pose : agents_teb_poses.poses)
        {
          std::vector<visualization_msgs::msg::Marker> agent_fp_markers;
          agent_model.visualizeRobot(pose, agent_fp_markers, color);
          for (auto &agent_marker : agent_fp_markers)
          {
            agent_marker.header.frame_id = map_frame_;
            agent_marker.header.stamp = node_->now();
            agent_marker.action = visualization_msgs::msg::Marker::ADD;
            agent_marker.ns = AGENT_FP_POSES_NS;
            agent_marker.pose.position.z = vel_agent[idx] / 2;
            agent_marker.scale.z = std::max(vel_agent[idx], 0.00001);
            agent_marker.id = idx++;
            agent_marker.color.a = 0.5;
            setMarkerColour(agent_marker, (double)idx, fp_size);
            agent_marker.scale.x = 0.2;
            agent_marker.scale.y = 0.2;
            agent_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            agents_teb_fp_poses.markers.push_back(agent_marker);
          }
        }
        while (idx < last_agent_fp_poses_idx_)
        {
          visualization_msgs::msg::Marker clean_fp_marker;
          clean_fp_marker.header.frame_id = map_frame_;
          clean_fp_marker.header.stamp = node_->now();
          clean_fp_marker.action = visualization_msgs::msg::Marker::DELETE;
          clean_fp_marker.id = idx++;
          clean_fp_marker.ns = AGENT_FP_POSES_NS;
          agents_teb_fp_poses.markers.push_back(clean_fp_marker);
        }
        last_agent_fp_poses_idx_ = idx;

        agents_tebs_fp_poses_pub_->publish(agents_teb_fp_poses);
      }
    }
  }

  void TebVisualization::publishAgentTrajectories(
      const std::vector<AgentPlanTrajCombined> &agents_plans_traj_combined)
  {
    if (printErrorWhenNotInitialized() ||
        !publish_agents_local_plans_)
    {
      return;
    }
    vel_agent.clear();

    auto now = node_->now();
    auto frame_id = map_frame_;

    cohan_msgs::msg::AgentTrajectoryArray agent_path_trajectory_array;
    agent_path_trajectory_array.header.stamp = now;
    agent_path_trajectory_array.header.frame_id = frame_id;

    cohan_msgs::msg::AgentTimeToGoalArray agent_time_to_goal_array;
    agent_time_to_goal_array.header.stamp = now;
    agent_time_to_goal_array.header.frame_id = frame_id;

    cohan_msgs::msg::AgentTimeToGoalArray agent_time_to_goal_array_full;
    agent_time_to_goal_array_full.header.stamp = now;
    agent_time_to_goal_array_full.header.frame_id = frame_id;

    for (auto &agent_plan_traj_combined : agents_plans_traj_combined)
    {
      cohan_msgs::msg::AgentTrajectory agent_path_trajectory;
      agent_path_trajectory.header.stamp = now;
      agent_path_trajectory.header.frame_id = frame_id;
      agent_path_trajectory.trajectory.header.stamp = now;
      agent_path_trajectory.trajectory.header.frame_id = frame_id;

      cohan_msgs::msg::AgentTimeToGoal agent_time_to_goal;
      agent_time_to_goal.header.stamp = now;
      agent_time_to_goal.header.frame_id = frame_id;

      for (auto &agent_pose : agent_plan_traj_combined.plan_before)
      {
        cohan_msgs::msg::TrajectoryPoint agent_path_trajectory_point;
        agent_path_trajectory_point.transform.translation.x =
            agent_pose.pose.position.x;
        agent_path_trajectory_point.transform.translation.y =
            agent_pose.pose.position.y;
        agent_path_trajectory_point.transform.translation.z =
            agent_pose.pose.position.z;
        agent_path_trajectory_point.transform.rotation = agent_pose.pose.orientation;
        agent_path_trajectory_point.time_from_start = rclcpp::Duration::from_seconds(-1.0);
        agent_path_trajectory.trajectory.points.push_back(agent_path_trajectory_point);
      }

      for (auto &agent_traj_point :
           agent_plan_traj_combined.optimized_trajectory)
      {
        cohan_msgs::msg::TrajectoryPoint agent_path_trajectory_point;
        agent_path_trajectory_point.transform.translation.x =
            agent_traj_point.pose.position.x;
        agent_path_trajectory_point.transform.translation.y =
            agent_traj_point.pose.position.y;
        agent_path_trajectory_point.transform.translation.z =
            agent_traj_point.pose.position.z;
        agent_path_trajectory_point.transform.rotation =
            agent_traj_point.pose.orientation;
        agent_path_trajectory_point.velocity = agent_traj_point.velocity;
        auto h_vel = std::hypot(agent_traj_point.velocity.linear.x, agent_traj_point.velocity.linear.y);
        vel_agent.push_back(h_vel);

        agent_path_trajectory_point.time_from_start = agent_traj_point.time_from_start;
        agent_path_trajectory.trajectory.points.push_back(agent_path_trajectory_point);
      }

      if (agent_plan_traj_combined.optimized_trajectory.size() > 0)
      {
        agent_time_to_goal.time_to_goal = agent_path_trajectory.trajectory.points.back().time_from_start;
      }
      else
      {
        agent_time_to_goal.time_to_goal = rclcpp::Duration::from_seconds(0);
      }

      double remaining_path_dist = 0.0;
      const geometry_msgs::msg::PoseStamped *previous_agent_pose = nullptr;
      for (auto agent_pose : agent_plan_traj_combined.plan_after)
      {
        cohan_msgs::msg::TrajectoryPoint agent_path_trajectory_point;
        agent_path_trajectory_point.transform.translation.x =
            agent_pose.pose.position.x;
        agent_path_trajectory_point.transform.translation.y =
            agent_pose.pose.position.y;
        agent_path_trajectory_point.transform.translation.z =
            agent_pose.pose.position.z;
        agent_path_trajectory_point.transform.rotation = agent_pose.pose.orientation;
        agent_path_trajectory_point.time_from_start = rclcpp::Duration::from_seconds(-1.0);
        agent_path_trajectory.trajectory.points.push_back(agent_path_trajectory_point);
        if (previous_agent_pose != nullptr)
        {
          remaining_path_dist +=
              std::hypot(agent_pose.pose.position.x - previous_agent_pose->pose.position.x,
                         agent_pose.pose.position.y - previous_agent_pose->pose.position.y);
        }
        previous_agent_pose = &agent_pose;
      }

      if (!agent_path_trajectory.trajectory.points.empty())
      {
        agent_path_trajectory.id = agent_plan_traj_combined.id;
        agent_path_trajectory_array.trajectories.push_back(agent_path_trajectory);

        agent_time_to_goal.id = agent_plan_traj_combined.id;
        agent_time_to_goal_array.times_to_goal.push_back(agent_time_to_goal);

        rclcpp::Duration additional_time = rclcpp::Duration::from_seconds(remaining_path_dist / nominal_vel_x_);

        builtin_interfaces::msg::Duration updated_duration = agent_time_to_goal.time_to_goal;
        updated_duration.sec += additional_time.seconds();
        updated_duration.nanosec += additional_time.nanoseconds() % 1000000000;

        // Normalize the time in case nanosec overflows
        if (updated_duration.nanosec >= 1000000000)
        {
          updated_duration.sec += 1;
          updated_duration.nanosec -= 1000000000;
        }

        agent_time_to_goal.time_to_goal = updated_duration;
        agent_time_to_goal_array_full.times_to_goal.push_back(agent_time_to_goal);
      }
    }

    if (!agent_path_trajectory_array.trajectories.empty())
    {
      agents_local_trajs_pub_->publish(agent_path_trajectory_array);
      agent_trajs_time_pub_->publish(agent_time_to_goal_array);
      agent_paths_time_pub_->publish(agent_time_to_goal_array_full);
    }
  }

  void TebVisualization::publishMode(int Mode)
  {

    geometry_msgs::msg::TransformStamped robot_to_map_tf;
    tf2::Transform start_pose_tr;
    bool transform_found = false;
    try
    {
      std::string base = "base_footprint";
      if (ns_ != "")
      {
      }
      base = ns_ + "/" + base;

      robot_to_map_tf = tf_buffer_->lookupTransform("map", base, tf2::TimePointZero);
      transform_found = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    }

    geometry_msgs::msg::Transform robot_behind_pose;
    if (transform_found)
    {
      tf2::Transform offset_transform;
      offset_transform.setOrigin(tf2::Vector3(-1.0, 0.0, 0.0));
      offset_transform.setRotation(tf2::Quaternion(0, 0, 0, 1)); // yaw 0.0

      tf2::Transform map_to_robot;
      tf2::fromMsg(robot_to_map_tf.transform, map_to_robot);

      start_pose_tr = map_to_robot * offset_transform;
      robot_behind_pose = tf2::toMsg(start_pose_tr);
    }

    visualization_msgs::msg::Marker mode_text;
    mode_text.header.frame_id = "map";
    mode_text.header.stamp = node_->now();
    mode_text.ns = "mode";
    mode_text.id = 1;
    mode_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mode_text.action = visualization_msgs::msg::Marker::ADD;
    mode_text.pose.position.x = robot_behind_pose.translation.x;
    mode_text.pose.position.y = robot_behind_pose.translation.y;
    mode_text.pose.position.z = 2.0;
    mode_text.pose.orientation = robot_behind_pose.rotation;
    // mode_text.pose.orientation.x = 0.0;
    // mode_text.pose.orientation.y = 0.0;
    // mode_text.pose.orientation.z = 0.0;
    // mode_text.pose.orientation.w = 1.0;

    if (Mode == -1)
      mode_text.text = "SingleBand";
    else if (Mode == 0)
      mode_text.text = "DualBand";
    else if (Mode == 1)
      mode_text.text = "VelObs";
    else if (Mode == 2)
      mode_text.text = "Backoff";
    else if (Mode == 3)
      mode_text.text = "PassingThrough";
    else if (Mode == 4)
      mode_text.text = "ApproachingPillar";
    else if (Mode == 5)
      mode_text.text = "ApproachingGoal";
    else
      mode_text.text = "No Mode yet";

    mode_text.scale.x = 10.0;
    mode_text.scale.y = 10.0;
    mode_text.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    mode_text.color.r = 0.0f;
    mode_text.color.g = 0.0f;
    mode_text.color.b = 0.0f;
    mode_text.color.a = 0.9;

    mode_text.lifetime = rclcpp::Duration::from_seconds(2.0);
    mode_text_pub->publish(mode_text);
  }

  void TebVisualization::publishTrackedAgents(const cohan_msgs::msg::TrackedAgents::SharedPtr agents)
  {
    visualization_msgs::msg::MarkerArray marker_arr, arrow_arr;

    int i = 0;
    for (auto &agent : agents->agents)
    {
      visualization_msgs::msg::Marker marker, arrow;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = node_->now();
      arrow.header.frame_id = "map";
      arrow.header.stamp = node_->now();

      for (auto segment : agent.segments)
      { // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "body";
        marker.id = i;
        arrow.ns = "direction";
        arrow.id = i;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        arrow.type = visualization_msgs::msg::Marker::ARROW;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::msg::Marker::ADD;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = segment.pose.pose.position.x;
        marker.pose.position.y = segment.pose.pose.position.y;
        marker.pose.position.z = 0.9;
        marker.pose.orientation = segment.pose.pose.orientation;

        arrow.pose.position.x = segment.pose.pose.position.x;
        arrow.pose.position.y = segment.pose.pose.position.y;
        arrow.pose.position.z = 0.0;
        arrow.pose.orientation = segment.pose.pose.orientation;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 1.8;

        arrow.scale.x = 0.8;
        arrow.scale.y = 0.05;
        arrow.scale.z = 0.05;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        arrow.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_arr.markers.push_back(marker);
        arrow_arr.markers.push_back(arrow);
        i++;
      }
    }
    agent_marker_pub->publish(marker_arr);
    agent_arrow_pub->publish(arrow_arr);
  }

  void TebVisualization::setMarkerColour(visualization_msgs::msg::Marker &marker, double itr, double n)
  {
    double N = n / 11;

    if (itr >= N && itr < 3 * N)
    {

      marker.color.r = (3 * N - itr) / (2 * N);
      marker.color.g = 1.0;
      marker.color.b = 0;
    }
    else if (itr >= 3 * N && itr < 5 * N)
    {

      marker.color.r = 0;
      marker.color.g = 1.0;
      marker.color.b = (itr - 3 * N) / (2 * N);
    }
    else if (itr >= 5 * N && itr < 7 * N)
    {

      marker.color.r = 0;
      marker.color.g = (7 * N - itr) / (2 * N);
      marker.color.b = 1.0;
    }
    else if (itr >= 7 * N && itr < 9 * N)
    {

      marker.color.r = (itr - 7 * N) / (2 * N);
      marker.color.g = 0;
      marker.color.b = 1.0;
    }
    else if (itr >= 9 * N && itr <= n)
    {
      marker.color.r = 1.0;
      marker.color.g = 0;
      marker.color.b = (11 * N - itr) / (2 * N);
    }
  }

  void TebVisualization::publishRobotFootprintModel(const PoseSE2 &current_pose, const BaseRobotFootprintModel &robot_model, const std::string &ns,
                                                    const std_msgs::msg::ColorRGBA &color)
  {
    if (printErrorWhenNotInitialized())
      return;

    std::vector<visualization_msgs::msg::Marker> markers;
    robot_model.visualizeRobot(current_pose, markers, color);
    if (markers.empty())
      return;

    int idx = 1000000; // avoid overshadowing by obstacles
    for (std::vector<visualization_msgs::msg::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
    {
      marker_it->header.frame_id = map_frame_;
      marker_it->header.stamp = node_->now();
      marker_it->action = visualization_msgs::msg::Marker::ADD;
      marker_it->ns = ns;
      marker_it->id = idx;
      marker_it->lifetime = rclcpp::Duration::from_seconds(2.0);
      teb_marker_pub_->publish(*marker_it);
    }
  }

  void TebVisualization::publishInfeasibleRobotPose(const PoseSE2 &current_pose, const BaseRobotFootprintModel &robot_model)
  {
    publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
  }

  void TebVisualization::publishObstacles(const ObstContainer &obstacles) const
  {
    if (obstacles.empty() || printErrorWhenNotInitialized())
      return;

    // Visualize point obstacles
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = node_->now();
      marker.ns = "PointObstacles";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(2.0);

      for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
      {
        boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);
        if (!pobst)
          continue;

        if (visualize_with_time_as_z_axis_scale_ < 0.001)
        {
          geometry_msgs::msg::Point point;
          point.x = pobst->x();
          point.y = pobst->y();
          point.z = 0;
          marker.points.push_back(point);
        }
        else // Spatiotemporally point obstacles become a line
        {
          marker.type = visualization_msgs::msg::Marker::LINE_LIST;
          geometry_msgs::msg::Point start;
          start.x = pobst->x();
          start.y = pobst->y();
          start.z = 0;
          marker.points.push_back(start);

          geometry_msgs::msg::Point end;
          double t = 20;
          Eigen::Vector2d pred;
          pobst->predictCentroidConstantVelocity(t, pred);
          end.x = pred[0];
          end.y = pred[1];
          end.z = visualize_with_time_as_z_axis_scale_ * t;
          marker.points.push_back(end);
        }
      }

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      teb_marker_pub_->publish(marker);
    }

    // Visualize line obstacles
    {
      std::size_t idx = 0;
      for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
      {
        boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);
        if (!pobst)
          continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "LineObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        geometry_msgs::msg::Point start;
        start.x = pobst->start().x();
        start.y = pobst->start().y();
        start.z = 0;
        marker.points.push_back(start);
        geometry_msgs::msg::Point end;
        end.x = pobst->end().x();
        end.y = pobst->end().y();
        end.z = 0;
        marker.points.push_back(end);

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        teb_marker_pub_->publish(marker);
      }
    }

    // Visualize polygon obstacles
    {
      std::size_t idx = 0;
      for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
      {
        boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(*obst);
        if (!pobst)
          continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "PolyObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(2.0);

        for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
        {
          geometry_msgs::msg::Point point;
          point.x = vertex->x();
          point.y = vertex->y();
          point.z = 0;
          marker.points.push_back(point);
        }

        // Also add last point to close the polygon
        // but only if polygon has more than 2 points (it is not a line)
        if (pobst->vertices().size() > 2)
        {
          geometry_msgs::msg::Point point;
          point.x = pobst->vertices().front().x();
          point.y = pobst->vertices().front().y();
          point.z = 0;
          marker.points.push_back(point);
        }
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        teb_marker_pub_->publish(marker);
      }
    }
  }

  void TebVisualization::publishViaPoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &via_points, const std::string &ns) const
  {
    if (via_points.empty() || printErrorWhenNotInitialized())
      return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);

    for (std::size_t i = 0; i < via_points.size(); ++i)
    {
      geometry_msgs::msg::Point point;
      point.x = via_points[i].x();
      point.y = via_points[i].y();
      point.z = 0;
      marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    teb_marker_pub_->publish(marker);
  }

  void TebVisualization::publishTebContainer(const TebOptPlannerContainer &teb_planner, const std::string &ns)
  {
    if (printErrorWhenNotInitialized())
      return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Iterate through teb pose sequence
    for (TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb)
    {
      // iterate single poses
      PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
      TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
      PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
      std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one.
      double time = 0;

      while (it_pose != it_pose_end)
      {
        geometry_msgs::msg::Point point_start;
        point_start.x = (*it_pose)->x();
        point_start.y = (*it_pose)->y();
        point_start.z = visualize_with_time_as_z_axis_scale_ * time;
        marker.points.push_back(point_start);

        time += (*it_timediff)->dt();

        geometry_msgs::msg::Point point_end;
        point_end.x = (*boost::next(it_pose))->x();
        point_end.y = (*boost::next(it_pose))->y();
        point_end.z = visualize_with_time_as_z_axis_scale_ * time;
        marker.points.push_back(point_end);
        ++it_pose;
        ++it_timediff;
      }
    }
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    teb_marker_pub_->publish(marker);
  }

  void TebVisualization::publishFeedbackMessage(const std::vector<boost::shared_ptr<TebOptimalPlanner>> &teb_planners,
                                                unsigned int selected_trajectory_idx, const ObstContainer &obstacles)
  {
    cohan_msgs::msg::FeedbackMsg msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = map_frame_;
    msg.selected_trajectory_idx = selected_trajectory_idx;

    msg.trajectories.resize(teb_planners.size());

    // Iterate through teb pose sequence
    std::size_t idx_traj = 0;
    for (TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj)
    {
      msg.trajectories[idx_traj].header = msg.header;
      it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
    }

    // add obstacles
    msg.obstacles_msg.obstacles.resize(obstacles.size());
    for (std::size_t i = 0; i < obstacles.size(); ++i)
    {
      msg.obstacles_msg.header = msg.header;

      // copy polygon
      msg.obstacles_msg.obstacles[i].header = msg.header;
      obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

      // copy id
      msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

      // orientation
      // msg.obstacles_msg.obstacles[i].orientation =; // TODO

      // copy velocities
      obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
    }

    feedback_pub_->publish(msg);
  }

  void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner &teb_planner, const ObstContainer &obstacles)
  {
    cohan_msgs::msg::FeedbackMsg msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = map_frame_;
    msg.selected_trajectory_idx = 0;

    msg.trajectories.resize(1);
    msg.trajectories.front().header = msg.header;
    teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);

    // add obstacles
    msg.obstacles_msg.obstacles.resize(obstacles.size());
    for (std::size_t i = 0; i < obstacles.size(); ++i)
    {
      msg.obstacles_msg.header = msg.header;

      // copy polygon
      msg.obstacles_msg.obstacles[i].header = msg.header;
      obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

      // copy id
      msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

      // orientation
      // msg.obstacles_msg.obstacles[i].orientation =; // TODO

      // copy velocities
      obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
    }

    feedback_pub_->publish(msg);
  }

  std_msgs::msg::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  bool TebVisualization::printErrorWhenNotInitialized() const
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(node_->get_logger(), "TebVisualization class not initialized. You must call initialize or an appropriate constructor");
      return true;
    }
    return false;
  }

  void TebVisualization::clearingTimerCB()
  {
    if ((last_publish_robot_global_plan_ !=
         publish_robot_global_plan_) &&
        !publish_robot_global_plan_)
    {
      // clear robot global plans
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = node_->now();
      empty_path.header.frame_id = map_frame_;
      global_plan_pub_->publish(empty_path);
    }
    last_publish_robot_global_plan_ =
        publish_robot_global_plan_;

    if ((last_publish_robot_local_plan_ !=
         publish_robot_local_plan_) &&
        !publish_robot_local_plan_)
    {
      // clear robot local plans
      nav_msgs::msg::Path empty_path;
      cohan_msgs::msg::Trajectory empty_traj;
      empty_path.header.stamp = node_->now();
      empty_path.header.frame_id = map_frame_;
      local_plan_pub_->publish(empty_path);
      local_traj_pub_->publish(empty_traj);
    }
    last_publish_robot_local_plan_ = publish_robot_local_plan_;

    if ((last_publish_robot_local_plan_poses_ !=
         publish_robot_local_plan_poses_) &&
        !publish_robot_local_plan_poses_)
    {
      // clear robot local plan poses
      geometry_msgs::msg::PoseArray empty_pose_array;
      empty_pose_array.header.stamp = node_->now();
      empty_pose_array.header.frame_id = map_frame_;
      teb_poses_pub_->publish(empty_pose_array);
    }
    last_publish_robot_local_plan_poses_ =
        publish_robot_local_plan_poses_;

    if ((last_publish_robot_local_plan_fp_poses_ !=
         publish_robot_local_plan_fp_poses_) &&
        !publish_robot_local_plan_fp_poses_)
    {
      // clear robot local plan fp poses
      visualization_msgs::msg::Marker clean_fp_poses;
      clean_fp_poses.header.frame_id = map_frame_;
      clean_fp_poses.header.stamp = node_->now();
      clean_fp_poses.action = 3; // visualization_msgs::msg::Marker::DELETEALL;
      clean_fp_poses.ns = ROBOT_FP_POSES_NS;
      visualization_msgs::msg::MarkerArray clean_fp_poses_array;
      clean_fp_poses_array.markers.push_back(clean_fp_poses);
      teb_fp_poses_pub_->publish(clean_fp_poses_array);
    }
    last_publish_robot_local_plan_fp_poses_ =
        publish_robot_local_plan_fp_poses_;

    if ((last_publish_agents_global_plans_ !=
         publish_agents_global_plans_) &&
        !publish_agents_global_plans_)
    {
      // clear agent global plans
      cohan_msgs::msg::AgentPathArray empty_path_array;
      empty_path_array.header.stamp = node_->now();
      empty_path_array.header.frame_id = map_frame_;
      agents_global_plans_pub_->publish(empty_path_array);
    }
    last_publish_agents_global_plans_ =
        publish_agents_global_plans_;

    if ((last_publish_agents_local_plans_ !=
         publish_agents_local_plans_) &&
        !publish_agents_local_plans_)
    {
      // clear agent local plans
      cohan_msgs::msg::AgentPathArray empty_trajectory_array;
      empty_trajectory_array.header.stamp = node_->now();
      empty_trajectory_array.header.frame_id = map_frame_;
      agents_local_plans_pub_->publish(empty_trajectory_array);
    }
    last_publish_agents_local_plans_ =
        publish_agents_local_plans_;

    if ((last_publish_agents_local_plan_poses_ !=
         publish_agents_local_plan_poses_) &&
        !publish_agents_local_plan_poses_)
    {
      // clear agent local plan poses
      geometry_msgs::msg::PoseArray empty_pose_array;
      empty_pose_array.header.stamp = node_->now();
      empty_pose_array.header.frame_id = map_frame_;
      agents_tebs_poses_pub_->publish(empty_pose_array);
    }
    last_publish_agents_local_plan_poses_ =
        publish_agents_local_plan_poses_;

    if ((last_publish_agents_local_plan_fp_poses_ !=
         publish_agents_local_plan_fp_poses_) &&
        !publish_agents_local_plan_fp_poses_)
    {
      // clear agent local plan fp poses
      visualization_msgs::msg::Marker clean_fp_poses;
      clean_fp_poses.header.frame_id = map_frame_;
      clean_fp_poses.header.stamp = node_->now();
      clean_fp_poses.action = 3; // visualization_msgs::msg::Marker::DELETEALL;
      clean_fp_poses.ns = AGENT_FP_POSES_NS;
      visualization_msgs::msg::MarkerArray clean_fp_poses_array;
      clean_fp_poses_array.markers.push_back(clean_fp_poses);
      agents_tebs_fp_poses_pub_->publish(clean_fp_poses_array);
    }
    last_publish_agents_local_plan_fp_poses_ =
        publish_agents_local_plan_fp_poses_;
  }

} // namespace hateb_local_planner
