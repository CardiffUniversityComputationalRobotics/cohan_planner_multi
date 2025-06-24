/*! \file planning_framework_main.cpp
 * \brief Framework for planning collision-free paths.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Framework for planning collision-free paths online. Iterative planning
 * uses last known solution path in order to guarantee that new solution paths are always
 * at least as good as the previous one.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <iostream>
#include <vector>

#include <boost/bind.hpp>
#include <math.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/path.hpp>

class HATEBPlanningFramework : public rclcpp::Node
{
public:
    //! Constructor
    HATEBPlanningFramework();
    //! Planner setup
    void planWithSimpleSetup();
    //! Periodic callback to solve the query
    void planningTimerCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg);
    //! Callback for getting the state of the Smf base controller
    void controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg);
    uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::TwistStamped &velocity, geometry_msgs::msg::TwistStamped &cmd_vel, std::string &message);

private:
    // ! SUBSCRIBERS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_active_sub_;

    // ! PUBLISHERS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_motion_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr query_goal_pose_rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr query_goal_radius_rviz_pub_;

    // ROS2 TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform last_robot_pose_;

    double timer_period_, robot_base_radius_;

    bool odom_available_, goal_available_, control_active_;
    std::vector<double> start_state_, goal_map_frame_, goal_odom_frame_;
    double goal_radius_, xy_goal_tolerance_, yaw_goal_tolerance_, local_goal_radius_, local_path_range_, global_time_percent_, max_trans_vel_, max_rot_vel_;
    std::string odometry_topic_, query_goal_topic_, solution_path_topic_, world_frame_, control_active_topic_;

    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    geometry_msgs::msg::Twist current_robot_velocity_;

    // configs params
    rclcpp::Time last_call_time_;
};

//!  Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
 */
HATEBPlanningFramework::HATEBPlanningFramework()
    : Node("hateb_planning_framework"), control_active_(false)
{

    //=======================================================================
    // TF LISTENER
    //=======================================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //=======================================================================
    // Get parameters
    //=======================================================================

    // ! DECLARE PARAMETERS
    this->declare_parameter("world_frame", rclcpp::ParameterValue(std::string("map")));
    this->declare_parameter("start_state", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
    this->declare_parameter("goal_state", rclcpp::ParameterValue(std::vector<double>{5.0, 0.0, 0.0}));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.52));
    this->declare_parameter("odometry_topic", rclcpp::ParameterValue(std::string("/odom")));
    this->declare_parameter("query_goal_topic", rclcpp::ParameterValue(std::string("/tidup_move_base_planner/query_goal")));
    this->declare_parameter("solution_path_topic", rclcpp::ParameterValue(std::string("/tidup_move_base_planner/solution_path")));
    this->declare_parameter("robot_base_radius", rclcpp::ParameterValue(0.35));
    this->declare_parameter("max_trans_vel", rclcpp::ParameterValue(0.3));
    this->declare_parameter("max_rot_vel", rclcpp::ParameterValue(1.2));

    // ! GET PARAMETERS
    world_frame_ = this->get_parameter("world_frame").as_string();
    start_state_ = this->get_parameter("start_state").as_double_array();
    goal_map_frame_ = this->get_parameter("goal_state").as_double_array();
    timer_period_ = this->get_parameter("timer_period").as_double();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    query_goal_topic_ = this->get_parameter("query_goal_topic").as_string();
    solution_path_topic_ = this->get_parameter("solution_path_topic").as_string();
    robot_base_radius_ = this->get_parameter("robot_base_radius").as_double();
    max_trans_vel_ = this->get_parameter("max_trans_vel").as_double();
    max_rot_vel_ = this->get_parameter("max_rot_vel").as_double();

    start_state_.resize(3);

    goal_available_ = false;

    //=======================================================================
    // ! Subscribers
    //=======================================================================
    // Odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 1, std::bind(&HATEBPlanningFramework::odomCallback, this, std::placeholders::_1));
    odom_available_ = false;

    // 2D Nav Goal
    nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(query_goal_topic_, 1, std::bind(&HATEBPlanningFramework::queryGoalCallback, this, std::placeholders::_1));

    // Controller active flag
    // control_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(control_active_topic_, 1, std::bind(&OnlinePlannFramework::controlActiveCallback, this, std::placeholders::_1));

    //=======================================================================
    // ! Publishers
    //=======================================================================
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 1);
    stop_motion_pub_ = this->create_publisher<std_msgs::msg::Bool>("stop_motion", 1);

    query_goal_pose_rviz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("query_goal_pose_rviz", 1);
    query_goal_radius_rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("query_goal_radius_rviz", 1);

    // while (!grid_map_client_->wait_for_service(1s))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //     }
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    // }

    //=======================================================================
    // Action server
    //=======================================================================
    // goto_action_server_ = new SmfBaseGoToActionServer(
    //     this, goto_action_, std::bind(&HATEBPlanningFramework::goToActionCallback, this, std::placeholders::_1), false);

    //=======================================================================
    // ! Waiting for odometry
    //=======================================================================
    rclcpp::Rate loop_rate(5);
    while (rclcpp::ok() && !odom_available_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        RCLCPP_WARN(this->get_logger(), "Waiting for vehicle's odometry");
    }
    RCLCPP_WARN(this->get_logger(), "Odometry received");

    // goto_action_server_->start();
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry
 */
void HATEBPlanningFramework::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    if (!odom_available_)
        odom_available_ = true;

    geometry_msgs::msg::Pose predictedPose = odom_msg->pose.pose;

    predictedPose.position.x = odom_msg->pose.pose.position.x;

    predictedPose.position.y = odom_msg->pose.pose.position.y;

    tf2::fromMsg(predictedPose, last_robot_pose_);

    double useless_pitch,
        useless_roll, yaw;
    tf2::Matrix3x3(last_robot_pose_.getRotation()).getEulerYPR(yaw, useless_pitch, useless_roll);

    if ((goal_available_) &&
        sqrt(pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
             pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) < (goal_radius_ + 0.2) &&
        abs(yaw - goal_odom_frame_[2]) < (yaw_goal_tolerance_ + 0.2))
    {
        goal_available_ = false;
        std_msgs::msg::Bool goal_reached;
        goal_reached.data = true;
        goal_reached_pub_->publish(goal_reached);
        RCLCPP_WARN(this->get_logger(), "Goal reached");
    }

    current_robot_velocity_ = odom_msg->twist.twist;
    odom_data_ = odom_msg;
}

//! Control active callback.
/*!
 * Callback for getting the state of the Smf base controller
 */
void HATEBPlanningFramework::controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg)
{
    control_active_ = control_active_msg->data;
}

//! Navigation goal callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void HATEBPlanningFramework::queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr query_goal_msg)
{
    double useless_pitch, useless_roll, yaw;
    tf2::Quaternion q(query_goal_msg->pose.orientation.x, query_goal_msg->pose.orientation.y,
                      query_goal_msg->pose.orientation.z, query_goal_msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(useless_roll, useless_pitch, yaw);

    goal_map_frame_[0] = query_goal_msg->pose.position.x; // x
    goal_map_frame_[1] = query_goal_msg->pose.position.y; // y
    goal_map_frame_[2] = yaw;

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    geometry_msgs::msg::TransformStamped tf_map_to_fixed;
    try
    {
        tf_map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        return;
    }

    tf2::Transform tf2_map_to_fixed;
    tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
    tf2::Matrix3x3(tf2_map_to_fixed.getRotation()).getRPY(useless_roll, useless_pitch, yaw);

    tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf2_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.x();
    goal_odom_frame_[1] = goal_point_odom_frame.y();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    //=======================================================================
    // Clean and merge octomap
    //=======================================================================
    // ! COMMENTED TO AVOID UNNEEDED PROCESSING
    // while (nh_.ok() && !ros::service::call("/tidup_move_base_mapper/clean_merge_octomap", req, resp))  //
    // TODO
    // {
    //     ROS_WARN("Request to %s failed; trying again...",
    //              nh_.resolveName("/tidup_move_base_mapper/clean_merge_octomap").c_str());
    //     usleep(1000000);
    // }
    goal_available_ = true;

    //=======================================================================
    // Publish RViz Maker
    //=======================================================================
    query_goal_pose_rviz_pub_->publish(*query_goal_msg);

    visualization_msgs::msg::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = this->now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::msg::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::msg::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goal_map_frame_[0];
    radius_msg.pose.position.y = goal_map_frame_[1];
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_->publish(radius_msg);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void HATEBPlanningFramework::planWithSimpleSetup()
{

    rclcpp::Rate loop_rate(1.0 / (timer_period_)); // 10 hz
    // goal_available_ = true;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        if (goal_available_)
            RCLCPP_INFO(this->get_logger(), "goal available");

        HATEBPlanningFramework::planningTimerCallback();
        loop_rate.sleep();
    }
}

//!  Periodic callback to solve the query.
/*!
 * Solve the query.
 */
void HATEBPlanningFramework::planningTimerCallback()
{
    if (goal_available_)
    {
    }
}

uint32_t HATEBPlanningFramework::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                         const geometry_msgs::msg::TwistStamped &velocity,
                                                         geometry_msgs::msg::TwistStamped &cmd_vel, std::string &message)
{
    auto start_time = this->get_clock()->now();
    if ((start_time - last_call_time_).toSec() >
        cfg_.hateb.pose_prediction_reset_time)
    {
        resetAgentsPrediction();
    }
    last_call_time_ = start_time;

    // check if plugin initialized
    logs.clear();
    if (!initialized_)
    {
        ROS_ERROR("hateb_local_planner has not been initialized, please call initialize() before using this planner");
        message = "hateb_local_planner has not been initialized";
        return mbf_msgs::ExePathResult::NOT_INITIALIZED;
    }

    if (reset_states)
    {
        for (int i = 0; i < agents_states_.states.size(); i++)
        {
            agents_states_.states[i] = hateb_local_planner::AgentState::NO_STATE;
            // agents_states_.states[i]=hateb_local_planner::AgentState::STATIC;
        }
        reset_states = false;
    }
    static uint32_t seq = 0;
    cmd_vel.header.seq = seq++;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    goal_reached_ = false;

    // Get robot pose
    auto pose_get_start_time = ros::Time::now();
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    robot_pose_ = PoseSE2(robot_pose.pose);
    robot_pose_.toPoseMsg(robot_pos_msg);

    if ((ros::Time::now() - last_door_pass_detect_).toSec() >= 5.0 && door_pass)
    {
        door_pass = false;
        isMode = 0;
    }
    // robot_pose_pub_.publish(robot_pos_msg);
    logs += "position: " + std::to_string(robot_pose.pose.position.x) + " " + std::to_string(robot_pose.pose.position.y) + "; ";
    // logs+="position: x= " + std::to_string(robot_pose.pose.position.x) +", " + " y= " + std::to_string(robot_pose.pose.position.y)+", ";
    // logs+=std::to_string(robot_pose.pose.position.x) +", " + std::to_string(robot_pose.pose.position.y);
    if (std::hypot(robot_pose.pose.position.x - last_robot_pose.position.x, robot_pose.pose.position.y - last_robot_pose.position.y) > 0.06)
    {
        last_position_time = ros::Time::now();
    }

    last_robot_pose = robot_pose.pose;

    if ((ros::Time::now() - last_position_time).toSec() >= 2.0 && cfg_.robot.type == 0)
    { // 0: Robot and 1: Human for type
        if (visible_agent_ids.size() > 0)
        {
            if (agent_still[visible_agent_ids[0] - 1] && isDistunderThreshold && !stuck)
            {
                if (change_mode == 0)
                {
                    ROS_INFO("I am stuck because of an agent, Changing to VelObs mode");
                }
                change_mode++;
                isMode = 1;

                if (change_mode > 20 && cfg_.hateb.enable_backoff)
                {
                    if (!stuck)
                        ROS_INFO("I am stuck");
                    stuck = true;
                    agents_states_.states[visible_agent_ids[0] - 1] = hateb_local_planner::AgentState::BLOCKED;
                    stuck_agent_id = visible_agent_ids[0];
                    isMode = 2;
                }
            }
        }
    }

    if (!stuck)
        stuck_agent_id = -1;

    auto pose_get_time = ros::Time::now() - pose_get_start_time;

    // Get robot velocity
    auto vel_get_start_time = ros::Time::now();
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    robot_vel_.linear.x = robot_vel_tf.pose.position.x;
    robot_vel_.linear.y = robot_vel_tf.pose.position.y;
    robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
    auto vel_get_time = ros::Time::now() - vel_get_start_time;
    logs += "velocity: " + std::to_string(robot_vel_.linear.x) + " " + std::to_string(robot_vel_.linear.y) + "; ";
    // logs+="velocity: x= " + std::to_string(robot_vel_.linear.x) +", "  + "y= "+ std::to_string(robot_vel_.linear.y)+", ";
    // logs+= std::to_string(robot_vel_.linear.x) +", " + std::to_string(robot_vel_.linear.y)+", ";
    // logs+="dist " + std::to_string(current_agent_dist)+", ";

    // prune global plan to cut off parts of the past (spatially before the robot)
    auto prune_start_time = ros::Time::now();
    pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
    auto prune_time = ros::Time::now() - prune_start_time;

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    auto transform_start_time = ros::Time::now();
    PlanCombined transformed_plan_combined;
    int goal_idx;
    geometry_msgs::TransformStamped tf_plan_to_global;
    if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                             transformed_plan_combined, &goal_idx, &tf_plan_to_global))
    {
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        message = "Could not transform the global plan to the frame of the controller";
        return mbf_msgs::ExePathResult::INTERNAL_ERROR;
    }
    auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
    auto transform_time = ros::Time::now() - transform_start_time;

    // check if we should enter any backup mode and apply settings
    configureBackupModes(transformed_plan, goal_idx);

    // update via-points container
    // if (!custom_via_points_active_)
    //   updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

    auto other_start_time = ros::Time::now();
    // check if global goal is reached
    geometry_msgs::PoseStamped global_goal;
    tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
    double dx = global_goal.pose.position.x - robot_pose_.x();
    double dy = global_goal.pose.position.y - robot_pose_.y();
    double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta());
    if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0) && goal_ctrl)
    {
        goal_reached_ = true;
        return mbf_msgs::ExePathResult::SUCCESS;
    }

    if (fabs(std::sqrt(dx * dx + dy * dy)) < 1.0 && !backed_off)
    {
        door_pass = true;
        isMode = 5;
    }

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
        ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
        message = "Transformed plan is empty";
        return mbf_msgs::ExePathResult::INVALID_PATH;
    }

    // Get current goal point (last point of the transformed plan)
    robot_goal_.x() = transformed_plan.back().pose.position.x;
    robot_goal_.y() = transformed_plan.back().pose.position.y;
    // Overwrite goal orientation if needed
    if (cfg_.trajectory.global_plan_overwrite_orientation)
    {
        robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
        // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_goal_.theta());
        tf2::convert(q, transformed_plan.back().pose.orientation);
    }
    else
    {
        robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    if (transformed_plan.size() == 1) // plan only contains the goal
    {
        transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
    }
    transformed_plan.front() = robot_pose; // update start

    // clear currently existing obstacles
    obstacles_.clear();
    auto other_time = ros::Time::now() - other_start_time;

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    auto cc_start_time = ros::Time::now();
    if (costmap_converter_)
        updateObstacleContainerWithCostmapConverter();
    else
        updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    updateObstacleContainerWithCustomObstacles();
    updateObstacleContainerWithInvHumans();
    auto cc_time = ros::Time::now() - cc_start_time;

    // Do not allow config changes during the following optimization step
    boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

    // update agents
    auto agent_start_time = ros::Time::now();
    std::vector<AgentPlanCombined> transformed_agent_plans;
    AgentPlanVelMap transformed_agent_plan_vel_map;
    agents_states_pub_.publish(agents_states_);

    switch (cfg_.planning_mode)
    {
    case 0:
        goal_ctrl = true;
        break;
    case 1:
    {
        bool found = true;
        if (stuck_agent_id != -1)
        {
            if (visible_agent_ids.size() > 0)
            {
                if (visible_agent_ids[0] != stuck_agent_id)
                    found = false;
            }
            else if (visible_agent_ids.size() == 0)
                found = false;

            // Check for timeout
            if (backed_off && backoff_recovery_.timeOut())
                found = false;
        }

        if (backoff_recovery_.checkRandomRot())
        {
            break;
        }

        if (isDistMax || !found)
        {
            agents_via_points_map_.clear();

            if (visible_agent_ids.size() > 0)
            {
                updateAgentViaPointsContainers(transformed_agent_plan_vel_map,
                                               cfg_.trajectory.global_plan_viapoint_sep);
            }
            if (backed_off)
            {
                if (backoff_recovery_.setbackGoal())
                {
                    isMode = 0;
                    change_mode = 0;
                    backed_off = false;
                    stuck = false;
                    // goal_ctrl = true;
                    if (agents_states_.states[visible_agent_ids[0] - 1] != hateb_local_planner::AgentState::MOVING)
                        agents_states_.states[visible_agent_ids[0] - 1] = hateb_local_planner::AgentState::STATIC;
                }
            }
            else
                goal_ctrl = true;

            break;
        }

        if (backoff_recovery_.checkNewGoal())
        {
            goal_ctrl = true;
            isMode = 0;
            change_mode = 0;
            backed_off = false;
            stuck = false;
            if (agents_states_.states[visible_agent_ids[0] - 1] != hateb_local_planner::AgentState::MOVING)
                agents_states_.states[visible_agent_ids[0] - 1] = hateb_local_planner::AgentState::STATIC;
        }

        agent_path_prediction::AgentPosePredict predict_srv;

        if (isMode == 0)
            isMode = -1;

        std::vector<int> static_agents_ids;

        for (int i = 0; i < 2 && i < visible_agent_ids.size(); i++)
        {
            if ((int)agents_states_.states[visible_agent_ids[i] - 1] > 1)
            {
                predict_srv.request.ids.push_back(visible_agent_ids[i]);
                if (isMode == -1)
                    isMode = 0;
            }
            else
            {
                static_agents_ids.push_back(visible_agent_ids[i]);
            }
        }

        if (cfg_.hateb.use_external_prediction && change_mode < 1)
        {

            std_srvs::Trigger g_srv;
            agent_goal_client_.call(g_srv);
            if (g_srv.response.success && !ext_goal)
                ext_goal = true;

            if (cfg_.hateb.predict_agent_behind_robot && !ext_goal)
            {
                predict_srv.request.type =
                    agent_path_prediction::AgentPosePredictRequest::BEHIND_ROBOT;
            }
            else if (cfg_.hateb.predict_agent_goal && !ext_goal)
            {
                predict_srv.request.type =
                    agent_path_prediction::AgentPosePredictRequest::PREDICTED_GOAL;
            }
            else
            {
                predict_srv.request.type =
                    agent_path_prediction::AgentPosePredictRequest::EXTERNAL;
            }
        }
        else
        {
            if (!stuck && !door_pass)
            {
                isMode = 1;
            }
            double traj_size = 10, predict_time = 5.0; // TODO: make these values configurable
            for (double i = 1.0; i <= traj_size; ++i)
            {
                predict_srv.request.predict_times.push_back(predict_time *
                                                            (i / traj_size));
            }
            predict_srv.request.type =
                agent_path_prediction::AgentPosePredictRequest::VELOCITY_OBSTACLE;
            if (!backed_off && stuck)
            {
                backed_off = backoff_recovery_.recovery(ang_theta);
                goal_ctrl = false;
            }
        }

        std_srvs::SetBool publish_predicted_markers_srv;
        publish_predicted_markers_srv.request.data =
            publish_predicted_agent_markers_;
        if (!publish_predicted_markers_client_ &&
            publish_predicted_markers_client_.call(publish_predicted_markers_srv))
        {
            ROS_WARN("Failed to call %s service, is agent prediction server running?",
                     publish_makers_srv_name_.c_str());
        }

        if (predict_agents_client_ && predict_agents_client_.call(predict_srv))
        {
            tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

            for (int indx = 0; indx < static_agents_ids.size(); indx++)
            {
                geometry_msgs::Twist empty_vel;
                geometry_msgs::PoseStamped current_hpose;
                current_hpose.header.frame_id = "static";
                current_hpose.pose = agents_[static_agents_ids[indx] - 1];

                PlanStartVelGoalVel plan_start_vel_goal_vel;
                plan_start_vel_goal_vel.plan.push_back(current_hpose);
                plan_start_vel_goal_vel.start_vel = empty_vel;
                plan_start_vel_goal_vel.nominal_vel = 0;
                plan_start_vel_goal_vel.isMode = isMode;
                transformed_agent_plan_vel_map[static_agents_ids[indx]] = plan_start_vel_goal_vel;
            }

            for (auto predicted_agents_poses :
                 predict_srv.response.predicted_agents_poses)
            {

                if (std::find(predict_srv.request.ids.begin(), predict_srv.request.ids.end(), predicted_agents_poses.id) == predict_srv.request.ids.end())
                {
                    continue;
                }

                if (isMode == 2)
                    continue;

                // transform agent plans
                AgentPlanCombined agent_plan_combined;
                auto &transformed_vel = predicted_agents_poses.start_velocity;

                if (!transformAgentPlan(*tf_, robot_pose, *costmap_, global_frame_,
                                        predicted_agents_poses.poses,
                                        agent_plan_combined, transformed_vel,
                                        &tf_agent_plan_to_global))
                {
                    ROS_WARN("Could not transform the agent %ld plan to the frame of the "
                             "controller",
                             predicted_agents_poses.id);
                    continue;
                }

                agent_plan_combined.id = predicted_agents_poses.id;
                transformed_agent_plans.push_back(agent_plan_combined);

                PlanStartVelGoalVel plan_start_vel_goal_vel;
                plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
                plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
                plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agent_nominal_vels[predicted_agents_poses.id - 1]);
                plan_start_vel_goal_vel.isMode = isMode;
                if (agent_plan_combined.plan_after.size() > 0)
                {
                    plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
                }
                transformed_agent_plan_vel_map[agent_plan_combined.id] =
                    plan_start_vel_goal_vel;
            }
        }
        else
        {
            ROS_WARN_THROTTLE(
                THROTTLE_RATE,
                "Failed to call %s service, is agent prediction server running?",
                predict_srv_name_.c_str());
        }
        updateAgentViaPointsContainers(transformed_agent_plan_vel_map,
                                       cfg_.trajectory.global_plan_viapoint_sep);
        break;
    }
    case 2:
    {
        agent_path_prediction::AgentPosePredict predict_srv;
        predict_srv.request.predict_times.push_back(0.0);
        predict_srv.request.predict_times.push_back(5.0);
        predict_srv.request.type =
            agent_path_prediction::AgentPosePredictRequest::VELOCITY_OBSTACLE;

        // setup marker publishing
        std_srvs::SetBool publish_predicted_markers_srv;
        publish_predicted_markers_srv.request.data =
            publish_predicted_agent_markers_;
        if (!publish_predicted_markers_client_ &&
            publish_predicted_markers_client_.call(publish_predicted_markers_srv))
        {
            ROS_WARN("Failed to call %s service, is agent prediction server running?",
                     publish_makers_srv_name_.c_str());
        }

        // call agent prediction server
        if (predict_agents_client_ && predict_agents_client_.call(predict_srv))
        {
            for (auto predicted_agents_poses :
                 predict_srv.response.predicted_agents_poses)
            {
                if (predicted_agents_poses.id == cfg_.approach.approach_id)
                {
                    geometry_msgs::PoseStamped transformed_agent_pose;
                    if (!transformAgentPose(*tf_, global_frame_,
                                            predicted_agents_poses.poses.front(),
                                            transformed_agent_pose))
                    {
                        ROS_WARN(
                            "Could not transform the agent %ld pose to controller frame",
                            predicted_agents_poses.id);
                    }

                    PlanStartVelGoalVel plan_start_vel_goal_vel;
                    plan_start_vel_goal_vel.plan.push_back(transformed_agent_pose);
                    plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agent_nominal_vels[predicted_agents_poses.id - 1]);
                    plan_start_vel_goal_vel.isMode = isMode;
                    transformed_agent_plan_vel_map[predicted_agents_poses.id] =
                        plan_start_vel_goal_vel;

                    // update global plan of the robot
                    // find position in front of the agent
                    tf2::Transform tf_agent_pose, tf_approach_pose[3];
                    geometry_msgs::PoseStamped approach_pose[3];
                    for (int i = 0; i < 3; i++)
                    {
                        tf2::fromMsg(transformed_agent_pose.pose, tf_agent_pose);
                        tf_approach_pose[i].setOrigin(tf2::Vector3(cfg_.approach.approach_dist + (2 - i) * 0.3, 0.0, 0.0));
                        tf2::Quaternion approachQuaternion;
                        approachQuaternion.setEuler(cfg_.approach.approach_angle, 0.0, 0.0);
                        tf_approach_pose[i].setRotation(approachQuaternion);
                        tf_approach_pose[i] = tf_agent_pose * tf_approach_pose[i];
                        tf2::toMsg(tf_approach_pose[i], approach_pose[i].pose);
                        approach_pose[i].header = transformed_agent_pose.header;
                    }

                    // add approach pose to the robot plan, only within reachable distance
                    // auto &plan_goal = transformed_plan.back().pose;
                    // auto &approach_goal = approach_pose[2].pose;
                    tf2::Transform plan_goal, approach_goal;
                    tf2::fromMsg(transformed_plan.back().pose, plan_goal);
                    tf2::fromMsg(approach_pose[2].pose, approach_goal);
                    double lin_dist = std::abs(
                        std::hypot(plan_goal.getOrigin().getX() - approach_goal.getOrigin().getX(),
                                   plan_goal.getOrigin().getY() - approach_goal.getOrigin().getY()));
                    double ang_dist = std::abs(angles::shortest_angular_distance(
                        tf2::impl::getYaw(plan_goal.getRotation()),
                        tf2::impl::getYaw(approach_goal.getRotation())));
                    // ROS_INFO("lin_dist=%.2f, ang_dist=%.2f", lin_dist, ang_dist);
                    tf2::Transform tf_approach_global[3];
                    geometry_msgs::PoseStamped approach_pose_global[3];
                    if (lin_dist > cfg_.approach.approach_dist_tolerance ||
                        ang_dist > cfg_.approach.approach_angle_tolerance)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            transformed_plan.push_back(approach_pose[i]);

                            // get approach poses in to the frame of global plan
                            tf2::Stamped<tf2::Transform> temp;
                            tf2::fromMsg(tf_plan_to_global, temp);
                            tf_approach_global[i] = temp.inverse() * tf_approach_pose[i];

                            tf2::toMsg(tf_approach_global[i], approach_pose_global[i].pose);
                            approach_pose_global[i].header = global_plan_.back().header;
                        }

                        // prune and update global plan
                        auto global_plan_it = global_plan_.begin();
                        double last_dist = std::numeric_limits<double>::infinity();
                        while (global_plan_it != global_plan_.end())
                        {
                            auto &p_pos = (*global_plan_it).pose.position;
                            auto &a_pos = approach_pose_global[0].pose.position;
                            double pa_dist = std::hypot(p_pos.x - a_pos.x, p_pos.y - a_pos.y);
                            if (pa_dist > last_dist)
                            {
                                break;
                            }
                            last_dist = pa_dist;
                            global_plan_it++;
                        }
                        global_plan_.erase(global_plan_it, global_plan_.end());
                        for (int i = 0; i < 3; i++)
                        {
                            global_plan_.push_back(approach_pose_global[i]);
                        }
                        ROS_INFO("Global plan modified for approach behavior");
                    }
                }
            }
        }
        else
        {
            ROS_WARN_THROTTLE(
                THROTTLE_RATE,
                "Failed to call %s service, is agent prediction server running?",
                predict_srv_name_.c_str());
        }
        // TODO: check if plan-map is not empty
        break;
    }
    default:
        break;
    }

    std::string mode;

    if (isMode == 0)
    {
        mode = "DualBand";
    }
    else if (isMode == 1)
    {
        mode = "VelObs";
    }
    else if (isMode == 2)
    {
        mode = "Backoff";
    }
    else if (isMode == 3)
    {
        mode = "Passing through";
    }
    else if (isMode == 4)
    {
        mode = "Approaching Pillar";
    }
    else if (isMode == 5)
    {
        mode = "Approaching Goal";
    }
    else
    {
        mode = "SingleBand";
    }
    // logs+="Mode: " + mode+", ";

    std_msgs::String log_msg;
    log_msg.data = logs;
    log_pub_.publish(log_msg);

    auto agent_time = ros::Time::now() - agent_start_time;

    // update via-points container
    auto via_start_time = ros::Time::now();
    // overwrite/update start of the transformed plan with the actual robot
    // position (allows using the plan as initial trajectory)
    // tf::poseTFToMsg(robot_pose, transformed_plan.front().pose);
    transformed_plan.front() = robot_pose;
    if (!custom_via_points_active_)
        updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
    auto via_time = ros::Time::now() - via_start_time;

    // Now perform the actual
    auto plan_start_time = ros::Time::now();
    // bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
    hateb_local_planner::OptimizationCostArray op_costs;

    double dt_resize = cfg_.trajectory.dt_ref;
    double dt_hyst_resize = cfg_.trajectory.dt_hysteresis;

    // if(isMode==0 or isMode==1){
    //   dt_resize = 0.4;
    //   dt_hyst_resize = 0.0125;
    // }

    bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, isMode);

    if (!success)
    {
        planner_->clearPlanner(); // force reinitialization for next time
        ROS_WARN("hateb_local_planner was not able to obtain a local plan for the current setting.");

        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        message = "hateb_local_planner was not able to obtain a local plan";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
    // op_costs_pub_.publish(op_costs);
    auto plan_time = ros::Time::now() - plan_start_time;

    PlanTrajCombined plan_traj_combined;
    plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
    planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
    plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
    visualization_->publishTrajectory(plan_traj_combined);

    if (cfg_.planning_mode == 1)
    {
        visualization_->publishAgentGlobalPlans(transformed_agent_plans);
        std::vector<AgentPlanTrajCombined> agent_plans_traj_array;
        for (auto &agent_plan_combined : transformed_agent_plans)
        {
            AgentPlanTrajCombined agent_plan_traj_combined;
            agent_plan_traj_combined.id = agent_plan_combined.id;
            agent_plan_traj_combined.plan_before = agent_plan_combined.plan_before;
            planner_->getFullAgentTrajectory(
                agent_plan_traj_combined.id,
                agent_plan_traj_combined.optimized_trajectory);

            agent_plan_traj_combined.plan_after = agent_plan_combined.plan_after;
            agent_plans_traj_array.push_back(agent_plan_traj_combined);
        }
        visualization_->publishAgentTrajectories(agent_plans_traj_array);
    }

    double ttg = std::hypot(transformed_plan.back().pose.position.x - transformed_plan.front().pose.position.x,
                            transformed_plan.back().pose.position.y - transformed_plan.front().pose.position.y) /
                 std::hypot(robot_vel_.linear.x, robot_vel_.linear.y);

    // Undo temporary horizon reduction
    auto hr2_start_time = ros::Time::now();

    auto hr2_time = ros::Time::now() - hr2_start_time;

    // Check feasibility (but within the first few states only)
    auto fsb_start_time = ros::Time::now();
    if (cfg_.robot.is_footprint_dynamic)
    {
        // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }
    bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
    if (!feasible)
    {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        // now we reset everything to start again with the initialization of new trajectories.
        planner_->clearPlanner();
        ROS_WARN("HATebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;

        message = "hateb_local_planner trajectory is not feasible";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
    auto fsb_time = ros::Time::now() - fsb_start_time;

    // Get the velocity command for this sampling interval
    auto vel_start_time = ros::Time::now();
    if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses, dt_resize))
    {
        planner_->clearPlanner();
        ROS_WARN("HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        message = "hateb_local_planner velocity command invalid";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
    saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                     cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_theta,
                     cfg_.robot.max_vel_x_backwards);

    // convert rot-vel to steering angle if desired (carlike robot).
    // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
    // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
    if (cfg_.robot.cmd_angle_instead_rotvel)
    {
        cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_.robot.wheelbase, 0.95 * cfg_.robot.min_turning_radius);
        if (!std::isfinite(cmd_vel.twist.angular.z))
        {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
            last_cmd_ = cmd_vel.twist;
            planner_->clearPlanner();
            ROS_WARN("HATebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
            ++no_infeasible_plans_; // increase number of infeasible solutions in a row
            time_last_infeasible_plan_ = ros::Time::now();

            message = "hateb_local_planner steering angle is not finite";
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
        }
    }
    auto vel_time = ros::Time::now() - vel_start_time;

    // a feasible solution should be found, reset counter
    no_infeasible_plans_ = 0;

    // store last command (for recovery analysis etc.)
    last_cmd_ = cmd_vel.twist;

    // Now visualize everything
    auto viz_start_time = ros::Time::now();
    planner_->visualize();
    visualization_->publishObstacles(obstacles_);
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);
    if (isDistMax && !door_pass)
        visualization_->publishMode(-1);
    else
        visualization_->publishMode(isMode);

    auto viz_time = ros::Time::now() - viz_start_time;
    auto total_time = ros::Time::now() - start_time;

    ROS_DEBUG_STREAM_COND(total_time.toSec() > 0.1, "\tcompute velocity times:\n"
                                                        << "\t\ttotal time                   "
                                                        << std::to_string(total_time.toSec()) << "\n"
                                                        << "\t\tpose get time                "
                                                        << std::to_string(pose_get_time.toSec()) << "\n"
                                                        << "\t\tvel get time                 "
                                                        << std::to_string(vel_get_time.toSec()) << "\n"
                                                        << "\t\tprune time                   "
                                                        << std::to_string(prune_time.toSec()) << "\n"
                                                        << "\t\ttransform time               "
                                                        << std::to_string(transform_time.toSec()) << "\n"
                                                        // << "\t\thorizon setup time           "
                                                        // << std::to_string((hr1_time + hr2_time).toSec()) << "\n"
                                                        << "\t\tother time                   "
                                                        << std::to_string(other_time.toSec()) << "\n"
                                                        << "\t\tcostmap convert time         "
                                                        << std::to_string(cc_time.toSec()) << "\n"
                                                        << "\t\tvia points time              "
                                                        << std::to_string(via_time.toSec()) << "\n"
                                                        << "\t\tagent time                   "
                                                        << std::to_string(agent_time.toSec()) << "\n"
                                                        << "\t\tplanning time                "
                                                        << std::to_string(plan_time.toSec()) << "\n"
                                                        << "\t\tplan feasibility check time  "
                                                        << std::to_string(fsb_time.toSec()) << "\n"
                                                        << "\t\tvelocity extract time        "
                                                        << std::to_string(vel_time.toSec()) << "\n"
                                                        << "\t\tvisualization publish time   "
                                                        << std::to_string(viz_time.toSec()) << "\n=========================");
    return mbf_msgs::ExePathResult::SUCCESS;
}

//! Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hateb_planning_framework = std::make_shared<HATEBPlanningFramework>();

    hateb_planning_framework->planWithSimpleSetup();

    rclcpp::spin(hateb_planning_framework);
    return 0;
}
