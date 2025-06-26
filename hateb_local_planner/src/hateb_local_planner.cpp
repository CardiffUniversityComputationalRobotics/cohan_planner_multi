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

#include <cohan_msgs/msg/state_array.hpp>

#include <visualization.h>

enum AgentState
{
    NO_STATE,
    STATIC,
    MOVING,
    STOPPED,
    BLOCKED
};

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
    bool HATebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::msg::PoseStamped &global_pose, std::vector<geometry_msgs::msg::PoseStamped> &global_plan, double dist_behind_robot);
    uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::TwistStamped &velocity, geometry_msgs::msg::TwistStamped &cmd_vel);

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
    std::string odometry_topic_, query_goal_topic_, solution_path_topic_, world_frame_, control_active_topic_, robot_base_frame_;

    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    geometry_msgs::msg::Twist current_robot_velocity_;

    // configs params
    double pose_prediction_reset_time_ = 0.1;
    bool initialized_, reset_states_, goal_reached_;
    cohan_msgs::msg::StateArray agents_states_; // State of agents

    rclcpp::Time last_position_time_;

    int robot_type_ = 0;
    int is_mode_, change_mode_, stuck_agent_id_;

    bool enable_backoff_ = false;
    std::vector<bool> agent_still_;
    std::vector<int> visible_agent_ids_; // List of visible agents
    bool is_dist_under_threshold_, stuck_;

    double global_plan_prune_distance_ = 4.0;

    nav2_costmap_2d::Costmap2D costmap_model_;
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
                                                         geometry_msgs::msg::TwistStamped &cmd_vel)
{

    if (!initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "hateb_local_planner has not been initialized, please call initialize() before using this planner");
    }

    if (reset_states_)
    {
        for (int i = 0; i < agents_states_.states.size(); i++)
        {
            agents_states_.states[i] = AgentState::NO_STATE;
        }
        reset_states_ = false;
    }
    static uint32_t seq = 0;
    cmd_vel.header.stamp = this->get_clock()->now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    goal_reached_ = false;

    // TODO: evaluate whether the robot is stuck or not
    if ((this->get_clock()->now() - last_position_time_).seconds() >= 2.0)
    { // 0: Robot and 1: Human for type
        if (visible_agent_ids_.size() > 0)
        {
            if (agent_still_[visible_agent_ids_[0] - 1] && is_dist_under_threshold_ && !stuck_)
            {
                if (change_mode_ == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "I am stuck because of an agent, Changing to VelObs mode");
                }
                change_mode_++;
                is_mode_ = 1;
            }
        }
    }

    if (!stuck_)
        stuck_agent_id_ = -1;

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tf_buffer_, last_robot_pose_, global_plan_, global_plan_prune_distance_);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    PlanCombined transformed_plan_combined;
    int goal_idx;
    // TODO: add costmap_ support from nav2 costmap
    geometry_msgs::msg::TransformStamped tf_plan_to_global;
    if (!transformGlobalPlan(*tf_buffer_, global_plan_, robot_pose, *costmap_, global_frame_,
                             max_global_plan_lookahead_dist_,
                             transformed_plan_combined, &goal_idx, &tf_plan_to_global))
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform the global plan to the frame of the controller");
    }
    auto &transformed_plan = transformed_plan_combined.plan_to_optimize;

    // check if global goal is reached
    geometry_msgs::msg::PoseStamped global_goal;
    tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
    double dx = global_goal.pose.position.x - robot_pose_.x();
    double dy = global_goal.pose.position.y - robot_pose_.y();
    double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta());
    if (fabs(std::sqrt(dx * dx + dy * dy)) < xy_goal_tolerance_ && fabs(delta_orient) < yaw_goal_tolerance_ && (!complete_global_plan_ || via_points_.size() == 0) && goal_ctrl)
    {
        goal_reached_ = true;
    }

    if (fabs(std::sqrt(dx * dx + dy * dy)) < 1.0)
    {
        is_mode_ = 5;
    }

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Transformed plan is empty. Cannot determine a local plan.");
    }

    // Get current goal point (last point of the transformed plan)
    robot_goal_.x() = transformed_plan.back().pose.position.x;
    robot_goal_.y() = transformed_plan.back().pose.position.y;
    // Overwrite goal orientation if needed
    if (global_plan_overwrite_orientation_)
    {
        robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
        // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_goal_.theta());
        transformed_plan.back().pose.orientation = tf2::toMsg(q);
    }
    else
    {
        robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    if (transformed_plan.size() == 1) // plan only contains the goal
    {
        transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
    }
    transformed_plan.front() = robot_pose; // update start

    // clear currently existing obstacles
    obstacles_.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    if (costmap_converter_)
        updateObstacleContainerWithCostmapConverter();
    else
        updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    updateObstacleContainerWithCustomObstacles();
    updateObstacleContainerWithInvHumans();

    // update agents
    std::vector<AgentPlanCombined> transformed_agent_plans;
    AgentPlanVelMap transformed_agent_plan_vel_map;
    agents_states_pub_->publish(agents_states_);

    bool found = true;
    if (stuck_agent_id != -1)
    {
        if (!visible_agent_ids_.size() > 0)
        {
            if (visible_agent_ids_[0] != stuck_agent_id)
                found = false;
        }
        else if (visible_agent_ids_.size() == 0)
            found = false;
    }

    if (isDistMax || !found)
    {
        agents_via_points_map_.clear();

        if (!visible_agent_ids_.size() > 0)
        {
            updateAgentViaPointsContainers(transformed_agent_plan_vel_map,
                                           global_plan_viapoint_sep_);
        }
        goal_ctrl = true;

        break;
    }

    if (is_mode_ == 0)
        is_mode_ = -1;

    std::vector<int> static_agents_ids;

    for (int i = 0; i < 2 && i < visible_agent_ids_.size(); i++)
    {
        if ((int)agents_states_.states[visible_agent_ids_[i] - 1] > 1)
        {
            if (is_mode_ == -1)
                is_mode_ = 0;
        }
        else
        {
            static_agents_ids.push_back(visible_agent_ids_[i]);
        }
    }

    tf2::Stamped<tf2::Transform> tf_agent_plan_to_global;

    for (int indx = 0; indx < static_agents_ids.size(); indx++)
    {
        geometry_msgs::msg::Twist empty_vel;
        geometry_msgs::msg::PoseStamped current_hpose;
        current_hpose.header.frame_id = "static";
        current_hpose.pose = agents_[static_agents_ids[indx] - 1];

        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan.push_back(current_hpose);
        plan_start_vel_goal_vel.start_vel = empty_vel;
        plan_start_vel_goal_vel.nominal_vel = 0;
        plan_start_vel_goal_vel.is_mode_ = is_mode_;
        transformed_agent_plan_vel_map[static_agents_ids[indx]] = plan_start_vel_goal_vel;
    }

    // TODO: predicted_agents_poses should contain the agents information
    for (auto predicted_agents_poses :
         predicted_agents_poses)
    {

        // transform agent plans
        AgentPlanCombined agent_plan_combined;
        auto &transformed_vel = predicted_agents_poses.start_velocity;
        if (!transformAgentPlan(*tf_buffer_, robot_pose, *costmap_, global_frame_,
                                predicted_agents_poses.poses,
                                agent_plan_combined, transformed_vel,
                                &tf_agent_plan_to_global))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not transform the agent %ld plan to the frame of the controller",
                        predicted.id);
            continue;
        }

        agent_plan_combined.id = predicted_agents_poses.id;
        transformed_agent_plans.push_back(agent_plan_combined);

        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
        plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
        plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agent_nominal_vels[predicted_agents_poses.id - 1]);
        plan_start_vel_goal_vel.is_mode_ = is_mode_;
        if (!agent_plan_combined.plan_after.size() > 0)
        {
            plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
        }
        transformed_agent_plan_vel_map[agent_plan_combined.id] =
            plan_start_vel_goal_vel;
    }
    updateAgentViaPointsContainers(transformed_agent_plan_vel_map,
                                   global_plan_viapoint_sep_);

    std::string mode;

    if (is_mode_ == 0)
    {
        mode = "DualBand";
    }
    else if (is_mode_ == 1)
    {
        mode = "VelObs";
    }
    else if (is_mode_ == 3)
    {
        mode = "Passing through";
    }
    else if (is_mode_ == 4)
    {
        mode = "Approaching Pillar";
    }
    else if (is_mode_ == 5)
    {
        mode = "Approaching Goal";
    }
    else
    {
        mode = "SingleBand";
    }

    transformed_plan.front() = robot_pose;
    if (!custom_via_points_active_)
        updateViaPointsContainer(transformed_plan, global_plan_viapoint_sep_);

    cohan_msgs::msg::OptimizationCostArray op_costs;
    double dt_resize = dt_ref_;
    double dt_hyst_resize = dt_hysteresis_;

    bool success = planner_->plan(transformed_plan, &robot_vel_, free_goal_vel_, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, is_mode_);

    if (!success)
    {
        planner_->clearPlanner(); // force reinitialization for next time
        RCLCPP_WARN(node_->get_logger(), "hateb_local_planner was not able to obtain a local plan.");
        ++no_infeasible_plans_;
        last_cmd_ = cmd_vel.twist;
    }

    PlanTrajCombined plan_traj_combined;
    plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
    planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
    plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
    visualization_->publishTrajectory(plan_traj_combined);

    if (planning_mode_ == 1)
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

    // Check feasibility (but within the first few states only)
    if (is_footprint_dynamic_)
    {
        // TODO: min and max distance would be the radius of the robot
        // footprint_spec_ = costmap_ros_->getRobotFootprint();
        // costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
        robot_inscribed_radius_ = 0.4;
        robot_circumscribed_radius_ = 0.4;
    }
    bool feasible = planner_->isTrajectoryFeasible(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, feasibility_check_no_poses_);
    if (!feasible)
    {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        // now we reset everything to start again with the initialization of new trajectories.
        planner_->clearPlanner();
        RCLCPP_WARN(node_->get_logger(), "HATebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        last_cmd_ = cmd_vel.twist;
    }

    // Get the velocity command for this sampling interval
    if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, control_look_ahead_poses_, dt_resize))
    {
        planner_->clearPlanner();
        RCLCPP_WARN(node_->get_logger(), "HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        last_cmd_ = cmd_vel.twist;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
    saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                     robot.max_vel_x_, robot.max_vel_y_, max_vel_theta_,
                     max_vel_x_backwards_);

    // convert rot-vel to steering angle if desired (carlike robot).
    // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
    // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
    if (cmd_angle_instead_rotvel_)
    {
        cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, wheelbase_, 0.95 * min_turning_radius_);
        if (!std::isfinite(cmd_vel.twist.angular.z))
        {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
            last_cmd_ = cmd_vel.twist;
            planner_->clearPlanner();
            RCLCPP_WARN(node_->get_logger(), "HATebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
            ++no_infeasible_plans_;
        }
    }

    // a feasible solution should be found, reset counter
    no_infeasible_plans_ = 0;

    // store last command (for recovery analysis etc.)
    last_cmd_ = cmd_vel.twist;

    // Now visualize everything
    planner_->visualize();
    visualization_->publishObstacles(obstacles_);
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);
    if (isDistMax && !door_pass)
        visualization_->publishMode(-1);
    else
        visualization_->publishMode(isMode);
}

bool HATEBPlanningFramework::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
{
    if (global_plan.empty())
        return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
        geometry_msgs::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

        double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
        while (it != global_plan.end())
        {
            double dx = robot.pose.position.x - it->pose.position.x;
            double dy = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
        }
        if (erase_end == global_plan.end())
            return false;

        if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf::TransformException &ex)
    {
        ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
        return false;
    }
    return true;
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
