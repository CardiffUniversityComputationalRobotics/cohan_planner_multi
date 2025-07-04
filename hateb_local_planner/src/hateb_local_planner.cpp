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

// C++ Standard Library
#include <iostream>
#include <vector>
#include <math.h>

// Boost
#include <boost/bind.hpp>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

// ROS2 Geometry and Navigation
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ROS2 TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Navigation / Costmap
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

// Custom / External Messages
#include <cohan_msgs/msg/state_array.hpp>
#include <cohan_msgs/msg/agent_states_prediction.hpp>
#include <esc_move_base_msgs/msg/path2_d.hpp>

// Pedsim Messages
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>

// Local Project Headers
#include <visualization.h>
#include <optimal_planner.h>

#define DEFAULT_AGENT_SEGMENT cohan_msgs::msg::TrackedSegmentType::TORSO

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
    void planningSetup();
    //! Periodic callback to solve the query
    void planningTimerCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg);
    //! Callback for getting the state of the Smf base controller
    void controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg);
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void agentsCallback(const pedsim_msgs::msg::AgentStates::SharedPtr agent_states_msg);
    void agentsPredictionCallback(const cohan_msgs::msg::AgentStatesPrediction::SharedPtr agent_states_msg);
    void globalPlanCallback(const esc_move_base_msgs::msg::Path2D::SharedPtr path_msg);
    void initialize();
    bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped &global_pose, std::vector<geometry_msgs::msg::PoseStamped> &global_plan, double dist_behind_robot);
    uint32_t computeVelocityCommands(geometry_msgs::msg::TwistStamped &cmd_vel);
    bool transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan,
                             const geometry_msgs::msg::PoseStamped &global_pose, const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                             hateb_local_planner::PlanCombined &transformed_plan_combined, int *current_goal_idx, geometry_msgs::msg::TransformStamped *tf_plan_to_global) const;
    void saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards);
    double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan, const geometry_msgs::msg::PoseStamped &local_goal,
                                        int current_goal_idx, const geometry_msgs::msg::TransformStamped &tf_plan_to_global, int moving_average_length = 3) const;
    bool transformAgentPlan(const geometry_msgs::msg::PoseStamped &robot_pose,
                            const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame,
                            const std::vector<cohan_msgs::msg::PoseWith2DCovariance> &agent_plan,
                            hateb_local_planner::AgentPlanCombined &transformed_agent_plan_combined,
                            geometry_msgs::msg::Twist &transformed_agent_twist,
                            tf2::Stamped<tf2::Transform> *tf_agent_plan_to_global) const;
    void updateObstacleContainerWithCostmap();
    void updateAgentViaPointsContainers(const hateb_local_planner::AgentPlanVelMap &transformed_agent_plan_vel_map, double min_separation);
    void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan, double min_separation);

private:
    // ! SUBSCRIBERS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_active_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_sub_;
    rclcpp::Subscription<cohan_msgs::msg::AgentStatesPrediction>::SharedPtr agent_states_prediction_sub_;
    rclcpp::Subscription<esc_move_base_msgs::msg::Path2D>::SharedPtr global_plan_sub_;

    // ! PUBLISHERS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_motion_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr query_goal_pose_rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr query_goal_radius_rviz_pub_;

    // =============================
    // ROS2 TF
    // =============================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform last_robot_pose_;

    // =============================
    // Costmap & Planner Interfaces
    // =============================
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D *costmap_;
    hateb_local_planner::PlannerInterfacePtr planner_;       //!< Instance of the underlying optimal planner class
    hateb_local_planner::TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)

    // =============================
    // Robot Geometry & Limits
    // =============================
    double robot_base_radius_;
    double robot_inscribed_radius_ = 0.4;
    double robot_circumscribed_radius_ = 0.4;

    double max_vel_x_ = 0.4;
    double max_vel_y_ = 0.0;
    double max_vel_theta_ = 1.0;
    double max_vel_x_backwards_ = 0.0;

    double max_trans_vel_, max_rot_vel_;

    std::string robot_base_frame_;
    std::string world_frame_;

    // =============================
    // Goal and State Flags
    // =============================
    bool odom_available_, goal_available_, control_active_;
    bool initialized_, goal_reached_;
    bool is_dist_under_threshold_;
    bool complete_global_plan_ = true;
    bool is_dist_max_ = true;
    bool free_goal_vel_ = false;
    std::string query_goal_topic_;

    // =============================
    // Timing & Prediction
    // =============================
    double timer_period_;
    double omega_chage_time_seperation_ = 1.0;
    rclcpp::Time last_position_time_;
    rclcpp::Time last_omega_sign_change_ = this->now() - rclcpp::Duration::from_seconds(omega_chage_time_seperation_);
    double last_omega_;

    // =============================
    // Global Plan & Local Plan
    // =============================
    std::vector<geometry_msgs::msg::PoseStamped> global_plan_; //!< Store the current global plan
    double max_global_plan_lookahead_dist_ = 4.0;
    double global_plan_prune_distance_ = 5.0;
    double global_plan_viapoint_sep_ = -0.1;
    std::string solution_path_topic_;

    // =============================
    // Goals & Tolerances
    // =============================
    std::vector<double> start_state_, goal_map_frame_, goal_odom_frame_;
    double goal_radius_, xy_goal_tolerance_, yaw_goal_tolerance_;
    hateb_local_planner::PoseSE2 robot_goal_; //!< Store current robot goal

    // =============================
    // Agents
    // =============================
    cohan_msgs::msg::StateArray agents_states_;                           // State of agents
    cohan_msgs::msg::TrackedAgents tracked_agents_, prev_tracked_agents_; // Tracked agents from an external module
    std::vector<geometry_msgs::msg::Pose> agents_;
    std::vector<int> visible_agent_ids_; // List of visible agents
    std::vector<bool> agent_still_;
    std::vector<double> agent_nominal_vels_;      // Nominal velocities of agents based on moving average filter
    std::vector<std::vector<double>> agent_vels_; // List of agent velocities over time
    std::vector<cohan_msgs::msg::AgentStatePrediction> agent_states_prediction_;
    double agent_radius_ = 0.4;
    int stuck_agent_id_; // Stores the agent id who blocked the robot's way during backoff recovery
    double current_agent_dist_;
    std::map<uint64_t, hateb_local_planner::ViaPointContainer> agents_via_points_map_;

    // =============================
    // Odometry and Commands
    // =============================
    std::string odometry_topic_;
    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    geometry_msgs::msg::Twist current_robot_velocity_;
    geometry_msgs::msg::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()

    // =============================
    // Obstacle Handling
    // =============================
    hateb_local_planner::ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
    double costmap_obstacles_behind_robot_dist_ = 1.5;

    // =============================
    // Via Points
    // =============================
    hateb_local_planner::ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization

    // =============================
    // Planner Control and Recovery
    // =============================
    int no_infeasible_plans_;      //!< Store how many times in a row the planner failed to find a feasible plan.
    int control_look_ahead_poses_; //! Index of the pose used to extract the velocity command
    int feasibility_check_no_poses_ = 5;
    double ang_theta_; // Re-orientation angle

    // =============================
    // Footprint
    // =============================
    std::vector<geometry_msgs::msg::Point> footprint_spec_; //!< Store the footprint of the robot

    // =============================
    // Optimization Parameters
    // =============================
    double dt_hysteresis_ = 0.1;
    double dt_ref_ = 0.3;
    double weight_optimaltime_ = 1;

    // =============================
    // Misc
    // =============================
    int is_mode_, change_mode_;
    int num_moving_avg_ = 5;
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
    start_state_.resize(2);
    goal_map_frame_.resize(3);
    goal_odom_frame_.resize(3);

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
    this->declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.1));
    this->declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.75));

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
    xy_goal_tolerance_ = this->get_parameter("xy_goal_tolerance").as_double();
    yaw_goal_tolerance_ = this->get_parameter("yaw_goal_tolerance").as_double();

    start_state_.resize(3);

    goal_radius_ = xy_goal_tolerance_;

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

    global_plan_sub_ = this->create_subscription<esc_move_base_msgs::msg::Path2D>("/esc_move_base_planner/solution_path", 1, std::bind(&HATEBPlanningFramework::globalPlanCallback, this, std::placeholders::_1));

    // costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw", 1, std::bind(&HATEBPlanningFramework::costmapCallback, this, std::placeholders::_1));

    agent_states_sub_ = this->create_subscription<pedsim_msgs::msg::AgentStates>("/pedsim_simulator/simulated_agents", 1, std::bind(&HATEBPlanningFramework::agentsCallback, this, std::placeholders::_1));

    agent_states_prediction_sub_ = this->create_subscription<cohan_msgs::msg::AgentStatesPrediction>("/agents_prediction", 1, std::bind(&HATEBPlanningFramework::agentsPredictionCallback, this, std::placeholders::_1));

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

    // ! Obtaining costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());

    costmap_ = costmap_ros_->getCostmap();

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

    initialize();
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

void HATEBPlanningFramework::globalPlanCallback(const esc_move_base_msgs::msg::Path2D::SharedPtr path_msg)
{

    global_plan_.clear();

    if (!path_msg->waypoints.empty())
    {
        for (geometry_msgs::msg::Pose2D waypoint : path_msg->waypoints)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = waypoint.x;
            pose_stamped.pose.position.y = waypoint.y;

            tf2::Quaternion q;
            q.setRPY(0, 0, waypoint.theta);
            pose_stamped.pose.orientation = tf2::toMsg(q);

            global_plan_.push_back(pose_stamped);
        }
    }
}

//! Control active callback.
/*!
 * Callback for getting the state of the Smf base controller
 */
void HATEBPlanningFramework::controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg)
{
    control_active_ = control_active_msg->data;
}

void HATEBPlanningFramework::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr costmap_msg)
{
    // Resize local copy of costmap
    costmap_->resizeMap(
        costmap_msg->metadata.size_x,
        costmap_msg->metadata.size_y,
        costmap_msg->metadata.resolution,
        costmap_msg->metadata.origin.position.x,
        costmap_msg->metadata.origin.position.y);

    // Copy data
    unsigned char *costmap_data = costmap_->getCharMap();
    std::memcpy(costmap_data, costmap_msg->data.data(), costmap_msg->data.size() * sizeof(unsigned char));
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

void HATEBPlanningFramework::agentsCallback(const pedsim_msgs::msg::AgentStates::SharedPtr agent_states_msg)
{

    cohan_msgs::msg::TrackedAgents converted_tracked_agents;
    converted_tracked_agents.header.stamp = agent_states_msg->header.stamp;
    converted_tracked_agents.header.frame_id = agent_states_msg->header.frame_id;

    for (const auto &agent : agent_states_msg->agent_states)
    {
        cohan_msgs::msg::TrackedAgent tracked_agent;
        tracked_agent.track_id = agent.id;
        tracked_agent.name = agent.type;
        tracked_agent.type = 1;
        tracked_agent.state = cohan_msgs::msg::TrackedAgent::MOVING;

        // Segment conversion
        cohan_msgs::msg::TrackedSegment segment;
        segment.type = 0; // DEFAULT_AGENT_SEGMENT
        segment.pose.pose.position = agent.pose.position;
        segment.pose.pose.orientation = agent.pose.orientation;
        segment.twist.twist.linear = agent.twist.linear;
        segment.twist.twist.angular.z = 0.0;

        tracked_agent.segments.push_back(segment);
        converted_tracked_agents.agents.push_back(tracked_agent);
    }

    tracked_agents_ = converted_tracked_agents;

    std::vector<double> agent_dists;
    std::vector<double> agents_behind;
    std::vector<double> agents_radii;
    geometry_msgs::msg::TransformStamped transformed_stamped;
    std::string base_link = "base_footprint";

    transformed_stamped = tf_buffer_->lookupTransform("map", base_link, tf2::TimePointZero, tf2::durationFromSec(0.5));
    auto xpos = transformed_stamped.transform.translation.x;
    auto ypos = transformed_stamped.transform.translation.y;
    auto ryaw = tf2::getYaw(transformed_stamped.transform.rotation);
    Eigen::Vector2d robot_vec(std::cos(ryaw), std::sin(ryaw));
    std::vector<double> hum_xpos;
    std::vector<double> hum_ypos;

    int itr = 0;

    for (auto &agent : tracked_agents_.agents)
    {
        if (agents_states_.states.size() < tracked_agents_.agents.size())
        {
            agents_states_.states.push_back(AgentState::NO_STATE);
            std::vector<double> h_vels;
            agent_vels_.push_back(h_vels);
            agent_nominal_vels_.push_back(0.0);
            geometry_msgs::msg::Pose h_pose;
            agents_.push_back(h_pose);
        }
        for (auto &segment : agent.segments)
        {
            if (segment.type == DEFAULT_AGENT_SEGMENT)
            {
                agents_[itr] = segment.pose.pose;
                Eigen::Vector2d rh_vec(segment.pose.pose.position.x - xpos, segment.pose.pose.position.y - ypos);

                agents_behind.push_back(rh_vec.dot(robot_vec));
                agent_dists.push_back(rh_vec.norm());

                agent_vels_[itr].push_back(std::hypot(segment.twist.twist.linear.x, segment.twist.twist.linear.y));

                if ((abs(segment.twist.twist.linear.x) + abs(segment.twist.twist.linear.y) + abs(segment.twist.twist.angular.z)) > 0.0001)
                {
                    if (agents_states_.states[itr] != AgentState::BLOCKED)
                    {
                        agents_states_.states[itr] = AgentState::MOVING;
                    }
                }

                auto n = agent_vels_[itr].size();
                float average = 0.0f;
                if (n != 0)
                {
                    average = accumulate(agent_vels_[itr].begin(), agent_vels_[itr].end(), 0.0) / n;
                }
                agent_nominal_vels_[itr] = average;

                if (n == num_moving_avg_)
                    agent_vels_[itr].erase(agent_vels_[itr].begin());
            }
        }
        itr++;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "Number of agents: %zu", agent_vels_.size());

    agent_still_.clear();
    for (int i = 0; i < prev_tracked_agents_.agents.size(); i++)
    {
        for (int j = 0; j < prev_tracked_agents_.agents[i].segments.size(); j++)
        {
            if (prev_tracked_agents_.agents[i].segments[j].type == DEFAULT_AGENT_SEGMENT)
            {
                double hum_move_dist = std::hypot(tracked_agents_.agents[i].segments[j].pose.pose.position.x - prev_tracked_agents_.agents[i].segments[j].pose.pose.position.x,
                                                  tracked_agents_.agents[i].segments[j].pose.pose.position.y - prev_tracked_agents_.agents[i].segments[j].pose.pose.position.y);

                auto tm_x = tracked_agents_.agents[i].segments[j].pose.pose.position.x;
                auto tm_y = tracked_agents_.agents[i].segments[j].pose.pose.position.y;

                hum_xpos.push_back(tm_x);
                hum_ypos.push_back(tm_y);
                auto n_dist = std::hypot(tm_y - ypos, tm_x - xpos);
                if (tracked_agents_.agents[i].track_id == stuck_agent_id_)
                    ang_theta_ = std::atan2((tm_y - ypos) / n_dist, (tm_x - xpos) / n_dist);

                if (hum_move_dist < 0.0001)
                {
                    agent_still_.push_back(true);
                    if (agents_states_.states[i] == AgentState::MOVING)
                    {
                        agents_states_.states[i] = AgentState::STOPPED;
                    }
                }
                else
                {
                    agent_still_.push_back(false);
                }
            }
        }
    }
    prev_tracked_agents_ = tracked_agents_;

    std::vector<std::pair<double, int>> temp_dist_idx;
    visible_agent_ids_.clear();
    is_dist_max_ = true;
    for (int i = 0; i < agent_dists.size(); i++)
    {
        auto dist = agent_dists[i];
        current_agent_dist_ = agent_dists[0];
        if (dist < 10.0 && agents_behind[i] >= 0.0)
        {
            is_dist_max_ = false;
            temp_dist_idx.push_back(std::make_pair(dist, i + 1));
        }
    }

    if (temp_dist_idx.size() > 0)
    {
        std::sort(temp_dist_idx.begin(), temp_dist_idx.end());

        if (agent_dists[temp_dist_idx[0].second - 1] <= 2.5)
        {
            is_dist_under_threshold_ = true;
        }
        else
        {
            is_dist_under_threshold_ = false;
        }
    }

    int n = 1000;
    if (temp_dist_idx.size() >= 5)
        n = 100;
    for (int it = 0; it < temp_dist_idx.size(); it++)
    {
        // Ray Tracing
        double tm_x = tracked_agents_.agents[temp_dist_idx[it].second - 1].segments[0].pose.pose.position.x;
        double tm_y = tracked_agents_.agents[temp_dist_idx[it].second - 1].segments[0].pose.pose.position.y;
        auto Dx = (tm_x - xpos) / n;
        auto Dy = (tm_y - ypos) / n;

        // Checking using raytracing
        bool cell_collision = false;
        double rob_x = xpos;
        double rob_y = ypos;

        for (int j = 0; j < n; j++)
        {
            unsigned int mx;
            unsigned int my;

            double check_rad;
            if ((int)tracked_agents_.agents[temp_dist_idx[it].second - 1].type == 1)
                check_rad = agent_radius_ + 0.1;
            else
                check_rad = robot_base_radius_ + 0.1;

            if (sqrt((rob_x - tm_x) * (rob_x - tm_x) + (rob_y - tm_y) * (rob_y - tm_y)) <= check_rad)
                break;
            if (costmap_->worldToMap(rob_x, rob_y, mx, my))
            {
                auto cellcost = costmap_->getCost(mx, my);
                if ((int)cellcost > 200 && (int)cellcost < 255)
                {
                    cell_collision = true;
                    break;
                }
                rob_x += Dx;
                rob_y += Dy;
            }
        }
        int hum_id = temp_dist_idx[it].second;

        if (!cell_collision)
        {
            visible_agent_ids_.push_back(hum_id);
            if ((int)tracked_agents_.agents[hum_id - 1].type == 1)
                agents_radii.push_back(agent_radius_);
            else
                agents_radii.push_back(robot_base_radius_);

            if (agents_states_.states[hum_id - 1] == AgentState::NO_STATE)
            {
                agents_states_.states[hum_id - 1] = AgentState::STATIC;
            }
        }
    }

    // Safety step for agents if agent_layers is not added in local costmap
    // Adds a temporary costmap around the agents to let planner plan safe trajectories

    for (int i = 0; i < visible_agent_ids_.size() && i < hum_xpos.size(); i++)
    {
        geometry_msgs::msg::Point v1, v2, v3, v4;
        auto idx = visible_agent_ids_[i] - 1;
        auto agent_radius = agents_radii[idx];
        v1.x = hum_xpos[idx] - agent_radius, v1.y = hum_ypos[idx] - agent_radius, v1.z = 0.0;
        v2.x = hum_xpos[idx] - agent_radius, v2.y = hum_ypos[idx] + agent_radius, v2.z = 0.0;
        v3.x = hum_xpos[idx] + agent_radius, v3.y = hum_ypos[idx] + agent_radius, v3.z = 0.0;
        v4.x = hum_xpos[idx] + agent_radius, v4.y = hum_ypos[idx] - agent_radius, v4.z = 0.0;

        std::vector<geometry_msgs::msg::Point> agent_pos_costmap;

        agent_pos_costmap.push_back(v1);
        agent_pos_costmap.push_back(v2);
        agent_pos_costmap.push_back(v3);
        agent_pos_costmap.push_back(v4);

        // if(!agent_prev_pos_costmap.empty()){
        //   costmap_->setConvexPolygonCost(agent_prev_pos_costmap[idx+1], 0.0);
        // }
        // agent_prev_pos_costmap[idx+1] = agent_pos_costmap;

        bool set_success = false;
        set_success = costmap_->setConvexPolygonCost(agent_pos_costmap, 255.0);
    }
}

void HATEBPlanningFramework::agentsPredictionCallback(const cohan_msgs::msg::AgentStatesPrediction::SharedPtr agent_states_msg)
{
    agent_states_prediction_.clear();
    agent_states_prediction_ = agent_states_msg->agent_states_prediction;
    // rclcpp::Time init_msg_time(agent_states_prediction_[0].agent_state.header.stamp);
    // for (auto &agent : agent_states_prediction_)
    // {
    //     for (int i = 0; i < agent.predicted_poses.size(); i++)
    //     {
    //         rclcpp::Time current_pose_time(agent.predicted_poses[i].header.stamp);
    //         rclcpp::Duration delta = current_pose_time - init_msg_time;

    //         builtin_interfaces::msg::Time new_stamp;
    //         new_stamp.sec = static_cast<int32_t>(delta.seconds());
    //         new_stamp.nanosec = static_cast<uint32_t>(delta.nanoseconds() % 1000000000L);

    //         agent.predicted_poses[i].header.stamp = new_stamp;
    //     }
    // }
}

void HATEBPlanningFramework::initialize()
{

    // reserve some memory for obstacles
    obstacles_.reserve(500);

    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    // create visualization instance

    hateb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<hateb_local_planner::CircularRobotFootprint>(robot_base_radius_);
    hateb_local_planner::CircularRobotFootprintPtr agent_model = boost::make_shared<hateb_local_planner::CircularRobotFootprint>(agent_radius_);

    planner_ = hateb_local_planner::PlannerInterfacePtr(new hateb_local_planner::TebOptimalPlanner(&obstacles_, robot_model, visualization_, &via_points_, agent_model, &agents_via_points_map_));
    planner_->local_weight_optimaltime_ = weight_optimaltime_;

    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

    last_omega_sign_change_ = this->now() - rclcpp::Duration::from_seconds(omega_chage_time_seperation_);

    last_omega_ = 0.0;
    is_dist_under_threshold_ = false;
    is_dist_max_ = true;
    change_mode_ = 0;
    is_mode_ = 0;
    agent_still_.clear();
    agents_states_.states.clear();
    stuck_agent_id_ = -1;

    // set initialized flag
    initialized_ = true;

    RCLCPP_DEBUG(this->get_logger(), "hateb_local_planner plugin initialized.");
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void HATEBPlanningFramework::planningSetup()
{

    // ======================================
    hateb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<hateb_local_planner::CircularRobotFootprint>(0.4);
    hateb_local_planner::CircularRobotFootprintPtr agent_model = boost::make_shared<hateb_local_planner::CircularRobotFootprint>(0.4);

    planner_ = hateb_local_planner::PlannerInterfacePtr(new hateb_local_planner::TebOptimalPlanner(&obstacles_, robot_model, visualization_, &via_points_, agent_model, &agents_via_points_map_));

    // ++++++++++++++++++++++++++++++++++++++

    rclcpp::Rate loop_rate(1.0 / (timer_period_)); // 10 hz
    // goal_available_ = true;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        if (goal_available_)
            RCLCPP_INFO(this->get_logger(), "goal available");

        HATEBPlanningFramework::planningTimerCallback();
        loop_rate.sleep();
        RCLCPP_WARN(this->get_logger(), "spinning");
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
        // computeVelocityCommands()
    }
}

uint32_t HATEBPlanningFramework::computeVelocityCommands(geometry_msgs::msg::TwistStamped &cmd_vel)
{

    if (!initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "hateb_local_planner has not been initialized, please call initialize() before using this planner");
    }

    static uint32_t seq = 0;
    cmd_vel.header.stamp = this->get_clock()->now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    goal_reached_ = false;

    // TODO: evaluate whether the robot is stuck or not
    // if ((this->get_clock()->now() - last_position_time_).seconds() >= 2.0)
    // { // 0: Robot and 1: Human for type
    //     if (visible_agent_ids_.size() > 0)
    //     {
    //         if (agent_still_[visible_agent_ids_[0] - 1] && is_dist_under_threshold_)
    //         {
    //             if (change_mode_ == 0)
    //             {
    //                 RCLCPP_INFO(this->get_logger(), "I am stuck because of an agent, Changing to VelObs mode");
    //             }
    //             change_mode_++;
    //             is_mode_ = 1;
    //         }
    //     }
    // }

    geometry_msgs::msg::PoseStamped robot_pose;

    robot_pose.header.frame_id = world_frame_;
    robot_pose.header.stamp = this->now();

    robot_pose.pose.position.x = last_robot_pose_.getOrigin().x();
    robot_pose.pose.position.y = last_robot_pose_.getOrigin().y();
    robot_pose.pose.position.z = last_robot_pose_.getOrigin().z();

    tf2::Quaternion quat = last_robot_pose_.getRotation();
    robot_pose.pose.orientation = tf2::toMsg(quat);

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(robot_pose, global_plan_, global_plan_prune_distance_);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    hateb_local_planner::PlanCombined transformed_plan_combined;
    int goal_idx;
    // TODO: add costmap_ support from nav2 costmap
    geometry_msgs::msg::TransformStamped tf_plan_to_global;
    if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, world_frame_,
                             max_global_plan_lookahead_dist_,
                             transformed_plan_combined, &goal_idx, &tf_plan_to_global))
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform the global plan to the frame of the controller");
    }
    auto &transformed_plan = transformed_plan_combined.plan_to_optimize;

    // check if global goal is reached
    geometry_msgs::msg::PoseStamped global_goal;
    double useless_roll, useless_pitch, yaw;
    tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
    double dx = global_goal.pose.position.x - last_robot_pose_.getOrigin().x();
    double dy = global_goal.pose.position.y - last_robot_pose_.getOrigin().y();
    tf2::Matrix3x3(last_robot_pose_.getRotation()).getRPY(useless_roll, useless_pitch, yaw);
    double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - yaw);
    if (fabs(std::sqrt(dx * dx + dy * dy)) < xy_goal_tolerance_ && fabs(delta_orient) < yaw_goal_tolerance_ && via_points_.size() == 0)
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
    // Overwrite goal orientation
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    transformed_plan.back().pose.orientation = tf2::toMsg(q);

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    if (transformed_plan.size() == 1) // plan only contains the goal
    {
        transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
    }
    transformed_plan.front() = robot_pose; // update start

    // clear currently existing obstacles
    obstacles_.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    // updateObstacleContainerWithCustomObstacles();
    // updateObstacleContainerWithInvHumans();

    // update agents
    std::vector<hateb_local_planner::AgentPlanCombined> transformed_agent_plans;
    hateb_local_planner::AgentPlanVelMap transformed_agent_plan_vel_map;

    bool found = true;

    if (is_dist_max_ || !found)
    {
        agents_via_points_map_.clear();

        if (!visible_agent_ids_.size() > 0)
        {
            updateAgentViaPointsContainers(transformed_agent_plan_vel_map,
                                           global_plan_viapoint_sep_);
        }
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

        hateb_local_planner::PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan.push_back(current_hpose);
        plan_start_vel_goal_vel.start_vel = empty_vel;
        plan_start_vel_goal_vel.nominal_vel = 0;
        plan_start_vel_goal_vel.is_mode_ = is_mode_;
        transformed_agent_plan_vel_map[static_agents_ids[indx]] = plan_start_vel_goal_vel;
    }

    // TODO: predicted_agents_poses should contain the agents information
    for (auto predicted_agents_poses :
         agent_states_prediction_)
    {

        // transform agent plans
        hateb_local_planner::AgentPlanCombined agent_plan_combined;
        geometry_msgs::msg::Twist &transformed_vel = predicted_agents_poses.agent_state.twist;
        if (!transformAgentPlan(robot_pose, *costmap_, world_frame_,
                                predicted_agents_poses.predicted_poses,
                                agent_plan_combined, transformed_vel,
                                &tf_agent_plan_to_global))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not transform the agent %ld plan to the frame of the controller",
                        predicted_agents_poses.agent_state.id);
            continue;
        }

        agent_plan_combined.id = predicted_agents_poses.agent_state.id;
        transformed_agent_plans.push_back(agent_plan_combined);

        hateb_local_planner::PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan = agent_plan_combined.plan_to_optimize;
        plan_start_vel_goal_vel.start_vel = transformed_vel;
        plan_start_vel_goal_vel.nominal_vel = std::max(0.3, agent_nominal_vels_[predicted_agents_poses.agent_state.id - 1]);
        plan_start_vel_goal_vel.is_mode_ = is_mode_;
        if (!agent_plan_combined.plan_after.size() > 0)
        {
            plan_start_vel_goal_vel.goal_vel = transformed_vel;
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
    updateViaPointsContainer(transformed_plan, global_plan_viapoint_sep_);

    cohan_msgs::msg::OptimizationCostArray op_costs;
    double dt_resize = dt_ref_;
    double dt_hyst_resize = dt_hysteresis_;

    bool success = planner_->plan(transformed_plan, &current_robot_velocity_, free_goal_vel_, &transformed_agent_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize, is_mode_);

    if (!success)
    {
        planner_->clearPlanner(); // force reinitialization for next time
        RCLCPP_WARN(this->get_logger(), "hateb_local_planner was not able to obtain a local plan.");
        ++no_infeasible_plans_;
        last_cmd_ = cmd_vel.twist;
    }

    hateb_local_planner::PlanTrajCombined plan_traj_combined;
    plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
    planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
    plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
    visualization_->publishTrajectory(plan_traj_combined);

    visualization_->publishAgentGlobalPlans(transformed_agent_plans);
    std::vector<hateb_local_planner::AgentPlanTrajCombined> agent_plans_traj_array;
    for (auto &agent_plan_combined : transformed_agent_plans)
    {
        hateb_local_planner::AgentPlanTrajCombined agent_plan_traj_combined;
        agent_plan_traj_combined.id = agent_plan_combined.id;
        agent_plan_traj_combined.plan_before = agent_plan_combined.plan_before;
        planner_->getFullAgentTrajectory(
            agent_plan_traj_combined.id,
            agent_plan_traj_combined.optimized_trajectory);

        agent_plan_traj_combined.plan_after = agent_plan_combined.plan_after;
        agent_plans_traj_array.push_back(agent_plan_traj_combined);
    }
    visualization_->publishAgentTrajectories(agent_plans_traj_array);

    double ttg = std::hypot(transformed_plan.back().pose.position.x - transformed_plan.front().pose.position.x,
                            transformed_plan.back().pose.position.y - transformed_plan.front().pose.position.y) /
                 std::hypot(current_robot_velocity_.linear.x, current_robot_velocity_.linear.y);

    footprint_spec_ = costmap_ros_->getRobotFootprint();
    bool feasible = planner_->isTrajectoryFeasible(costmap_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, feasibility_check_no_poses_);
    if (!feasible)
    {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        // now we reset everything to start again with the initialization of new trajectories.
        planner_->clearPlanner();
        RCLCPP_WARN(this->get_logger(), "HATebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        last_cmd_ = cmd_vel.twist;
    }

    // Get the velocity command for this sampling interval
    if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, control_look_ahead_poses_, dt_resize))
    {
        planner_->clearPlanner();
        RCLCPP_WARN(this->get_logger(), "HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        last_cmd_ = cmd_vel.twist;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
    saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                     max_vel_x_, max_vel_y_, max_vel_theta_,
                     max_vel_x_backwards_);

    // a feasible solution should be found, reset counter
    no_infeasible_plans_ = 0;

    // store last command (for recovery analysis etc.)
    last_cmd_ = cmd_vel.twist;

    // Now visualize everything
    planner_->visualize();
    visualization_->publishObstacles(obstacles_);
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);
    if (is_dist_max_)
        visualization_->publishMode(-1);
    else
        visualization_->publishMode(is_mode_);
}

bool HATEBPlanningFramework::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped &global_pose, std::vector<geometry_msgs::msg::PoseStamped> &global_plan, double dist_behind_robot)
{
    if (global_plan.empty())
        return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_buffer_->lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, rclcpp::Time(0));

        geometry_msgs::msg::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

        double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
        std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
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
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Cannot prune path since no transform is available: %s", ex.what());
        return false;
    }
    return true;
}

bool HATEBPlanningFramework::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan,
                                                 const geometry_msgs::msg::PoseStamped &global_pose, const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                                                 hateb_local_planner::PlanCombined &transformed_plan_combined, int *current_goal_idx, geometry_msgs::msg::TransformStamped *tf_plan_to_global) const
{
    // this method is a slightly modified version of base_local_planner/goal_functions.h

    const geometry_msgs::msg::PoseStamped &plan_pose = global_plan[0];

    transformed_plan_combined.plan_to_optimize.clear();

    try
    {
        if (global_plan.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received plan with zero length");
            *current_goal_idx = 0;
            return false;
        }

        // get plan_to_global_transform from plan frame to global_frame
        geometry_msgs::msg::TransformStamped plan_to_global_transform = tf_buffer_->lookupTransform(global_frame,
                                                                                                    plan_pose.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose = tf_buffer_->transform(global_pose, plan_pose.header.frame_id, tf2::durationFromSec(0.05));

        // we'll discard points on the plan that are outside the local costmap
        double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                         costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
                                2.0;
        // dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
        // located on the border of the local costmap
        dist_threshold *= 0.9;

        int i = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist = 1e10;

        geometry_msgs::msg::PoseStamped newer_pose;
        // we need to loop to a point on the plan that is within a certain distance of the robot
        for (int j = 0; j < (int)global_plan.size(); ++j)
        {
            double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
            double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (new_sq_dist > sq_dist_threshold)
                break; // force stop if we have reached the costmap border

            if (new_sq_dist < sq_dist) // find closest distance
            {
                sq_dist = new_sq_dist;
                i = j;
            }

            const geometry_msgs::msg::PoseStamped &pose = global_plan[i];
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            transformed_plan_combined.plan_before.push_back(newer_pose);
        }
        double plan_length = 0; // check cumulative Euclidean distance along the plan

        // now we'll transform until points are outside of our distance threshold
        while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
        {
            const geometry_msgs::msg::PoseStamped &pose = global_plan[i];
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            transformed_plan_combined.plan_to_optimize.push_back(newer_pose);

            double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;

            // caclulate distance to previous pose
            if (i > 0 && max_plan_length > 0)
                plan_length += hateb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

            ++i;
        }

        if (current_goal_idx)
            *current_goal_idx = i - 1; // minus 1, since i was increased once before leaving the loop

        while (i < global_plan.size())
        {
            const geometry_msgs::msg::PoseStamped &pose = global_plan[i];
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);
            transformed_plan_combined.plan_after.push_back(newer_pose);
            ++i;
        }

        // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
        // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
        if (transformed_plan_combined.plan_after.empty())
        {
            tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

            transformed_plan_combined.plan_after.push_back(newer_pose);

            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx)
                *current_goal_idx = int(global_plan.size()) - 1;
        }
        else
        {
            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx)
                *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
        }

        // Return the transformation from the global plan to the global planning frame if desired
        if (tf_plan_to_global)
            *tf_plan_to_global = plan_to_global_transform;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "No Transform available Error: %s\n", ex.what());
        return false;
    }

    return true;
}

void HATEBPlanningFramework::saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards)
{
    // Limit translational velocity for forward driving
    if (vx > max_vel_x)
        vx = max_vel_x;

    // limit strafing velocity
    if (vy > max_vel_y)
        vy = max_vel_y;
    else if (vy < -max_vel_y)
        vy = -max_vel_y;

    // Limit angular velocity
    if (omega > max_vel_theta)
        omega = max_vel_theta;
    else if (omega < -max_vel_theta)
        omega = -max_vel_theta;

    // Limit backwards velocity
    if (max_vel_x_backwards <= 0)
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "HATebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
    }
    else if (vx < -max_vel_x_backwards)
        vx = -max_vel_x_backwards;

    // slow change of direction in angular velocity
    double min_vel_theta = 0.02;

    if (std::signbit(omega) != std::signbit(last_omega_))
    {
        // signs are changed
        auto now = this->now();
        if ((now - last_omega_sign_change_).seconds() <
            omega_chage_time_seperation_)
        {
            // do not allow sign change
            omega = std::copysign(min_vel_theta, omega);
        }
        last_omega_sign_change_ = now;
        last_omega_ = omega;
    }
}

double HATEBPlanningFramework::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped> &global_plan, const geometry_msgs::msg::PoseStamped &local_goal,
                                                            int current_goal_idx, const geometry_msgs::msg::TransformStamped &tf_plan_to_global, int moving_average_length) const
{
    int n = (int)global_plan.size();

    // check if we are near the global goal already
    if (current_goal_idx > n - moving_average_length - 2)
    {
        if (current_goal_idx >= n - 1) // we've exactly reached the goal
        {
            return tf2::getYaw(local_goal.pose.orientation);
        }
        else
        {
            tf2::Quaternion global_orientation;
            tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
            tf2::Quaternion rotation;
            tf2::fromMsg(tf_plan_to_global.transform.rotation, rotation);
            // TODO(roesmann): avoid conversion to tf2::Quaternion
            return tf2::getYaw(rotation * global_orientation);
        }
    }

    // reduce number of poses taken into account if the desired number of poses is not available
    moving_average_length = std::min(moving_average_length, n - current_goal_idx - 1); // maybe redundant, since we have checked the vicinity of the goal before

    std::vector<double> candidates;
    geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
    geometry_msgs::msg::PoseStamped tf_pose_kp1;

    int range_end = current_goal_idx + moving_average_length;
    for (int i = current_goal_idx; i < range_end; ++i)
    {
        // Transform pose of the global plan to the planning frame
        tf2::doTransform(global_plan.at(i + 1), tf_pose_kp1, tf_plan_to_global);

        // calculate yaw angle
        candidates.push_back(std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
                                        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x));

        if (i < range_end - 1)
            tf_pose_k = tf_pose_kp1;
    }
    return hateb_local_planner::average_angles(candidates);
}

bool HATEBPlanningFramework::transformAgentPlan(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame,
    const std::vector<cohan_msgs::msg::PoseWith2DCovariance> &agent_plan,
    hateb_local_planner::AgentPlanCombined &transformed_agent_plan_combined,
    geometry_msgs::msg::Twist &transformed_agent_twist,
    tf2::Stamped<tf2::Transform> *tf_agent_plan_to_global) const
{
    try
    {
        if (agent_plan.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received agent plan with zero length");
            return false;
        }

        // get agent_plan_to_global_transform from plan frame to global_frame
        geometry_msgs::msg::TransformStamped agent_plan_to_global_transform;
        agent_plan_to_global_transform = tf_buffer_->lookupTransform(global_frame, agent_plan.front().header.frame_id,
                                                                     tf2::TimePointZero, tf2::durationFromSec(0.5));
        tf2::Stamped<tf2::Transform> agent_plan_to_global_transform_;
        tf2::fromMsg(agent_plan_to_global_transform, agent_plan_to_global_transform_);

        // transform the full plan to local planning frame
        std::vector<geometry_msgs::msg::PoseStamped> transformed_agent_plan;
        tf2::Stamped<tf2::Transform> tf_pose_stamped;
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::Transform tf_pose;
        auto agent_start_pose = agent_plan[0];
        for (auto &agent_pose : agent_plan)
        {
            if (is_mode_ >= 1 && is_mode_ < 3)
            {
                if (std::hypot(agent_pose.pose.position.x - agent_start_pose.pose.position.x,
                               agent_pose.pose.position.y - agent_start_pose.pose.position.y) > (agent_radius_))
                {
                    unsigned int mx, my;
                    if (costmap_->worldToMap(agent_pose.pose.position.x, agent_pose.pose.position.y, mx, my))
                    {
                        if (costmap_->getCost(mx, my) >= 254)
                            break;
                    }
                }
            }
            tf2::fromMsg(agent_pose.pose, tf_pose);
            tf_pose_stamped.setData(agent_plan_to_global_transform_ * tf_pose);
            tf_pose_stamped.stamp_ = agent_plan_to_global_transform_.stamp_;
            tf_pose_stamped.frame_id_ = global_frame;

            transformed_pose.header.stamp = agent_plan_to_global_transform.header.stamp;
            transformed_pose.header.frame_id = tf_pose_stamped.frame_id_;
            transformed_pose.pose.position.x = tf_pose_stamped.getOrigin().x();
            transformed_pose.pose.position.y = tf_pose_stamped.getOrigin().y();
            transformed_pose.pose.position.z = tf_pose_stamped.getOrigin().z();
            transformed_pose.pose.orientation = tf2::toMsg(tf_pose_stamped.getRotation());

            transformed_agent_plan.push_back(transformed_pose);
        }

        // transform agent twist to local planning frame
        geometry_msgs::msg::Twist agent_to_global_twist;
        agent_to_global_twist = transformed_agent_twist;
        transformed_agent_twist.linear.x -= agent_to_global_twist.linear.x;
        transformed_agent_twist.linear.y -= agent_to_global_twist.linear.y;
        transformed_agent_twist.angular.z -= agent_to_global_twist.angular.z;

        double dist_threshold =
            std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
            2.0;
        dist_threshold *= 0.9;

        double sq_dist_threshold = dist_threshold * dist_threshold;
        double x_diff, y_diff, sq_dist;

        // get first point of agent plan within threshold distance from robot
        int start_index = transformed_agent_plan.size(), end_index = 0;
        for (int i = 0; i < transformed_agent_plan.size(); i++)
        {
            x_diff = robot_pose.pose.position.x -
                     transformed_agent_plan[i].pose.position.x;
            y_diff = robot_pose.pose.position.y -
                     transformed_agent_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (sq_dist < sq_dist_threshold)
            {
                start_index = i;
                break;
            }
        }
        // now get last point of agent plan withing threshold distance from robot
        for (int i = (transformed_agent_plan.size() - 1); i >= 0; i--)
        {
            x_diff = robot_pose.pose.position.x -
                     transformed_agent_plan[i].pose.position.x;
            y_diff = robot_pose.pose.position.y -
                     transformed_agent_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (sq_dist < sq_dist_threshold)
            {
                end_index = i;
                break;
            }
        }

        transformed_agent_plan_combined.plan_before.clear();
        transformed_agent_plan_combined.plan_to_optimize.clear();
        transformed_agent_plan_combined.plan_after.clear();
        for (int i = 0; i < transformed_agent_plan.size(); i++)
        {
            if (i < start_index)
            {
                transformed_agent_plan_combined.plan_before.push_back(
                    transformed_agent_plan[i]);
            }
            else if (i >= start_index && i <= end_index)
            {
                transformed_agent_plan_combined.plan_to_optimize.push_back(
                    transformed_agent_plan[i]);
            }
            else if (i > end_index)
            {
                transformed_agent_plan_combined.plan_after.push_back(
                    transformed_agent_plan[i]);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Transform agent plan indexing error");
            }
        }

        if (tf_agent_plan_to_global)
            *tf_agent_plan_to_global = agent_plan_to_global_transform_;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return false;
    }

    return true;
}

void HATEBPlanningFramework::updateObstacleContainerWithCostmap()
{
    // Add costmap obstacles if desired

    tf2::Vector3 forward = last_robot_pose_.getBasis() * tf2::Vector3(1, 0, 0);
    Eigen::Vector2d robot_orient(forward.x(), forward.y());

    Eigen::Vector2d robot_position(last_robot_pose_.getOrigin().x(),
                                   last_robot_pose_.getOrigin().y());

    for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
    {
        for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
        {
            if (costmap_->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                Eigen::Vector2d obs;
                costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                // check if obstacle is interesting (e.g. not far behind the robot)
                Eigen::Vector2d obs_dir = obs - robot_position;
                if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > costmap_obstacles_behind_robot_dist_)
                    continue;

                obstacles_.push_back(hateb_local_planner::ObstaclePtr(new hateb_local_planner::PointObstacle(obs)));
            }
        }
    }
}

void HATEBPlanningFramework::updateAgentViaPointsContainers(
    const hateb_local_planner::AgentPlanVelMap &transformed_agent_plan_vel_map,
    double min_separation)
{
    if (min_separation < 0)
        return;

    // reset via-points for known agents, create via-points for new agents
    for (auto &transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map)
    {
        auto &agent_id = transformed_agent_plan_vel_kv.first;
        auto &initial_agent_plan = transformed_agent_plan_vel_kv.second.plan;
        if (initial_agent_plan.size() == 1)
        {
            if (initial_agent_plan[0].header.frame_id == "static")
            {
                return;
            }
        }

        if (agents_via_points_map_.find(agent_id) != agents_via_points_map_.end())
        {
            agents_via_points_map_[agent_id].clear();
        }
        else
        {
            agents_via_points_map_[agent_id] = hateb_local_planner::ViaPointContainer();
        }
    }

    // remove agent via-points for vanished agents
    auto itr = agents_via_points_map_.begin();
    while (itr != agents_via_points_map_.end())
    {
        if (transformed_agent_plan_vel_map.count(itr->first) == 0)
        {
            itr = agents_via_points_map_.erase(itr);
        }
        else
            ++itr;
    }

    std::size_t prev_idx;
    for (auto &transformed_agent_plan_vel_kv : transformed_agent_plan_vel_map)
    {
        prev_idx = 0;
        auto &agent_id = transformed_agent_plan_vel_kv.first;
        auto &transformed_agent_plan = transformed_agent_plan_vel_kv.second.plan;
        for (std::size_t i = 1; i < transformed_agent_plan.size(); ++i)
        {
            if (hateb_local_planner::distance_points2d(transformed_agent_plan[prev_idx].pose.position, transformed_agent_plan[i].pose.position) < min_separation)
                continue;
            agents_via_points_map_[agent_id].push_back(Eigen::Vector2d(transformed_agent_plan[i].pose.position.x, transformed_agent_plan[i].pose.position.y));

            prev_idx = i;
        }
    }
}

void HATEBPlanningFramework::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped> &transformed_plan, double min_separation)
{
    via_points_.clear();

    if (min_separation <= 0)
        return;

    std::size_t prev_idx = 0;
    for (std::size_t i = 1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
    {
        // check separation to the previous via-point inserted
        if (hateb_local_planner::distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation)
            continue;

        // add via-point
        via_points_.push_back(Eigen::Vector2d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y));
        prev_idx = i;
    }
}

//! Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hateb_planning_framework = std::make_shared<HATEBPlanningFramework>();

    hateb_planning_framework->planningSetup();

    rclcpp::spin(hateb_planning_framework);
    return 0;
}
