/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
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
 * Author: Christoph Rösmann
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#define THROTTLE_RATE 1.0 // seconds

#include <rclcpp/rclcpp.hpp>

#include <optimal_planner.h>
#include <map>
#include <memory>
#include <limits>
#include <fstream>
#include <iostream>

namespace hateb_local_planner
{

  // ============== Implementation ===================

  TebOptimalPlanner::TebOptimalPlanner() : obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                           robot_model_(new PointRobotFootprint()), agent_model_(new CircularRobotFootprint()), initialized_(false), optimized_(false)
  {
  }

  TebOptimalPlanner::TebOptimalPlanner(ObstContainer *obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer *via_points, CircularRobotFootprintPtr agent_model,
                                       const std::map<uint64_t, ViaPointContainer> *agents_via_points_map)
  {
    initialize(obstacles, robot_model, visual, via_points, agent_model, agents_via_points_map);
  }

  TebOptimalPlanner::~TebOptimalPlanner()
  {
    clearGraph();
    // free dynamically allocated memory
    // if (optimizer_)
    //  g2o::Factory::destroy();
    // g2o::OptimizationAlgorithmFactory::destroy();
    // g2o::HyperGraphActionLibrary::destroy();
  }

  void TebOptimalPlanner::initialize(ObstContainer *obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer *via_points, CircularRobotFootprintPtr agent_model, const std::map<uint64_t, ViaPointContainer> *agents_via_points_map)
  {
    // init optimizer (set solver and block ordering settings)
    optimizer_ = initOptimizer();

    // cfg_ = &cfg;
    obstacles_ = obstacles;
    robot_model_ = robot_model;
    agent_model_ = agent_model;
    via_points_ = via_points;
    agents_via_points_map_ = agents_via_points_map;
    cost_ = HUGE_VAL;
    prefer_rotdir_ = RotType::none;
    setVisualization(visual);

    vel_start_.first = true;
    vel_start_.second.linear.x = 0;
    vel_start_.second.linear.y = 0;
    vel_start_.second.angular.z = 0;

    vel_goal_.first = true;
    vel_goal_.second.linear.x = 0;
    vel_goal_.second.linear.y = 0;
    vel_goal_.second.angular.z = 0;

    robot_radius_ = robot_model_->getCircumscribedRadius();
    agent_radius_ = agent_model_->getCircumscribedRadius();

    initialized_ = true;
    is_mode_ = 0;
  }

  void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
  {
    visualization_ = visualization;
  }

  void TebOptimalPlanner::visualize()
  {
    if (!visualization_)
      return;

    double fp_size = 0.0;
    fp_size = teb_.sizePoses();

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;
      fp_size = teb_.sizePoses() > agent_teb.sizePoses() ? teb_.sizePoses() : agent_teb.sizePoses();
    }
    visualization_->publishLocalPlanAndPoses(teb_, *robot_model_, fp_size);
    visualization_->publishAgentLocalPlansAndPoses(agents_tebs_map_, *agent_model_, fp_size);

    if (teb_.sizePoses() > 0)
      visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);

    if (publish_feedback_)
      visualization_->publishFeedbackMessage(*this, *obstacles_);
  }

  /*
   * registers custom vertices and edges in g2o framework
   */
  void TebOptimalPlanner::registerG2OTypes()
  {
    g2o::Factory *factory = g2o::Factory::instance();
    factory->registerType("VERTEX_POSE", std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
    factory->registerType("VERTEX_TIMEDIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());

    factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
    factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
    factory->registerType("EDGE_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
    factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
    factory->registerType("EDGE_ACCELERATION", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
    factory->registerType("EDGE_ACCELERATION_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
    factory->registerType("EDGE_ACCELERATION_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
    factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
    factory->registerType("EDGE_KINEMATICS_CARLIKE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
    factory->registerType("EDGE_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
    factory->registerType("EDGE_INFLATED_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
    factory->registerType("EDGE_DYNAMIC_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
    factory->registerType("EDGE_INVISIBLE_HUMAN", std::make_shared<g2o::HyperGraphElementCreator<EdgeInvisibleHuman>>());
    factory->registerType("EDGE_INVISIBLE_HUMAN_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeInvisibleHumanVelocity>>());
    factory->registerType("EDGE_VIA_POINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
    factory->registerType("EDGE_PREFER_ROTDIR", std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());

    // Agents
    factory->registerType("EDGE_VELOCITY_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityAgent>>());
    factory->registerType("EDGE_VELOCITY_HOLONOMIC_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomicAgent>>());
    factory->registerType("EDGE_ACCELERATION_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationAgent>>());
    factory->registerType("EDGE_ACCELERATION_AGENT_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationAgentStart>>());
    factory->registerType("EDGE_ACCELERATION_AGENT_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationAgentGoal>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC_AGENT", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicAgent>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC_AGENT_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicAgentStart>>());
    factory->registerType("EDGE_ACCELERATION_HOLONOMIC_AGENT_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicAgentGoal>>());
    factory->registerType("EDGE_AGENT_ROBOT_SAFETY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotSafety>>());
    factory->registerType("EDGE_AGENT_AGENT_SAFETY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentAgentSafety>>());
    factory->registerType("EDGE_AGENT_ROBOT_TTC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotTTC>>());
    factory->registerType("EDGE_AGENT_ROBOT_TTCPLUS", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotTTCplus>>());
    factory->registerType("EDGE_AGENT_ROBOT_REL_VELOCITy", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotRelVelocity>>());
    factory->registerType("EDGE_AGENT_ROBOT_VISIBILITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeAgentRobotVisibility>>());
    factory->registerType("EDGE_STATIC_AGENT_VISIBILITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeStaticAgentVisibility>>());

    return;
  }

  /*
   * initialize g2o optimizer. Set solver settings here.
   * Return: pointer to new SparseOptimizer Object.
   */
  boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
  {
    // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
    static std::once_flag flag;
    std::call_once(flag, &TebOptimalPlanner::registerG2OTypes);

    // allocating the optimizer
    boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
    std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
    linear_solver->setBlockOrdering(true);
    std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer->setAlgorithm(solver);

    optimizer->initMultiThreading(); // required for >Eigen 3.1

    return optimizer;
  }

  bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                      double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, cohan_msgs::msg::OptimizationCostArray *op_costs)
  {

    optimizeTEB(iterations_innerloop, iterations_outerloop, compute_cost_afterwards, obst_cost_scale, viapoint_cost_scale, alternative_time_cost, op_costs, dt_ref_, dt_hysteresis_);
  }

  bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                      double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, cohan_msgs::msg::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst)
  {
    if (optimization_activate_ == false)
      return false;

    bool success = false;
    optimized_ = false;

    double weight_multiplier = 1.0;

    // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
    //                (which leads to better results in terms of x-y-t homotopy planning).
    //                 however, we have not tested this mode intensively yet, so we keep
    //                 the legacy fast mode as default until we finish our tests.
    bool fast_mode = include_dynamic_obstacles_;

    for (int i = 0; i < iterations_outerloop; ++i)
    {
      if (teb_autosize_)
      {
        teb_.autoResize(dt_ref, dt_hyst,
                        min_samples_);
        for (auto &agent_teb_kv : agents_tebs_map_)
          agent_teb_kv.second.autoResize(dt_ref,
                                         dt_hyst,
                                         min_samples_);
      }

      success = buildGraph(weight_multiplier);
      if (!success)
      {
        clearGraph();
        return false;
      }
      // std::cout << "teb_.poses(1) before"  <<teb_.Pose(1)<< '\n';
      success = optimizeGraph(iterations_innerloop, false);
      if (!success)
      {
        clearGraph();
        return false;
      }
      optimized_ = true;
      // std::cout << "teb_.poses(1) after"  <<teb_.Pose(1)<< '\n';

      if (compute_cost_afterwards && i == iterations_outerloop - 1) // compute cost vec only in the last iteration
        computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost, op_costs);

      clearGraph();

      weight_multiplier *= weight_adapt_factor_;
    }

    return true;
  }

  void TebOptimalPlanner::setVelocityStart(const geometry_msgs::msg::Twist &vel_start)
  {
    vel_start_.first = true;
    vel_start_.second.linear.x = vel_start.linear.x;
    vel_start_.second.linear.y = vel_start.linear.y;
    vel_start_.second.angular.z = vel_start.angular.z;
  }

  void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::msg::Twist &vel_goal)
  {
    vel_goal_.first = true;
    vel_goal_.second = vel_goal;
  }

  bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::msg::PoseStamped> &initial_plan, const geometry_msgs::msg::Twist *start_vel, bool free_goal_vel, const AgentPlanVelMap *initial_agent_plan_vel_map, cohan_msgs::msg::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst, int Mode)
  {
    is_mode_ = Mode;
    assert(initialized_);

    auto prep_start_time = rclcpp::Clock().now();

    if (!teb_.isInit())
    {
      // init trajectory
      // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion,cfg_->trajectory.teb_init_skip_dist);
      teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                         min_samples_,
                         teb_init_skip_dist_);
    }
    else if (disable_warm_start_)
    {
      teb_.clearTimedElasticBand();
      // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion,cfg_->trajectory.teb_init_skip_dist);
      teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                         min_samples_,
                         teb_init_skip_dist_);
    }
    else // warm start
    {
      PoseSE2 start_(initial_plan.front().pose);
      PoseSE2 goal_(initial_plan.back().pose);
      if (teb_.sizePoses() > 0 && (goal_.position() - teb_.BackPose().position()).norm() < force_reinit_new_goal_dist_ && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < force_reinit_new_goal_angular_)
      {                                                      // actual warm start!
        teb_.updateAndPruneTEB(start_, goal_, min_samples_); // update TEB
      }
      else // goal too far away -> reinit
      {
        RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                     "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
        teb_.clearTimedElasticBand();
        // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, true, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
        teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                           min_samples_,
                           teb_init_skip_dist_);
      }
    }
    if (start_vel)
      setVelocityStart(*start_vel);
    if (free_goal_vel)
      setVelocityGoalFree();
    else
      vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
    auto prep_time = rclcpp::Clock().now() - prep_start_time;

    auto agent_prep_time_start = rclcpp::Clock().now();
    agents_vel_start_.clear();
    agents_vel_goal_.clear();
    agent_nominal_vels.clear();

    current_agent_robot_min_dist = std::numeric_limits<double>::max();

    switch (planning_mode_)
    {
    case 0:
      agents_tebs_map_.clear();
      break;
    case 1:
    {
      auto itr = agents_tebs_map_.begin();
      while (itr != agents_tebs_map_.end())
      {
        if (initial_agent_plan_vel_map->find(itr->first) ==
            initial_agent_plan_vel_map->end())
          itr = agents_tebs_map_.erase(itr);
        else
          ++itr;
      }

      static_agents.clear();

      auto &rp = initial_plan.front().pose.position;

      for (auto &initial_agent_plan_vel_kv : *initial_agent_plan_vel_map)
      {
        auto &agent_id = initial_agent_plan_vel_kv.first;
        auto &initial_agent_plan = initial_agent_plan_vel_kv.second.plan;

        // is_mode_ = initial_agent_plan_vel_kv.second.is_mode_;
        // erase agent-teb if agent plan is empty
        if (initial_agent_plan.empty())
        {
          auto itr = agents_tebs_map_.find(agent_id);
          if (itr != agents_tebs_map_.end())
          {
            RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                         "New plan: new agent plan is empty. Removing agent trajectories.");
            agents_tebs_map_.erase(itr);
          }
          continue;
        }

        // if (initial_agent_plan.size()<3){
        if (initial_agent_plan[0].header.frame_id == "static")
        {
          static_agents.push_back(initial_agent_plan[0].pose);
          // static_agents.push_back(initial_agent_plan[0].pose);
          // std::cout <<initial_agent_plan[0].pose << std::endl;
          continue;
        }
        // }

        agent_nominal_vels.push_back(initial_agent_plan_vel_kv.second.nominal_vel);

        auto &hp = initial_agent_plan.front().pose.position;
        auto dist = std::hypot(rp.x - hp.x, rp.y - hp.y);
        if (dist < current_agent_robot_min_dist)
        {
          current_agent_robot_min_dist = dist;
        }

        if (agents_tebs_map_.find(agent_id) == agents_tebs_map_.end())
        {

          // create new agent-teb for new agent
          agents_tebs_map_[agent_id] = TimedElasticBand();
          // agents_tebs_map_[agent_id].initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, true, cfg_->trajectory.agent_min_samples, cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);
          agents_tebs_map_[agent_id].initTEBtoGoal(
              initial_agent_plan, dt_ref, true,
              agent_min_samples_,
              teb_init_skip_dist_);
        }
        else if (disable_warm_start_)
        {
          auto &agent_teb = agents_tebs_map_[agent_id];
          agent_teb.clearTimedElasticBand();
          // agent_teb.initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, true, cfg_->trajectory.agent_min_samples, cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);
          agent_teb.initTEBtoGoal(initial_agent_plan, dt_ref,
                                  true, agent_min_samples_,
                                  teb_init_skip_dist_);
        }

        else
        {
          // modify agent-teb for existing agent
          PoseSE2 agent_start_(initial_agent_plan.front().pose);
          PoseSE2 agent_goal_(initial_agent_plan.back().pose);
          auto &agent_teb = agents_tebs_map_[agent_id];
          if (agent_teb.sizePoses() > 0 && (agent_goal_.position() - agent_teb.BackPose().position()).norm() < force_reinit_new_goal_dist_)
            agent_teb.updateAndPruneTEB(agent_start_, agent_goal_, agent_min_samples_);
          else
          {
            RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                         "New goal: distance to existing goal is higher than the specified threshold. Reinitializing agent trajectories.");
            agent_teb.clearTimedElasticBand();
            // agent_teb.initTrajectoryToGoal(initial_agent_plan, cfg_->agent.max_vel_x, true, cfg_->trajectory.agent_min_samples, false, cfg_->trajectory.teb_init_skip_dist);
            agent_teb.initTEBtoGoal(initial_agent_plan, dt_ref,
                                    true, agent_min_samples_,
                                    teb_init_skip_dist_);
          }
        }
        // give start velocity for agents
        std::pair<bool, geometry_msgs::msg::Twist> agent_start_vel;
        agent_start_vel.first = true;
        agent_start_vel.second.linear.x = initial_agent_plan_vel_kv.second.start_vel.linear.x;
        agent_start_vel.second.linear.y = initial_agent_plan_vel_kv.second.start_vel.linear.y;
        agent_start_vel.second.angular.z = initial_agent_plan_vel_kv.second.start_vel.angular.z;
        agents_vel_start_[agent_id] = agent_start_vel;

        // do not set goal velocity for agents
        std::pair<bool, geometry_msgs::msg::Twist> agent_goal_vel;
        agent_goal_vel.first = false;
        // agent_goal_vel.first = true;
        // agent_goal_vel.second.coeffRef(0) =
        //     initial_agent_plan_vel_kv.second.goal_vel.linear.x;
        // agent_goal_vel.second.coeffRef(1) =
        //     initial_agent_plan_vel_kv.second.goal_vel.angular.z;
        // agents_vel_goal_[agent_id] = agent_goal_vel;
      }
      break;
    }
    case 2:
    {
      if (initial_agent_plan_vel_map->size() == 1)
      {
        auto &approach_plan = initial_agent_plan_vel_map->begin()->second.plan;
        if (approach_plan.size() == 1)
        {
          approach_pose_ = approach_plan.front();
          // modify robot global plan
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("optimal_planner"),
                      "empty pose of the agent for approaching");
          // set approach_pose_ same as the current robot pose
          approach_pose_ = initial_plan.front();
        }
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("optimal_planner"),
                    "no or multiple agents for approaching %zu", initial_agent_plan_vel_map->size());
        // set approach_pose_ same as the current robot pose
        approach_pose_ = initial_plan.front();
      }
      break;
    }
    default:
      agents_tebs_map_.clear();
    }
    auto agent_prep_time = rclcpp::Clock().now() - agent_prep_time_start;

    // now optimize
    auto opt_start_time = rclcpp::Clock().now();
    bool teb_opt_result = optimizeTEB(no_inner_iterations_, no_outer_iterations_, true, 1.0, 1.0, false, op_costs, dt_ref, dt_hyst);

    if (op_costs)
    {
      cohan_msgs::msg::OptimizationCost op_cost;
      op_cost.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_MIN_DIST;
      op_cost.cost = current_agent_robot_min_dist;
      op_costs->costs.push_back(op_cost);
    }

    auto opt_time = rclcpp::Clock().now() - opt_start_time;

    auto total_time = rclcpp::Clock().now() - prep_start_time;

    if (total_time.seconds() > 0.1)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                   "\nteb optimal plan times:\n"
                   "\ttotal plan time                %f\n"
                   "\toptimization preparation time  %f\n"
                   "\tagent preparation time         %f\n"
                   "\tteb optimize time              %f\n"
                   "-------------------------",
                   total_time.seconds(),
                   prep_time.seconds(),
                   prep_time.seconds(),
                   opt_time.seconds());
    }

    return teb_opt_result;
  }

  bool TebOptimalPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal, const geometry_msgs::msg::Twist *start_vel, bool free_goal_vel, cohan_msgs::msg::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst, int Mode)
  {
    is_mode_ = Mode;
    auto start_time = rclcpp::Clock().now();
    PoseSE2 start_(start);
    PoseSE2 goal_(goal);
    geometry_msgs::msg::Twist *zero_vel;
    const geometry_msgs::msg::Twist *vel = start_vel ? start_vel : zero_vel;
    auto pre_plan_time = rclcpp::Clock().now() - start_time;
    return plan(start_, goal_, vel, free_goal_vel, pre_plan_time.seconds(), op_costs, dt_ref, dt_hyst);
  }

  bool TebOptimalPlanner::plan(const PoseSE2 &start, const PoseSE2 &goal, const geometry_msgs::msg::Twist *start_vel, bool free_goal_vel, double pre_plan_time, cohan_msgs::msg::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst, int Mode)
  {
    is_mode_ = Mode;
    assert(initialized_);
    auto prep_start_time = rclcpp::Clock().now();
    if (!teb_.isInit())
    {
      // init trajectory
      // teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
      teb_.initTEBtoGoal(start, goal, 0, 1,
                         min_samples_);
    }
    else // warm start
    {
      if (teb_.sizePoses() > 0 && (goal.position() - teb_.BackPose().position()).norm() < force_reinit_new_goal_dist_ && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < force_reinit_new_goal_angular_) // actual warm start!
        teb_.updateAndPruneTEB(start, goal, min_samples_);
      else // goal too far away -> reinit
      {
        RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                     "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
        teb_.clearTimedElasticBand();
        // teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
        teb_.initTEBtoGoal(start, goal, 0, 1, min_samples_);
      }
    }
    if (start_vel)
      setVelocityStart(*start_vel);
    if (free_goal_vel)
      setVelocityGoalFree();
    else
      vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
    auto prep_time = rclcpp::Clock().now() - prep_start_time;
    // now optimize

    auto opt_start_time = rclcpp::Clock().now();
    bool teb_opt_result = optimizeTEB(no_inner_iterations_,
                                      no_outer_iterations_, true, 1.0, 1.0, false, op_costs, dt_ref, dt_hyst);
    auto opt_time = rclcpp::Clock().now() - opt_start_time;

    auto total_time = rclcpp::Clock().now() - prep_start_time;
    if ((total_time.seconds() + pre_plan_time) > 0.05)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                   "\nteb optimal plan times:\n"
                   "\ttotal plan time                %f\n"
                   "\tpre-plan time                  %f\n"
                   "\toptimization preparation time  %f\n"
                   "\tteb optimize time              %f\n"
                   "-------------------------",
                   (total_time.seconds() + pre_plan_time),
                   pre_plan_time,
                   prep_time.seconds(),
                   opt_time.seconds());
    }

    return teb_opt_result;
  }

  bool TebOptimalPlanner::buildGraph(double weight_multiplier)
  {
    if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
    {
      RCLCPP_WARN(rclcpp::get_logger("optimal_planner"),
                  "Cannot build graph, because it is not empty. Call graphClear()!");
      return false;
    }

    // add TEB vertices
    AddTEBVertices();

    // add Edges (local cost functions)
    if (legacy_obstacle_association_)
      AddEdgesObstaclesLegacy(weight_multiplier);
    else
      AddEdgesObstacles(weight_multiplier);

    if (include_dynamic_obstacles_)
      AddEdgesDynamicObstacles();

    AddEdgesViaPoints();

    AddEdgesVelocity();

    AddEdgesAcceleration();

    AddEdgesTimeOptimal();

    AddEdgesShortestPath();

    if (min_turning_radius_ == 0 || weight_kinematics_turning_radius_ == 0)
      AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
    else
      AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

    AddEdgesPreferRotDir();

    switch (planning_mode_)
    {
    case 0:
      break;
    case 1:
      AddEdgesObstaclesForAgents();
      // AddEdgesDynamicObstaclesForAgents();

      AddEdgesViaPointsForAgents();

      AddEdgesVelocityForAgents();
      AddEdgesAccelerationForAgents();

      AddEdgesTimeOptimalForAgents();

      AddEdgesKinematicsDiffDriveForAgents();
      // AddEdgesKinematicsCarlikeForAgents();

      if (use_agent_robot_safety_c_)
      {
        AddEdgesAgentRobotSafety();
      }

      if (use_agent_agent_safety_c_)
      {
        AddEdgesAgentAgentSafety();
      }

      if (use_agent_robot_ttc_c_)
      {
        AddEdgesAgentRobotTTC();
      }

      if (use_agent_robot_ttcplus_c_)
      {
        AddEdgesAgentRobotTTCplus();
      }
      if (use_agent_robot_rel_vel_c_)
      {
        AddEdgesAgentRobotRelVelocity();
      }
      if (use_agent_robot_visi_c_)
      {
        AddEdgesAgentRobotVisibility();
        AddEdgesStaticAgentVisibility();
      }
      if (add_invisible_humans_)
      {
        // AddEdgesInvisibleHumans();
        AddEdgesInvisibleHumansVelocity();
      }
      break;
    case 2:
      AddVertexEdgesApproach();
      break;
    default:
      break;
    }

    return true;
  }

  bool TebOptimalPlanner::optimizeGraph(int no_iterations, bool clear_after)
  {
    if (max_vel_x_ < 0.01)
    {
      RCLCPP_WARN(rclcpp::get_logger("optimal_planner"),
                  "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");

      if (clear_after)
        clearGraph();
      return false;
    }

    if (!teb_.isInit() || (int)teb_.sizePoses() < min_samples_)
    {
      RCLCPP_WARN(rclcpp::get_logger("optimal_planner"),
                  "optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
      if (clear_after)
        clearGraph();
      return false;
    }

    optimizer_->setVerbose(optimization_verbose_);
    optimizer_->initializeOptimization();

    int iter = optimizer_->optimize(no_iterations);

    // Save Hessian for visualization
    //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
    //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

    if (!iter)
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
                   "optimizeGraph(): Optimization failed! iter=%i", iter);
      return false;
    }

    if (clear_after)
      clearGraph();

    return true;
  }

  void TebOptimalPlanner::clearGraph()
  {
    // clear optimizer states
    // optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    if (optimizer_)
    {
      // optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
      optimizer_->vertices().clear(); // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
      optimizer_->clear();
    }
  }

  void TebOptimalPlanner::AddTEBVertices()
  {
    // add vertices to graph
    if (optimization_verbose_)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                   "Adding TEB vertices ...");
    }
    unsigned int id_counter = 0; // used for vertices ids
    for (int i = 0; i < teb_.sizePoses(); ++i)
    {
      teb_.PoseVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.PoseVertex(i));
      if (teb_.sizeTimeDiffs() != 0 && i < teb_.sizeTimeDiffs())
      {
        teb_.TimeDiffVertex(i)->setId(id_counter++);
        optimizer_->addVertex(teb_.TimeDiffVertex(i));
      }
    }

    switch (planning_mode_)
    {
    case 0:
      break;
    case 1:
    {
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_teb = agent_teb_kv.second;
        for (int i = 0; i < agent_teb.sizePoses(); ++i)
        {
          agent_teb.PoseVertex(i)->setId(id_counter++);
          optimizer_->addVertex(agent_teb.PoseVertex(i));
          if (teb_.sizeTimeDiffs() != 0 && i < agent_teb.sizeTimeDiffs())
          {
            agent_teb.TimeDiffVertex(i)->setId(id_counter++);
            optimizer_->addVertex(agent_teb.TimeDiffVertex(i));
          }
        }
      }
      break;
    }
    case 2:
    {
      PoseSE2 approach_pose_se2(approach_pose_.pose);
      approach_pose_vertex = new VertexPose(approach_pose_se2, true);
      approach_pose_vertex->setId(id_counter++);
      optimizer_->addVertex(approach_pose_vertex);
      break;
    }
    default:
      break;
    }
  }

  void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
  {
    if (weight_obstacle_ == 0 || weight_multiplier == 0 || obstacles_ == nullptr)
      return; // if weight equals zero skip adding edges!

    bool inflated = inflation_dist_ > min_obstacle_dist_;

    Eigen::Matrix<double, 1, 1> information;
    information.fill(weight_obstacle_ * weight_multiplier);

    Eigen::Matrix<double, 2, 2> information_inflated;
    information_inflated(0, 0) = weight_obstacle_ * weight_multiplier;
    information_inflated(1, 1) = weight_inflation_;
    information_inflated(0, 1) = information_inflated(1, 0) = 0;

    // iterate all teb points (skip first and last)
    for (int i = 1; i < teb_.sizePoses() - 1; ++i)
    {
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      Obstacle *left_obstacle = nullptr;
      Obstacle *right_obstacle = nullptr;

      std::vector<Obstacle *> relevant_obstacles;

      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

      // iterate obstacles
      for (const ObstaclePtr &obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if (include_dynamic_obstacles_ && obst->isDynamic())
          continue;

        // calculate distance to robot model
        double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());

        // force considering obstacle if really close to the current pose
        if (dist < min_obstacle_dist_ * obstacle_association_force_inclusion_factor_)
        {
          relevant_obstacles.push_back(obst.get());
          continue;
        }
        // cut-off distance
        if (dist > min_obstacle_dist_ * obstacle_association_cutoff_factor_)
          continue;

        // determine side (left or right) and assign obstacle if closer than the previous one
        if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
        {
          if (dist < left_min_dist)
          {
            left_min_dist = dist;
            left_obstacle = obst.get();
          }
        }
        else
        {
          if (dist < right_min_dist)
          {
            right_min_dist = dist;
            right_obstacle = obst.get();
          }
        }
      }

      // create obstacle edges
      if (left_obstacle)
      {
        if (inflated)
        {
          EdgeInflatedObstacle *dist_bandpt_obst = new EdgeInflatedObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information_inflated);
          dist_bandpt_obst->setParameters(robot_model_.get(), left_obstacle);
          optimizer_->addEdge(dist_bandpt_obst);
        }
        else
        {
          EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information);
          dist_bandpt_obst->setParameters(robot_model_.get(), left_obstacle);
          optimizer_->addEdge(dist_bandpt_obst);
        }
      }

      if (right_obstacle)
      {
        if (inflated)
        {
          EdgeInflatedObstacle *dist_bandpt_obst = new EdgeInflatedObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information_inflated);
          dist_bandpt_obst->setParameters(robot_model_.get(), right_obstacle);
          optimizer_->addEdge(dist_bandpt_obst);
        }
        else
        {
          EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information);
          dist_bandpt_obst->setParameters(robot_model_.get(), right_obstacle);
          optimizer_->addEdge(dist_bandpt_obst);
        }
      }

      for (const Obstacle *obst : relevant_obstacles)
      {
        if (inflated)
        {
          EdgeInflatedObstacle *dist_bandpt_obst = new EdgeInflatedObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information_inflated);
          dist_bandpt_obst->setParameters(robot_model_.get(), obst);
          optimizer_->addEdge(dist_bandpt_obst);
        }
        else
        {
          EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
          dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
          dist_bandpt_obst->setInformation(information);
          dist_bandpt_obst->setParameters(robot_model_.get(), obst);
          optimizer_->addEdge(dist_bandpt_obst);
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
  {
    if (weight_obstacle_ == 0 || weight_multiplier == 0 || obstacles_ == nullptr)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 1, 1> information;
    information.fill(weight_obstacle_ * weight_multiplier);

    Eigen::Matrix<double, 2, 2> information_inflated;
    information_inflated(0, 0) = weight_obstacle_ * weight_multiplier;
    information_inflated(1, 1) = weight_inflation_;
    information_inflated(0, 1) = information_inflated(1, 0) = 0;

    bool inflated = inflation_dist_ > min_obstacle_dist_;

    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (include_dynamic_obstacles_ && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
        continue;

      int index;

      if (obstacle_poses_affected_ >= teb_.sizePoses())
        index = teb_.sizePoses() / 2;
      else
        index = teb_.findClosestTrajectoryPose(*(obst->get()));

      // check if obstacle is outside index-range between start and goal
      if ((index <= 1) || (index > teb_.sizePoses() - 2)) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
        continue;

      if (inflated)
      {
        EdgeInflatedObstacle *dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
      }
      else
      {
        EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
      }

      for (int neighbourIdx = 0; neighbourIdx < floor(obstacle_poses_affected_ / 2); neighbourIdx++)
      {
        if (index + neighbourIdx < teb_.sizePoses())
        {
          if (inflated)
          {
            EdgeInflatedObstacle *dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
            dist_bandpt_obst_n_r->setVertex(0, teb_.PoseVertex(index + neighbourIdx));
            dist_bandpt_obst_n_r->setInformation(information_inflated);
            dist_bandpt_obst_n_r->setParameters(robot_model_.get(), obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_r);
          }
          else
          {
            EdgeObstacle *dist_bandpt_obst_n_r = new EdgeObstacle;
            dist_bandpt_obst_n_r->setVertex(0, teb_.PoseVertex(index + neighbourIdx));
            dist_bandpt_obst_n_r->setInformation(information);
            dist_bandpt_obst_n_r->setParameters(robot_model_.get(), obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_r);
          }
        }
        if (index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
        {
          if (inflated)
          {
            EdgeInflatedObstacle *dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
            dist_bandpt_obst_n_l->setVertex(0, teb_.PoseVertex(index - neighbourIdx));
            dist_bandpt_obst_n_l->setInformation(information_inflated);
            dist_bandpt_obst_n_l->setParameters(robot_model_.get(), obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_l);
          }
          else
          {
            EdgeObstacle *dist_bandpt_obst_n_l = new EdgeObstacle;
            dist_bandpt_obst_n_l->setVertex(0, teb_.PoseVertex(index - neighbourIdx));
            dist_bandpt_obst_n_l->setInformation(information);
            dist_bandpt_obst_n_l->setParameters(robot_model_.get(), obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_l);
          }
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesObstaclesForAgents()
  {
    if (weight_obstacle_ == 0 || obstacles_ == NULL)
      return;

    for (ObstContainer::const_iterator obst = obstacles_->begin();
         obst != obstacles_->end(); ++obst)
    {
      if ((*obst)->isDynamic()) // we handle dynamic obstacles differently below
        continue;

      unsigned int index;

      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_teb = agent_teb_kv.second;

        if (obstacle_poses_affected_ >= (int)agent_teb.sizePoses())
          index = agent_teb.sizePoses() / 2;
        else
          index = agent_teb.findClosestTrajectoryPose(*(obst->get()));

        if ((index <= 1) || (index > agent_teb.sizePoses() - 1))
          continue;

        Eigen::Matrix<double, 1, 1> information;
        information.fill(weight_obstacle_);

        EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, agent_teb.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(
            static_cast<CircularRobotFootprintPtr>(agent_model_).get(),
            obst->get());
        optimizer_->addEdge(dist_bandpt_obst);

        for (unsigned int neighbourIdx = 0;
             neighbourIdx < floor(obstacle_poses_affected_ / 2);
             neighbourIdx++)
        {
          if (index + neighbourIdx < agent_teb.sizePoses())
          {
            EdgeObstacle *dist_bandpt_obst_n_r = new EdgeObstacle;
            dist_bandpt_obst_n_r->setVertex(
                0, agent_teb.PoseVertex(index + neighbourIdx));
            dist_bandpt_obst_n_r->setInformation(information);
            dist_bandpt_obst_n_r->setParameters(
                static_cast<CircularRobotFootprintPtr>(agent_model_).get(),
                obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_r);
          }
          if ((int)index - (int)neighbourIdx >=
              0)
          { // TODO: may be > is enough instead of >=
            EdgeObstacle *dist_bandpt_obst_n_l = new EdgeObstacle;
            dist_bandpt_obst_n_l->setVertex(
                0, agent_teb.PoseVertex(index - neighbourIdx));
            dist_bandpt_obst_n_l->setInformation(information);
            dist_bandpt_obst_n_l->setParameters(
                static_cast<CircularRobotFootprintPtr>(agent_model_).get(),
                obst->get());
            optimizer_->addEdge(dist_bandpt_obst_n_l);
          }
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
  {
    if (weight_obstacle_ == 0 || weight_multiplier == 0 || obstacles_ == NULL)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = weight_dynamic_obstacle_ * weight_multiplier;
    information(1, 1) = weight_dynamic_obstacle_inflation_;
    information(0, 1) = information(1, 0) = 0;

    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic() || (*obst)->isHuman())
        continue;

      // Skip first and last pose, as they are fixed
      double time = teb_.TimeDiff(0);
      for (int i = 1; i < teb_.sizePoses() - 1; ++i)
      {
        EdgeDynamicObstacle *dynobst_edge = new EdgeDynamicObstacle(time);
        dynobst_edge->setVertex(0, teb_.PoseVertex(i));
        dynobst_edge->setInformation(information);
        dynobst_edge->setParameters(robot_model_.get(), obst->get());
        optimizer_->addEdge(dynobst_edge);
        time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
      }
    }
  }

  void TebOptimalPlanner::AddEdgesInvisibleHumans(double weight_multiplier)
  {
    if (weight_invisible_human_ == 0 || weight_multiplier == 0 || obstacles_ == NULL || is_mode_ >= 3)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 1, 1> information;
    information(0, 0) = weight_invisible_human_ * weight_multiplier;

    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic() || !(*obst)->isHuman())
        continue;

      // TODO: Need to edit this part
      //  Skip first and last pose, as they are fixed
      double time = teb_.TimeDiff(0);
      for (int i = 1; i < teb_.sizePoses() - 1; ++i)
      {
        EdgeInvisibleHuman *inv_human_edge = new EdgeInvisibleHuman(time);
        inv_human_edge->setVertex(0, teb_.PoseVertex(i));
        inv_human_edge->setInformation(information);
        inv_human_edge->setParameters(robot_model_.get(), obst->get());
        optimizer_->addEdge(inv_human_edge);
        time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
      }
    }
  }

  void TebOptimalPlanner::AddEdgesInvisibleHumansVelocity(double weight_multiplier)
  {
    if (weight_invisible_human_ == 0 || weight_multiplier == 0 || obstacles_ == NULL || is_mode_ >= 3)
      return; // if weight equals zero skip adding edges!

    // std::cout << "Ima" << '\n';
    Eigen::Matrix<double, 1, 1> information;
    information(0, 0) = weight_invisible_human_ * weight_multiplier;

    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic() || !(*obst)->isHuman())
        continue;

      // TODO: Need to edit this part
      //  Skip first and last pose, as they are fixed
      double time = teb_.TimeDiff(0);
      for (int i = 1; i < teb_.sizePoses() - 1; ++i)
      {
        EdgeInvisibleHumanVelocity *inv_human_edge = new EdgeInvisibleHumanVelocity(time);
        inv_human_edge->setVertex(0, teb_.PoseVertex(i));
        inv_human_edge->setVertex(1, teb_.PoseVertex(i + 1));
        inv_human_edge->setVertex(2, teb_.TimeDiffVertex(i));
        inv_human_edge->setInformation(information);
        inv_human_edge->setParameters(robot_model_.get(), obst->get());
        optimizer_->addEdge(inv_human_edge);
        time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
      }
    }
  }

  void TebOptimalPlanner::AddEdgesDynamicObstaclesForAgents(double weight_multiplier)
  {
    if (weight_obstacle_ == 0 || weight_multiplier == 0 || obstacles_ == NULL)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = weight_dynamic_obstacle_ * weight_multiplier;
    information(1, 1) = weight_dynamic_obstacle_inflation_;
    information(0, 1) = information(1, 0) = 0;

    for (ObstContainer::const_iterator obst = obstacles_->begin();
         obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic())
        continue;

      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_teb = agent_teb_kv.second;

        for (std::size_t i = 1; i < agent_teb.sizePoses() - 1; ++i)
        {
          EdgeDynamicObstacle *dynobst_edge = new EdgeDynamicObstacle(i);
          dynobst_edge->setVertex(0, agent_teb.PoseVertex(i));
          dynobst_edge->setVertex(1, agent_teb.TimeDiffVertex(i));
          dynobst_edge->setInformation(information);
          dynobst_edge->setMeasurement(obst->get());
          // dynobst_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(dynobst_edge);
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesViaPoints()
  {
    if (weight_viapoint_ == 0 || via_points_ == NULL || via_points_->empty())
      return; // if weight equals zero skip adding edges!

    int start_pose_idx = 0;

    int n = teb_.sizePoses();
    if (n < 3) // we do not have any degrees of freedom for reaching via-points
      return;

    for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
    {

      int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
      if (via_points_ordered_)
        start_pose_idx = index + 2; // skip a point to have a DOF inbetween for further via-points

      // check if point conicides with goal or is located behind it
      if (index > n - 2)
        index = n - 2; // set to a pose before the goal, since we can move it away!
      // check if point coincides with start or is located before it
      if (index < 1)
      {
        if (via_points_ordered_)
        {
          index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
        }
        else
        {
          RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                       "TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
          continue; // skip via points really close or behind the current robot pose
        }
      }
      Eigen::Matrix<double, 1, 1> information;
      information.fill(weight_viapoint_);

      EdgeViaPoint *edge_viapoint = new EdgeViaPoint;
      edge_viapoint->setVertex(0, teb_.PoseVertex(index));
      edge_viapoint->setInformation(information);
      edge_viapoint->setParameters(&(*vp_it));
      optimizer_->addEdge(edge_viapoint);
    }
  }

  void TebOptimalPlanner::AddEdgesViaPointsForAgents()
  {
    if (weight_agent_viapoint_ == 0 || via_points_ == NULL ||
        via_points_->empty())
      return;

    int start_pose_idx = 0;

    int n = (int)teb_.sizePoses();
    if (n < 3)
      return;
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    for (auto &agent_via_points_kv : *agents_via_points_map_)
    {
      if (agents_tebs_map_.find(agent_via_points_kv.first) == agents_tebs_map_.end())
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("optimal_planner"), clock, THROTTLE_RATE,
                             "inconsistant data between agents_tebs_map and agents_via_points_map (for id %ld)", agent_via_points_kv.first);
        continue;
      }

      auto &agent_via_points = agent_via_points_kv.second;
      auto &agent_teb = agents_tebs_map_[agent_via_points_kv.first];

      for (ViaPointContainer::const_iterator vp_it = agent_via_points.begin();
           vp_it != agent_via_points.end(); ++vp_it)
      {
        int index =
            agent_teb.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
        if (via_points_ordered_)
          start_pose_idx = index + 2;

        if (index > n - 1)
          index = n - 1;
        if (index < 1)
          index = 1;

        Eigen::Matrix<double, 1, 1> information;
        information.fill(weight_agent_viapoint_);

        EdgeViaPoint *edge_viapoint = new EdgeViaPoint;
        edge_viapoint->setVertex(0, agent_teb.PoseVertex(index));
        edge_viapoint->setInformation(information);
        edge_viapoint->setParameters(&(*vp_it));
        optimizer_->addEdge(edge_viapoint);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesVelocity()
  {
    if (max_vel_y_ == 0) // non-holonomic robot
    {
      if (weight_max_vel_x_ == 0 && weight_max_vel_theta_ == 0)
        return; // if weight equals zero skip adding edges!

      int n = teb_.sizePoses();
      Eigen::Matrix<double, 2, 2> information;
      information(0, 0) = weight_max_vel_x_;
      information(1, 1) = weight_max_vel_theta_;
      information(0, 1) = 0.0;
      information(1, 0) = 0.0;

      // for (auto &agent_teb_kv : agents_tebs_map_) {
      //   auto &agent_teb = agent_teb_kv.second;
      for (int i = 0; i < n - 1; ++i)
      {
        EdgeVelocity *velocity_edge = new EdgeVelocity;
        velocity_edge->setVertex(0, teb_.PoseVertex(i));
        velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
        velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
        velocity_edge->setInformation(information);
        velocity_edge->setParameters(robot_model_.get(), is_mode_);
        optimizer_->addEdge(velocity_edge);
      }
      // }
    }
    else // holonomic-robot
    {
      if (weight_max_vel_x_ == 0 && weight_max_vel_y_ == 0 && weight_max_vel_theta_ == 0)
        return; // if weight equals zero skip adding edges!

      int n = teb_.sizePoses();
      Eigen::Matrix<double, 3, 3> information;
      information.fill(0);
      information(0, 0) = weight_max_vel_x_;
      information(1, 1) = weight_max_vel_y_;
      information(2, 2) = weight_max_vel_theta_;

      // for (auto &agent_teb_kv : agents_tebs_map_) {
      //   auto &agent_teb = agent_teb_kv.second;
      for (int i = 0; i < n - 1; ++i)
      {
        // std::cout << "is_mode_" <<is_mode_ << '\n';
        EdgeVelocityHolonomic *velocity_edge = new EdgeVelocityHolonomic;
        velocity_edge->setVertex(0, teb_.PoseVertex(i));
        velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
        velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
        velocity_edge->setInformation(information);
        velocity_edge->setParameters(robot_model_.get(), is_mode_);
        optimizer_->addEdge(velocity_edge);
      }
      // }
    }
  }

  void TebOptimalPlanner::AddEdgesVelocityForAgents()
  {
    if (max_vel_y_ == 0) // non-holonomic robot
    {
      if (weight_max_agent_vel_x_ == 0 && weight_max_agent_vel_theta_ == 0 && weight_nominal_agent_vel_x_ == 0)
        return; // if weight equals zero skip adding edges!

      Eigen::Matrix<double, 3, 3> information;
      information.fill(0);
      information(0, 0) = weight_max_agent_vel_x_;
      information(1, 1) = weight_max_agent_vel_theta_;
      information(2, 2) = weight_nominal_agent_vel_x_;

      int itr_idx = 0;
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        // agent_nominal_vels[itr_idx]
        auto &agent_teb = agent_teb_kv.second;

        int n = agent_teb.sizePoses();
        for (int i = 0; i < n - 1; ++i)
        {
          EdgeVelocityAgent *agent_velocity_edge = new EdgeVelocityAgent;
          agent_velocity_edge->setVertex(0, agent_teb.PoseVertex(i));
          agent_velocity_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
          agent_velocity_edge->setVertex(2, agent_teb.TimeDiffVertex(i));
          agent_velocity_edge->setInformation(information);
          agent_velocity_edge->setParameters(agent_nominal_vels[itr_idx]);
          optimizer_->addEdge(agent_velocity_edge);
        }
        itr_idx++;
      }
    }
    else // holonomic-agent
    {
      if (weight_max_agent_vel_x_ == 0 && weight_max_agent_vel_y_ == 0 && weight_max_agent_vel_theta_ == 0 && weight_nominal_agent_vel_x_ == 0)
        return; // if weight equals zero skip adding edges!

      // int n = agent_teb.sizePoses();
      Eigen::Matrix<double, 4, 4> information;
      information.fill(0);
      information(0, 0) = weight_max_agent_vel_x_;
      information(1, 1) = weight_max_agent_vel_y_;
      information(2, 2) = weight_max_agent_vel_theta_;
      information(3, 3) = weight_nominal_agent_vel_x_;

      int itr_idx = 0;
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_teb = agent_teb_kv.second;

        int n = agent_teb.sizePoses();
        for (int i = 0; i < n - 1; ++i)
        {
          EdgeVelocityHolonomicAgent *agent_velocity_edge = new EdgeVelocityHolonomicAgent;
          agent_velocity_edge->setVertex(0, agent_teb.PoseVertex(i));
          agent_velocity_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
          agent_velocity_edge->setVertex(2, agent_teb.TimeDiffVertex(i));
          agent_velocity_edge->setInformation(information);
          agent_velocity_edge->setParameters(agent_nominal_vels[itr_idx]);
          // agent_velocity_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_velocity_edge);
        }
        itr_idx++;
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAcceleration()
  {
    if (weight_acc_lim_x_ == 0 && weight_acc_lim_theta_ == 0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();

    if (max_vel_y_ == 0 || acc_lim_y_ == 0) // non-holonomic robot
    {
      Eigen::Matrix<double, 2, 2> information;
      information.fill(0);
      information(0, 0) = weight_acc_lim_x_;
      information(1, 1) = weight_acc_lim_theta_;

      // check if an initial velocity should be taken into accound
      if (vel_start_.first)
      {
        EdgeAccelerationStart *acceleration_edge = new EdgeAccelerationStart;
        acceleration_edge->setVertex(0, teb_.PoseVertex(0));
        acceleration_edge->setVertex(1, teb_.PoseVertex(1));
        acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
        acceleration_edge->setInitialVelocity(vel_start_.second);
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i = 0; i < n - 2; ++i)
      {
        EdgeAcceleration *acceleration_edge = new EdgeAcceleration;
        acceleration_edge->setVertex(0, teb_.PoseVertex(i));
        acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
        acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
        acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
        acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (vel_goal_.first)
      {
        EdgeAccelerationGoal *acceleration_edge = new EdgeAccelerationGoal;
        acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
        acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
        acceleration_edge->setVertex(2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
        acceleration_edge->setGoalVelocity(vel_goal_.second);
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }
    }
    else // holonomic robot
    {
      Eigen::Matrix<double, 3, 3> information;
      information.fill(0);
      information(0, 0) = weight_acc_lim_x_;
      information(1, 1) = weight_acc_lim_y_;
      information(2, 2) = weight_acc_lim_theta_;

      // check if an initial velocity should be taken into accound
      if (vel_start_.first)
      {
        EdgeAccelerationHolonomicStart *acceleration_edge = new EdgeAccelerationHolonomicStart;
        acceleration_edge->setVertex(0, teb_.PoseVertex(0));
        acceleration_edge->setVertex(1, teb_.PoseVertex(1));
        acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
        acceleration_edge->setInitialVelocity(vel_start_.second);
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i = 0; i < n - 2; ++i)
      {
        EdgeAccelerationHolonomic *acceleration_edge = new EdgeAccelerationHolonomic;
        acceleration_edge->setVertex(0, teb_.PoseVertex(i));
        acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
        acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
        acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
        acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (vel_goal_.first)
      {
        EdgeAccelerationHolonomicGoal *acceleration_edge = new EdgeAccelerationHolonomicGoal;
        acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
        acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
        acceleration_edge->setVertex(2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
        acceleration_edge->setGoalVelocity(vel_goal_.second);
        acceleration_edge->setInformation(information);
        // acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(acceleration_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAccelerationForAgents()
  {
    if (weight_agent_acc_lim_x_ == 0 && weight_agent_acc_lim_theta_ == 0)
      return; // if weight equals zero skip adding edges!

    if (max_vel_y_ == 0 || acc_lim_y_ == 0) // non-holonomic agent
    {
      Eigen::Matrix<double, 2, 2> information;
      information.fill(0);
      information(0, 0) = weight_agent_acc_lim_x_;
      information(1, 1) = weight_agent_acc_lim_theta_;
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_it = agent_teb_kv.first;
        auto &agent_teb = agent_teb_kv.second;
        // check if an initial velocity should be taken into accound
        int n = agent_teb.sizePoses();

        if (agents_vel_start_[agent_it].first)
        {
          EdgeAccelerationAgentStart *agent_acceleration_edge = new EdgeAccelerationAgentStart;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(0));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(1));
          agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(0));
          agent_acceleration_edge->setInitialVelocity(agents_vel_start_[agent_it].second);
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }

        // now add the usual acceleration edge for each tuple of three teb poses
        for (int i = 0; i < n - 2; ++i)
        {
          EdgeAccelerationAgent *agent_acceleration_edge = new EdgeAccelerationAgent;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(i));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
          agent_acceleration_edge->setVertex(2, agent_teb.PoseVertex(i + 2));
          agent_acceleration_edge->setVertex(3, agent_teb.TimeDiffVertex(i));
          agent_acceleration_edge->setVertex(4, agent_teb.TimeDiffVertex(i + 1));
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }

        // check if a goal velocity should be taken into accound
        if (agents_vel_goal_[agent_it].first)
        {
          EdgeAccelerationAgentGoal *agent_acceleration_edge = new EdgeAccelerationAgentGoal;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(n - 2));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(n - 1));
          agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(agent_teb.sizeTimeDiffs() - 1));
          agent_acceleration_edge->setGoalVelocity(agents_vel_goal_[agent_it].second);
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }
      }
    }
    else // holonomic robot
    {
      Eigen::Matrix<double, 3, 3> information;
      information.fill(0);
      information(0, 0) = weight_acc_lim_x_;
      information(1, 1) = weight_acc_lim_y_;
      information(2, 2) = weight_acc_lim_theta_;
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_it = agent_teb_kv.first;
        auto &agent_teb = agent_teb_kv.second;
        // check if an initial velocity should be taken into accound
        int n = agent_teb.sizePoses();
        // check if an initial velocity should be taken into accound
        if (vel_start_.first)
        {
          EdgeAccelerationHolonomicAgentStart *agent_acceleration_edge = new EdgeAccelerationHolonomicAgentStart;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(0));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(1));
          agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(0));
          agent_acceleration_edge->setInitialVelocity(agents_vel_start_[agent_it].second);
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }

        // now add the usual acceleration edge for each tuple of three teb poses
        for (int i = 0; i < n - 2; ++i)
        {
          EdgeAccelerationHolonomicAgent *agent_acceleration_edge = new EdgeAccelerationHolonomicAgent;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(i));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
          agent_acceleration_edge->setVertex(2, agent_teb.PoseVertex(i + 2));
          agent_acceleration_edge->setVertex(3, agent_teb.TimeDiffVertex(i));
          agent_acceleration_edge->setVertex(4, agent_teb.TimeDiffVertex(i + 1));
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }

        // check if a goal velocity should be taken into accound
        if (vel_goal_.first)
        {
          EdgeAccelerationHolonomicAgentGoal *agent_acceleration_edge = new EdgeAccelerationHolonomicAgentGoal;
          agent_acceleration_edge->setVertex(0, agent_teb.PoseVertex(n - 2));
          agent_acceleration_edge->setVertex(1, agent_teb.PoseVertex(n - 1));
          agent_acceleration_edge->setVertex(2, agent_teb.TimeDiffVertex(agent_teb.sizeTimeDiffs() - 1));
          agent_acceleration_edge->setGoalVelocity(agents_vel_goal_[agent_it].second);
          agent_acceleration_edge->setInformation(information);
          // agent_acceleration_edge->setHATebConfig(*cfg_);
          optimizer_->addEdge(agent_acceleration_edge);
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesTimeOptimal()
  {
    if (weight_optimaltime_ == 0)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 1, 1> information;
    information.fill(weight_optimaltime_);

    for (int i = 0; i < teb_.sizeTimeDiffs(); ++i)
    {
      EdgeTimeOptimal *timeoptimal_edge = new EdgeTimeOptimal;
      timeoptimal_edge->setVertex(0, teb_.TimeDiffVertex(i));
      timeoptimal_edge->setInformation(information);
      // timeoptimal_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(timeoptimal_edge);
    }
  }

  void TebOptimalPlanner::AddEdgesTimeOptimalForAgents()
  {
    if (weight_agent_optimaltime_ == 0)
    {
      return;
    }

    Eigen::Matrix<double, 1, 1> information;
    information.fill(weight_agent_optimaltime_);

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;

      std::size_t NoTimeDiffs(agent_teb.sizeTimeDiffs());
      for (std::size_t i = 0; i < NoTimeDiffs; ++i)
      {
        EdgeTimeOptimal *timeoptimal_edge = new EdgeTimeOptimal;
        timeoptimal_edge->setVertex(0, agent_teb.TimeDiffVertex(i));
        timeoptimal_edge->setInformation(information);
        // timeoptimal_edge->setHATebConfig(*cfg_);
        // timeoptimal_edge->setInitialTime(agent_teb.TimeDiffVertex(i)->dt());
        optimizer_->addEdge(timeoptimal_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesShortestPath()
  {
    if (weight_shortest_path_ == 0)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double, 1, 1> information;
    information.fill(weight_shortest_path_);

    for (int i = 0; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeShortestPath *shortest_path_edge = new EdgeShortestPath;
      shortest_path_edge->setVertex(0, teb_.PoseVertex(i));
      shortest_path_edge->setVertex(1, teb_.PoseVertex(i + 1));
      shortest_path_edge->setInformation(information);
      // shortest_path_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(shortest_path_edge);
    }
  }

  void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
  {
    if (weight_kinematics_nh_ == 0 && weight_kinematics_forward_drive_ == 0)
      return; // if weight equals zero skip adding edges!

    // create edge for satisfiying kinematic constraints
    Eigen::Matrix<double, 2, 2> information_kinematics;
    information_kinematics.fill(0.0);
    information_kinematics(0, 0) = weight_kinematics_nh_;
    information_kinematics(1, 1) = weight_kinematics_forward_drive_;

    for (int i = 0; i < teb_.sizePoses() - 1; i++) // ignore twiced start only
    {
      EdgeKinematicsDiffDrive *kinematics_edge = new EdgeKinematicsDiffDrive;
      kinematics_edge->setVertex(0, teb_.PoseVertex(i));
      kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
      kinematics_edge->setInformation(information_kinematics);
      // kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }

  void TebOptimalPlanner::AddEdgesKinematicsDiffDriveForAgents()
  {
    if (weight_kinematics_nh_ == 0 &&
        weight_kinematics_forward_drive_ == 0)
      return; // if weight equals zero skip adding edges!

    // create edge for satisfiying kinematic constraints
    Eigen::Matrix<double, 2, 2> information_kinematics;
    information_kinematics.fill(0.0);
    information_kinematics(0, 0) = weight_kinematics_nh_;
    information_kinematics(1, 1) = weight_kinematics_forward_drive_;

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;
      for (unsigned int i = 0; i < agent_teb.sizePoses() - 1; i++)
      {
        EdgeKinematicsDiffDrive *kinematics_edge = new EdgeKinematicsDiffDrive;
        kinematics_edge->setVertex(0, agent_teb.PoseVertex(i));
        kinematics_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        kinematics_edge->setInformation(information_kinematics);
        // kinematics_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(kinematics_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesKinematicsCarlike()
  {
    if (weight_kinematics_nh_ == 0 && weight_kinematics_turning_radius_ == 0)
      return; // if weight equals zero skip adding edges!

    // create edge for satisfiying kinematic constraints
    Eigen::Matrix<double, 2, 2> information_kinematics;
    information_kinematics.fill(0.0);
    information_kinematics(0, 0) = weight_kinematics_nh_;
    information_kinematics(1, 1) = weight_kinematics_turning_radius_;

    for (int i = 0; i < teb_.sizePoses() - 1; i++) // ignore twiced start only
    {
      EdgeKinematicsCarlike *kinematics_edge = new EdgeKinematicsCarlike;
      kinematics_edge->setVertex(0, teb_.PoseVertex(i));
      kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
      kinematics_edge->setInformation(information_kinematics);
      // kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }

  void TebOptimalPlanner::AddEdgesKinematicsCarlikeForAgents()
  {
    if (weight_kinematics_nh_ == 0 && weight_kinematics_turning_radius_ == 0)
      return; // if weight equals zero skip adding edges!

    // create edge for satisfiying kinematic constraints
    Eigen::Matrix<double, 2, 2> information_kinematics;
    information_kinematics.fill(0.0);
    information_kinematics(0, 0) = weight_kinematics_nh_;
    information_kinematics(1, 1) = weight_kinematics_turning_radius_;

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;
      for (int i = 0; i < agent_teb.sizePoses() - 1; i++) // ignore twiced start only
      {
        EdgeKinematicsCarlike *kinematics_edge = new EdgeKinematicsCarlike;
        kinematics_edge->setVertex(0, agent_teb.PoseVertex(i));
        kinematics_edge->setVertex(1, agent_teb.PoseVertex(i + 1));
        kinematics_edge->setInformation(information_kinematics);
        // kinematics_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(kinematics_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesPreferRotDir()
  {
    // TODO(roesmann): Note, these edges can result in odd predictions, in particular
    //                 we can observe a substantional mismatch between open- and closed-loop planning
    //                 leading to a poor control performance.
    //                 At the moment, we keep these functionality for oscillation recovery:
    //                 Activating the edge for a short time period might not be crucial and
    //                 could move the robot to a new oscillation-free state.
    //                 This needs to be analyzed in more detail!
    if (prefer_rotdir_ == RotType::none || weight_prefer_rotdir_ == 0)
      return; // if weight equals zero skip adding edges!

    if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
    {
      RCLCPP_WARN(rclcpp::get_logger("optimal_planner"),
                  "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
      return;
    }

    // create edge for satisfiying kinematic constraints
    Eigen::Matrix<double, 1, 1> information_rotdir;
    information_rotdir.fill(weight_prefer_rotdir_);

    for (int i = 0; i < teb_.sizePoses() - 1 && i < 3; ++i) // currently: apply to first 3 rotations
    {
      EdgePreferRotDir *rotdir_edge = new EdgePreferRotDir;
      rotdir_edge->setVertex(0, teb_.PoseVertex(i));
      rotdir_edge->setVertex(1, teb_.PoseVertex(i + 1));
      rotdir_edge->setInformation(information_rotdir);

      if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
      else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();

      optimizer_->addEdge(rotdir_edge);
    }
  }

  void TebOptimalPlanner::AddEdgesAgentRobotSafety()
  {
    auto robot_teb_size = (int)teb_.sizePoses();

    double min_dist_ = min_agent_robot_dist_;
    double weight_safety = weight_agent_robot_safety_;

    if (current_agent_robot_min_dist < 2.0)
    {
      for (auto &agent_teb_kv : agents_tebs_map_)
      {
        auto &agent_teb = agent_teb_kv.second;

        for (unsigned int i = 0;
             (i < agent_teb.sizePoses()) && (i < robot_teb_size); i++)
        {
          Eigen::Matrix<double, 1, 1> information_agent_robot;
          information_agent_robot.fill(weight_safety);
          EdgeAgentRobotSafety *agent_robot_safety_edge = new EdgeAgentRobotSafety;
          agent_robot_safety_edge->setVertex(0, teb_.PoseVertex(i));
          agent_robot_safety_edge->setVertex(1, agent_teb.PoseVertex(i));
          agent_robot_safety_edge->setInformation(information_agent_robot);
          agent_robot_safety_edge->setParameters(robot_model_.get(),
                                                 agent_radius_, min_dist_);
          optimizer_->addEdge(agent_robot_safety_edge);
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAgentAgentSafety()
  {
    for (auto oi = agents_tebs_map_.begin(); oi != agents_tebs_map_.end();)
    {
      auto &agent1_teb = oi->second;
      for (auto ii = ++oi; ii != agents_tebs_map_.end(); ii++)
      {
        auto &agent2_teb = ii->second;

        for (unsigned int k = 0;
             (k < agent1_teb.sizePoses()) && (k < agent2_teb.sizePoses()); k++)
        {
          Eigen::Matrix<double, 1, 1> information_agent_agent;
          information_agent_agent.fill(weight_agent_agent_safety_);

          EdgeAgentAgentSafety *agent_agent_safety_edge =
              new EdgeAgentAgentSafety;
          agent_agent_safety_edge->setVertex(0, agent1_teb.PoseVertex(k));
          agent_agent_safety_edge->setVertex(1, agent2_teb.PoseVertex(k));
          agent_agent_safety_edge->setInformation(information_agent_agent);
          agent_agent_safety_edge->setParameters(agent_radius_);
          optimizer_->addEdge(agent_agent_safety_edge);
        }
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAgentRobotTTC()
  {
    Eigen::Matrix<double, 1, 1> information_agent_robot_ttc;
    information_agent_robot_ttc.fill(weight_agent_robot_ttc_);

    auto robot_teb_size = (int)teb_.sizePoses();

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;

      size_t agent_teb_size = agent_teb.sizePoses();
      for (unsigned int i = 0;
           (i < agent_teb_size - 1) && (i < robot_teb_size - 1); i++)
      {

        EdgeAgentRobotTTC *agent_robot_ttc_edge = new EdgeAgentRobotTTC;
        agent_robot_ttc_edge->setVertex(0, teb_.PoseVertex(i));
        agent_robot_ttc_edge->setVertex(1, teb_.PoseVertex(i + 1));
        agent_robot_ttc_edge->setVertex(2, teb_.TimeDiffVertex(i));
        agent_robot_ttc_edge->setVertex(3, agent_teb.PoseVertex(i));
        agent_robot_ttc_edge->setVertex(4, agent_teb.PoseVertex(i + 1));
        agent_robot_ttc_edge->setVertex(5, agent_teb.TimeDiffVertex(i));
        agent_robot_ttc_edge->setInformation(information_agent_robot_ttc);
        agent_robot_ttc_edge->setParameters(robot_radius_, agent_radius_);
        optimizer_->addEdge(agent_robot_ttc_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAgentRobotTTCplus()
  {
    Eigen::Matrix<double, 1, 1> information_agent_robot_ttcplus;
    information_agent_robot_ttcplus.fill(weight_agent_robot_ttcplus_);

    auto robot_teb_size = teb_.sizePoses();
    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;
      size_t agent_teb_size = agent_teb.sizePoses();
      for (unsigned int i = 0;
           (i < agent_teb_size - 1) && (i < robot_teb_size - 1) && (is_mode_ == 0); i++)
      {

        EdgeAgentRobotTTCplus *agent_robot_ttcplus_edge = new EdgeAgentRobotTTCplus();
        agent_robot_ttcplus_edge->setVertex(0, teb_.PoseVertex(i));
        agent_robot_ttcplus_edge->setVertex(1, teb_.PoseVertex(i + 1));
        agent_robot_ttcplus_edge->setVertex(2, teb_.TimeDiffVertex(i));
        agent_robot_ttcplus_edge->setVertex(3, agent_teb.PoseVertex(i));
        agent_robot_ttcplus_edge->setVertex(4, agent_teb.PoseVertex(i + 1));
        agent_robot_ttcplus_edge->setVertex(5, agent_teb.TimeDiffVertex(i));
        agent_robot_ttcplus_edge->setInformation(information_agent_robot_ttcplus);
        agent_robot_ttcplus_edge->setParameters(robot_radius_, agent_radius_);
        optimizer_->addEdge(agent_robot_ttcplus_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAgentRobotRelVelocity()
  {
    Eigen::Matrix<double, 1, 1> information_agent_robot_rel_vel;
    information_agent_robot_rel_vel.fill(weight_agent_robot_rel_vel_);

    auto robot_teb_size = (int)teb_.sizePoses();
    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;

      size_t agent_teb_size = agent_teb.sizePoses();
      for (unsigned int i = 0; (i < agent_teb_size - 1) && (i < robot_teb_size - 1); i++)
      {

        EdgeAgentRobotRelVelocity *agent_robot_rel_vel_edge = new EdgeAgentRobotRelVelocity;
        agent_robot_rel_vel_edge->setVertex(0, teb_.PoseVertex(i));
        agent_robot_rel_vel_edge->setVertex(1, teb_.PoseVertex(i + 1));
        agent_robot_rel_vel_edge->setVertex(2, teb_.TimeDiffVertex(i));
        agent_robot_rel_vel_edge->setVertex(3, agent_teb.PoseVertex(i));
        agent_robot_rel_vel_edge->setVertex(4, agent_teb.PoseVertex(i + 1));
        agent_robot_rel_vel_edge->setVertex(5, agent_teb.TimeDiffVertex(i));
        agent_robot_rel_vel_edge->setInformation(information_agent_robot_rel_vel);
        // agent_robot_rel_vel_edge->setParameters(*cfg_);
        optimizer_->addEdge(agent_robot_rel_vel_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesAgentRobotVisibility()
  {
    auto robot_teb_size = (int)teb_.sizePoses();

    for (auto &agent_teb_kv : agents_tebs_map_)
    {
      auto &agent_teb = agent_teb_kv.second;

      for (unsigned int i = 0;
           (i < agent_teb.sizePoses()) && (i < robot_teb_size); i++)
      {
        Eigen::Matrix<double, 1, 1> information_agent_robot;
        information_agent_robot.fill(weight_agent_robot_visibility_);

        EdgeAgentRobotVisibility *agent_robot_visibility_edge = new EdgeAgentRobotVisibility;
        agent_robot_visibility_edge->setVertex(0, teb_.PoseVertex(i));
        agent_robot_visibility_edge->setVertex(1, agent_teb.PoseVertex(i));
        agent_robot_visibility_edge->setInformation(information_agent_robot);
        // agent_robot_visibility_edge->setParameters(*cfg_);
        optimizer_->addEdge(agent_robot_visibility_edge);
      }
    }
  }

  void TebOptimalPlanner::AddEdgesStaticAgentVisibility()
  {
    auto robot_teb_size = (int)teb_.sizePoses();

    // for (auto &agent_teb_kv : agents_tebs_map_) {
    //     auto &agent_teb = agent_teb_kv.second;
    for (auto &agent : static_agents)
    {
      PoseSE2 agent_pose(agent);
      for (unsigned int i = 0; i < robot_teb_size; i++)
      {
        Eigen::Matrix<double, 1, 1> information_agent_robot;
        information_agent_robot.fill(weight_agent_robot_visibility_);

        EdgeStaticAgentVisibility *static_agent_visibility_edge = new EdgeStaticAgentVisibility;
        static_agent_visibility_edge->setVertex(0, teb_.PoseVertex(i));
        static_agent_visibility_edge->setInformation(information_agent_robot);
        static_agent_visibility_edge->setParameters(agent_pose);
        optimizer_->addEdge(static_agent_visibility_edge);
      }
    }
  }

  void TebOptimalPlanner::AddVertexEdgesApproach()
  {
    if (!approach_pose_vertex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
                   "approch pose vertex does not exist");
      return;
    }
    double min_dist_ = min_agent_robot_dist_;
    // if(is_mode_==1){
    //   min_dist_ = 0.2;
    // }

    Eigen::Matrix<double, 1, 1> information_approach;
    information_approach.fill(weight_obstacle_);

    for (auto &teb_pose : teb_.poses())
    {
      EdgeAgentRobotSafety *approach_edge = new EdgeAgentRobotSafety;
      approach_edge->setVertex(0, teb_pose);
      approach_edge->setVertex(1, approach_pose_vertex);
      approach_edge->setInformation(information_approach);
      approach_edge->setParameters(robot_model_.get(), agent_radius_, min_dist_);
      optimizer_->addEdge(approach_edge);
    }
  }

  void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, cohan_msgs::msg::OptimizationCostArray *op_costs)
  {
    // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
    bool graph_exist_flag(false);
    if (optimizer_->edges().empty() && optimizer_->vertices().empty())
    {
      // here the graph is build again, for time efficiency make sure to call this function
      // between buildGraph and Optimize (deleted), but it depends on the application
      buildGraph();
      optimizer_->initializeOptimization();
    }
    else
    {
      graph_exist_flag = true;
    }

    optimizer_->computeInitialGuess();

    cost_ = 0;
    double time_opt_cost = 0.0, kinematics_dd_cost = 0.0, shortest_path_cost = 0.0,
           kinematics_cl_cost = 0.0, robot_vel_cost = 0.0, agent_vel_cost = 0.0, robot_vel_holo_cost = 0.0, robot_acc_holo_cost = 0.0,
           robot_acc_cost = 0.0, agent_acc_cost = 0.0, agent_vel_holo_cost = 0.0, agent_acc_holo_cost = 0.0, obst_cost = 0.0, rot_dir_cost = 0.0,
           dyn_obst_cost = 0.0, via_cost = 0.0, hr_safety_cost = 0.0, inv_human_cost = 0.0,
           hh_safety_cost = 0.0, hr_ttc_cost = 0.0, hr_ttclosest_cost = 0.0, hr_ttcplus_cost = 0.0, hr_rel_vel_cost = 0.0, hr_visi_cost = 0.0, ttcplus_error = 0.0;

    std::vector<double> time_opt_cost_vector, kinematics_dd_cost_vector, kinematics_cl_cost_vector,
        robot_vel_cost_vector, agent_vel_cost_vector, robot_vel_holo_cost_vector, agent_vel_holo_cost_vector, robot_acc_cost_vector, robot_acc_holo_cost_vector,
        agent_acc_cost_vector, agent_acc_holo_cost_vector, obst_cost_vector, dyn_obst_cost_vector, via_cost_vector, hr_safety_cost_vector,
        hh_safety_cost_vector, hr_ttc_cost_vector, hr_ttclosest_cost_vector, hr_ttcplus_cost_vector,
        hr_rel_vel_cost_vector, hr_visi_cost_vector, shortest_path_cost_vector, inv_human_cost_vector;

    bool f1 = false, f2 = false, f3 = false, f4 = false, f5 = false;
    double ttc_first, ttcplus_first, obs_first, safety_first, visible_first;

    if (alternative_time_cost)
    {
      cost_ += teb_.getSumOfAllTimeDiffs();
      // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
      // since we are using an AutoResize Function with hysteresis.
    }

    // now we need pointers to all edges -> calculate error for each edge-type
    // since we aren't storing edge pointers, we need to check every edge
    int i = 0;
    for (std::vector<g2o::OptimizableGraph::Edge *>::const_iterator it = optimizer_->activeEdges().begin(); it != optimizer_->activeEdges().end(); it++)
    {
      i++;
      double cur_cost = (*it)->chi2();

      // if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
      //     || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
      //     || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
      // {
      //   cur_cost *= obst_cost_scale;
      // }
      // else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
      // {
      //   cur_cost *= viapoint_cost_scale;
      // }
      // else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
      // {
      //   continue; // skip these edges if alternative_time_cost is active
      // }

      ////////////////////////////////////////////////////////////////////////////////////////

      if (dynamic_cast<EdgeTimeOptimal *>(*it) != nullptr && !alternative_time_cost)
      {
        time_opt_cost += cur_cost;
        time_opt_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeShortestPath *>(*it) != nullptr)
      {
        shortest_path_cost += cur_cost;
        shortest_path_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgePreferRotDir *>(*it) != nullptr)
      {
        rot_dir_cost += cur_cost;
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeKinematicsDiffDrive *>(*it) != nullptr)
      {
        kinematics_dd_cost += cur_cost;
        kinematics_dd_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeKinematicsCarlike *>(*it) != nullptr)
      {
        kinematics_cl_cost += cur_cost;
        kinematics_cl_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeVelocity *>(*it) != nullptr)
      {
        robot_vel_cost += cur_cost;
        robot_vel_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeVelocityHolonomic *>(*it) != nullptr)
      {
        robot_vel_holo_cost += cur_cost;
        robot_vel_holo_cost_vector.push_back(cur_cost);

        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeVelocityAgent *>(*it) != nullptr)
      {
        agent_vel_cost += cur_cost;
        agent_vel_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeVelocityHolonomicAgent *>(*it) != nullptr)
      {
        agent_vel_holo_cost += cur_cost;
        agent_vel_holo_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      // TODO: add cost of start and goal accelerations

      if (dynamic_cast<EdgeAcceleration *>(*it) != nullptr)
      {
        robot_acc_cost += cur_cost;
        robot_acc_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAccelerationHolonomic *>(*it) != nullptr)
      {
        robot_acc_holo_cost += cur_cost;
        robot_acc_holo_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAccelerationAgent *>(*it) != nullptr)
      {
        agent_acc_cost += cur_cost;
        agent_acc_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAccelerationHolonomicAgent *>(*it) != nullptr)
      {
        agent_acc_holo_cost += cur_cost;
        agent_acc_holo_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeObstacle *>(*it) != nullptr)
      {
        cur_cost *= obst_cost_scale;
        obst_cost += cur_cost;
        cost_ += cur_cost;
        obst_cost_vector.push_back(cur_cost);
        if (!f5)
        {
          obs_first = cur_cost;
          f5 = true;
        }
        continue;
      }

      if (dynamic_cast<EdgeInflatedObstacle *>(*it) != nullptr)
      {
        cur_cost *= obst_cost_scale;
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeDynamicObstacle *>(*it) != nullptr)
      {
        cur_cost *= obst_cost_scale;
        dyn_obst_cost += cur_cost;
        dyn_obst_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeInvisibleHuman *>(*it) != nullptr)
      {
        inv_human_cost += cur_cost;
        inv_human_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeViaPoint *>(*it) != nullptr)
      {
        cur_cost *= viapoint_cost_scale;
        via_cost += cur_cost;
        via_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAgentRobotSafety *>(*it) != nullptr)
      {
        hr_safety_cost += cur_cost;
        hr_safety_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        if (!f1)
        {
          safety_first = cur_cost;
          f1 = true;
        }

        continue;
      }

      if (dynamic_cast<EdgeAgentAgentSafety *>(*it) != nullptr)
      {
        hh_safety_cost += cur_cost;
        hh_safety_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAgentRobotTTC *>(*it) != nullptr)
      {
        hr_ttc_cost += cur_cost;
        hr_ttc_cost_vector.push_back(cur_cost);

        cost_ += cur_cost;
        if (!f2)
        {
          ttc_first = cur_cost;
          f2 = true;
        }
        continue;
      }

      EdgeAgentRobotTTCplus *edge_agent_robot_ttcplus = dynamic_cast<EdgeAgentRobotTTCplus *>(*it);
      if (edge_agent_robot_ttcplus != NULL)
      {
        cost_ += edge_agent_robot_ttcplus->getError().squaredNorm();
        hr_ttcplus_cost += edge_agent_robot_ttcplus->getError().squaredNorm();
        ttcplus_error += edge_agent_robot_ttcplus->getError()[0];
        hr_ttcplus_cost_vector.push_back(edge_agent_robot_ttcplus->getError().squaredNorm());
        if (!f3)
        {
          ttcplus_first = hr_ttcplus_cost;
          f3 = true;
        }

        continue;
      }

      if (dynamic_cast<EdgeAgentRobotRelVelocity *>(*it) != nullptr)
      {
        hr_rel_vel_cost += cur_cost;
        hr_rel_vel_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        continue;
      }

      if (dynamic_cast<EdgeAgentRobotVisibility *>(*it) != nullptr)
      {
        hr_visi_cost += cur_cost;
        hr_visi_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        if (!f4)
        {
          visible_first = cur_cost;
          f4 = true;
        }
        continue;
      }
    }

    if (op_costs)
    {
      op_costs->costs.clear();

      cohan_msgs::msg::OptimizationCost optc;

      optc.type = cohan_msgs::msg::OptimizationCost::TIME_OPTIMALITY;
      optc.cost = time_opt_cost;
      optc.costs_arr = time_opt_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::KINEMATIC_DD;
      optc.cost = kinematics_dd_cost;
      optc.costs_arr = kinematics_dd_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::KINEMATIC_CL;
      optc.cost = kinematics_cl_cost;
      optc.costs_arr = kinematics_cl_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::ROBOT_VEL;
      optc.cost = robot_vel_holo_cost;
      optc.costs_arr = robot_vel_holo_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_VEL;
      optc.cost = agent_vel_holo_cost;
      optc.costs_arr = agent_vel_holo_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::ROBOT_ACC;
      optc.cost = robot_acc_holo_cost;
      optc.costs_arr = robot_acc_holo_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ACC;
      optc.cost = agent_acc_holo_cost;
      optc.costs_arr = agent_acc_holo_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::OBSTACLE;
      // optc.cost = obst_cost;
      optc.cost = obs_first;
      optc.costs_arr = obst_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::DYNAMIC_OBSTACLE;
      optc.cost = dyn_obst_cost;
      optc.costs_arr = dyn_obst_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::VIA_POINT;
      optc.cost = via_cost;
      optc.costs_arr = via_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_SAFETY;
      optc.cost = hr_safety_cost;
      // optc.cost = safety_first;
      optc.costs_arr = hr_safety_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_AGENT_SAFETY;
      optc.cost = hh_safety_cost;
      optc.costs_arr = hh_safety_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_TTC;
      optc.cost = hr_ttc_cost;
      optc.costs_arr = hr_ttc_cost_vector;
      // optc.cost = ttc_first;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_TTCLOSEST; // michele
      optc.cost = hr_ttclosest_cost;
      optc.costs_arr = hr_ttclosest_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_TTCPLUS; // michele
      optc.cost = hr_ttcplus_cost;
      optc.costs_arr = hr_ttcplus_cost_vector;
      // optc.cost = ttcplus_first;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_REL_VEL;
      optc.cost = hr_rel_vel_cost;
      optc.costs_arr = hr_rel_vel_cost_vector;
      op_costs->costs.push_back(optc);

      optc.type = cohan_msgs::msg::OptimizationCost::AGENT_ROBOT_VISIBILITY;
      optc.cost = hr_visi_cost;
      optc.costs_arr = hr_visi_cost_vector;
      // optc.cost = visible_first;
      op_costs->costs.push_back(optc);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
                 "Costs:\n\ttime_opt_cost = %.2f\n\tkinematics_dd_cost = "
                 "%.2f\n\tkinematics_cl_cost = %.2f\n\trobot_vel_cost = "
                 "%.2f\n\tagent_vel_cost = %.2f\n\trobot_acc_cost = "
                 "%.2f\n\tagent_acc_cost = %.2f\n\tobst_cost = "
                 "%.2f\n\tdyn_obst_cost = %.2f\n\tvia_cost = "
                 "%.2f\n\thr_safety_cost = %.2f\n\thh_safety_cost = "
                 "%.2f\n\thr_ttc_cost =   %.2f\n\thr_rel_vel_cost = "
                 "%.2f\n\thr_ttclosest_cost = %.2f\n\thr_ttcplus_cost = " // michele
                 "%.2f\n\thr_visi_cost = %.2f\n\ttotal_tab_time = %.2f",
                 time_opt_cost, kinematics_dd_cost, kinematics_cl_cost,
                 robot_vel_cost, agent_vel_cost, robot_acc_cost, agent_acc_cost,
                 obst_cost, dyn_obst_cost, via_cost, hr_safety_cost, hh_safety_cost,
                 hr_ttc_cost, hr_rel_vel_cost, hr_ttclosest_cost, hr_ttcplus_cost, hr_visi_cost, teb_.getSumOfAllTimeDiffs());

    // delete temporary created graph
    if (!graph_exist_flag)
      clearGraph();
  }

  void TebOptimalPlanner::extractVelocity(const PoseSE2 &pose1, const PoseSE2 &pose2, double dt, double &vx, double &vy, double &omega) const
  {
    if (dt == 0)
    {
      vx = 0;
      vy = 0;
      omega = 0;
      return;
    }

    Eigen::Vector2d deltaS = pose2.position() - pose1.position();

    if (max_vel_y_ == 0) // nonholonomic robot
    {
      Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
      // translational velocity
      double dir = deltaS.dot(conf1dir);
      vx = (double)g2o::sign(dir) * deltaS.norm() / dt;
      vy = 0;
    }

    else // holonomic robot
    {
      // transform pose 2 into the current robot frame (pose1)
      // for velocities only the rotation of the direction vector is necessary.
      // (map->pose1-frame: inverse 2d rotation matrix)
      double cos_theta1 = std::cos(pose1.theta());
      double sin_theta1 = std::sin(pose1.theta());
      double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
      double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
      vx = p1_dx / dt;
      vy = p1_dy / dt;
    }

    // rotational velocity
    double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
    omega = orientdiff / dt;
  }

  bool TebOptimalPlanner::getVelocityCommand(double &vx, double &vy, double &omega, int look_ahead_poses, double dt_ref) const
  {
    if (teb_.sizePoses() < 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
                   "TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
      vx = 0;
      vy = 0;
      omega = 0;
      return false;
    }

    look_ahead_poses = std::max(1, std::min(look_ahead_poses, (int)teb_.sizePoses() - 1));
    double dt = 0.0;
    for (int counter = 0; counter < look_ahead_poses; ++counter)
    {
      dt += teb_.TimeDiff(counter);
      if (dt >= dt_ref * look_ahead_poses) // TODO: change to look-ahead time? Refine trajectory?
      {
        look_ahead_poses = counter + 1;
        break;
      }
    }

    if (dt <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
                   "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
      vx = 0;
      vy = 0;
      omega = 0;
      return false;
    }

    // Get velocity from the first two configurations
    extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);

    return true;
  }

  void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::msg::Twist> &velocity_profile) const
  {
    int n = teb_.sizePoses();
    velocity_profile.resize(n + 1);

    // start velocity
    velocity_profile.front().linear.z = 0;
    velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
    velocity_profile.front().linear.x = vel_start_.second.linear.x;
    velocity_profile.front().linear.y = vel_start_.second.linear.y;
    velocity_profile.front().angular.z = vel_start_.second.angular.z;

    for (int i = 1; i < n; ++i)
    {
      velocity_profile[i].linear.z = 0;
      velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
      extractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
    }

    // goal velocity
    velocity_profile.back().linear.z = 0;
    velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
    velocity_profile.back().linear.x = vel_goal_.second.linear.x;
    velocity_profile.back().linear.y = vel_goal_.second.linear.y;
    velocity_profile.back().angular.z = vel_goal_.second.angular.z;
  }

  void TebOptimalPlanner::getFullTrajectory(std::vector<cohan_msgs::msg::TrajectoryPointMsg> &trajectory) const
  {
    int n = teb_.sizePoses();

    trajectory.resize(n);

    if (n == 0)
      return;

    double curr_time = 0;

    // start
    cohan_msgs::msg::TrajectoryPointMsg &start = trajectory.front();
    teb_.Pose(0).toPoseMsg(start.pose);
    start.velocity.linear.z = 0;
    start.velocity.angular.x = start.velocity.angular.y = 0;
    start.velocity.linear.x = vel_start_.second.linear.x;
    start.velocity.linear.y = vel_start_.second.linear.y;
    start.velocity.angular.z = vel_start_.second.angular.z;
    start.time_from_start.sec = static_cast<int32_t>(curr_time);
    start.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);

    curr_time += teb_.TimeDiff(0);

    // intermediate points
    for (int i = 1; i < n - 1; ++i)
    {
      cohan_msgs::msg::TrajectoryPointMsg &point = trajectory[i];
      teb_.Pose(i).toPoseMsg(point.pose);
      point.velocity.linear.z = 0;
      point.velocity.angular.x = point.velocity.angular.y = 0;
      double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
      extractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1), vel1_x, vel1_y, omega1);
      extractVelocity(teb_.Pose(i), teb_.Pose(i + 1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
      point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
      point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
      point.velocity.angular.z = 0.5 * (omega1 + omega2);
      point.time_from_start.sec = static_cast<int32_t>(curr_time);
      point.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);

      curr_time += teb_.TimeDiff(i);
    }

    // goal
    cohan_msgs::msg::TrajectoryPointMsg &goal = trajectory.back();
    teb_.BackPose().toPoseMsg(goal.pose);
    goal.velocity.linear.z = 0;
    goal.velocity.angular.x = goal.velocity.angular.y = 0;
    goal.velocity.linear.x = vel_goal_.second.linear.x;
    goal.velocity.linear.y = vel_goal_.second.linear.y;
    goal.velocity.angular.z = vel_goal_.second.angular.z;
    goal.time_from_start.sec = static_cast<int32_t>(curr_time);
    goal.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);
  }

  void TebOptimalPlanner::getFullAgentTrajectory(const uint64_t agent_id, std::vector<cohan_msgs::msg::TrajectoryPointMsg> &agent_trajectory)
  {
    auto agent_teb_it = agents_tebs_map_.find(agent_id);
    if (agent_teb_it != agents_tebs_map_.end())
    {
      auto &agent_teb = agent_teb_it->second;
      auto agent_teb_size = agent_teb.sizePoses();
      if (agent_teb_size < 3)
      {
        RCLCPP_WARN(rclcpp::get_logger("optimal_planner"),
                    "TEB size is %ld for agent %ld", agent_teb_size, agent_id);
        return;
      }

      agent_trajectory.resize(agent_teb_size);

      double curr_time = 0;

      // start
      cohan_msgs::msg::TrajectoryPointMsg &start = agent_trajectory.front();
      agent_teb.Pose(0).toPoseMsg(start.pose);
      start.velocity.linear.z = 0;
      start.velocity.angular.x = start.velocity.angular.y = 0;
      start.velocity.linear.x = agents_vel_start_[agent_id].second.linear.x;
      start.velocity.linear.y = agents_vel_start_[agent_id].second.linear.y;
      start.velocity.angular.z = agents_vel_start_[agent_id].second.angular.z;
      start.time_from_start.sec = static_cast<int32_t>(curr_time);
      start.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);

      curr_time += agent_teb.TimeDiff(0);

      // intermediate points
      for (int i = 1; i < agent_teb_size - 1; ++i)
      {
        cohan_msgs::msg::TrajectoryPointMsg &point = agent_trajectory[i];
        agent_teb.Pose(i).toPoseMsg(point.pose);
        point.velocity.linear.z = 0;
        point.velocity.angular.x = point.velocity.angular.y = 0;
        double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
        extractVelocity(agent_teb.Pose(i - 1), agent_teb.Pose(i), agent_teb.TimeDiff(i - 1), vel1_x, vel1_y, omega1);
        extractVelocity(agent_teb.Pose(i), agent_teb.Pose(i + 1), agent_teb.TimeDiff(i), vel2_x, vel2_y, omega2);
        point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
        point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
        point.velocity.angular.z = 0.5 * (omega1 + omega2);
        point.time_from_start.sec = static_cast<int32_t>(curr_time);
        point.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);

        curr_time += agent_teb.TimeDiff(i);
      }
      // goal
      cohan_msgs::msg::TrajectoryPointMsg &goal = agent_trajectory.back();

      agent_teb.BackPose().toPoseMsg(goal.pose);
      goal.velocity.linear.z = 0;
      goal.velocity.angular.x = goal.velocity.angular.y = 0;
      goal.velocity.linear.x = agents_vel_goal_[agent_id].second.linear.x;
      goal.velocity.linear.y = agents_vel_goal_[agent_id].second.linear.y;
      goal.velocity.angular.z = agents_vel_goal_[agent_id].second.angular.z;
      goal.time_from_start.sec = static_cast<int32_t>(curr_time);
      goal.time_from_start.nanosec = static_cast<uint32_t>((curr_time - start.time_from_start.sec) * 1e9);
    }
    return;
  }
  bool TebOptimalPlanner::isTrajectoryFeasible(nav2_costmap_2d::Costmap2D *costmap_model, const std::vector<geometry_msgs::msg::Point> &footprint_spec,
                                               double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
  {
    if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
      look_ahead_idx = teb().sizePoses() - 1;

    nav2_costmap_2d::FootprintCollisionChecker footprint_collision_checker(costmap_model);

    for (int i = 0; i <= look_ahead_idx; ++i)
    {
      if (footprint_collision_checker.footprintCostAtPose(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec) == -1)
      {
        if (visualization_)
        {
          visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
        }
        return false;
      }
      // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
      // and interpolates in that case.
      // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
      if (i < look_ahead_idx)
      {
        double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i + 1).theta()) -
                                                g2o::normalize_theta(teb().Pose(i).theta()));
        Eigen::Vector2d delta_dist = teb().Pose(i + 1).position() - teb().Pose(i).position();
        if (fabs(delta_rot) > min_resolution_collision_check_angular_ || delta_dist.norm() > inscribed_radius)
        {
          int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / min_resolution_collision_check_angular_),
                                              std::ceil(delta_dist.norm() / inscribed_radius)) -
                                     1;
          PoseSE2 intermediate_pose = teb().Pose(i);
          for (int step = 0; step < n_additional_samples; ++step)
          {
            intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
            intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                             delta_rot / (n_additional_samples + 1.0));
            // nav2_costmap_2d::FootprintCollisionChecker footprint_collision_checker(costmap_model);

            if (footprint_collision_checker.footprintCostAtPose(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
                                                                footprint_spec) == -1)
            {
              if (visualization_)
              {
                visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
              }
              return false;
            }
          }
        }
      }
    }
    return true;
  }

} // namespace hateb_local_planner
