/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS. All rights reserved. (BSD-3, see LICENSE)
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 * ROS 2 / nav2 port.
 *********************************************************************/

#include <cohan_layers/agent_visibility_layer.h>

#include <Eigen/Core>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

using nav2_costmap_2d::NO_INFORMATION;

namespace cohan_layers
{
void AgentVisibilityLayer::onInitialize()
{
  AgentLayer::onInitialize();

  auto node = node_.lock();
  if (!node)
    throw std::runtime_error{"cohan_layers::AgentVisibilityLayer: failed to lock node"};

  declareParameter("amplitude", rclcpp::ParameterValue(250.0));
  declareParameter("radius", rclcpp::ParameterValue(1.5));

  node->get_parameter(name_ + "." + "amplitude", amplitude_);
  node->get_parameter(name_ + "." + "radius", radius_);
}

void AgentVisibilityLayer::updateBoundsFromAgents(double * min_x, double * min_y, double * max_x, double * max_y)
{
  for (const auto & agent : transformed_agents_)
  {
    *min_x = std::min(*min_x, agent.pose.position.x - radius_);
    *min_y = std::min(*min_y, agent.pose.position.y - radius_);
    *max_x = std::max(*max_x, agent.pose.position.x + radius_);
    *max_y = std::max(*max_y, agent.pose.position.y + radius_);
  }
}

void AgentVisibilityLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
                                       int max_j)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (!enabled_ || transformed_agents_.empty())
    return;

  nav2_costmap_2d::Costmap2D * costmap = &master_grid;
  double res = costmap->getResolution();

  for (const auto & agent : transformed_agents_)
  {
    if (agent.type != 1)
      continue;
    if (agent.state > 1)
      continue;

    double theta = tf2::getYaw(agent.pose.orientation);
    Eigen::Vector2d orient_vec(std::cos(theta), std::sin(theta));

    auto width = static_cast<int>(std::max(1, static_cast<int>((2 * radius_) / res)));
    auto height = width;

    double cx = agent.pose.position.x, cy = agent.pose.position.y;
    double ox = cx - radius_, oy = cy - radius_;

    int mx, my;
    costmap->worldToMapNoBounds(ox, oy, mx, my);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (mx < 0)
      start_x = -mx;
    else if (mx + width > static_cast<int>(costmap->getSizeInCellsX()))
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - mx);

    if (start_x + mx < min_i)
      start_x = min_i - mx;
    if (end_x + mx > max_i)
      end_x = max_i - mx;

    if (my < 0)
      start_y = -my;
    else if (my + height > static_cast<int>(costmap->getSizeInCellsY()))
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - my);

    if (start_y + my < min_j)
      start_y = min_j - my;
    if (end_y + my > max_j)
      end_y = max_j - my;

    double bx = ox + res / 2, by = oy + res / 2;
    double var = radius_;

    for (int cell_x = start_x; cell_x < end_x; cell_x++)
    {
      for (int cell_y = start_y; cell_y < end_y; cell_y++)
      {
        unsigned char old_cost = costmap->getCost(cell_x + mx, cell_y + my);
        if (old_cost == NO_INFORMATION)
          continue;

        double x = bx + cell_x * res, y = by + cell_y * res;
        double val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);
        double rad = sqrt(-2 * var * log(val / amplitude_));
        if (rad > radius_)
          continue;

        // Only cost the region BEHIND the person - the area they cannot see.
        Eigen::Vector2d pt_vec(x - cx, y - cy);
        if (orient_vec.dot(pt_vec) <= 0)
        {
          auto cvalue = static_cast<unsigned char>(val);
          costmap->setCost(cell_x + mx, cell_y + my, std::max(cvalue, old_cost));
        }
      }
    }
  }
}
}  // namespace cohan_layers

PLUGINLIB_EXPORT_CLASS(cohan_layers::AgentVisibilityLayer, nav2_costmap_2d::Layer)
