/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS. All rights reserved. (BSD-3, see LICENSE)
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 * ROS 2 / nav2 port.
 *********************************************************************/

#include <cohan_layers/static_agent_layer.h>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cohan_layers
{
void StaticAgentLayer::onInitialize()
{
  AgentLayer::onInitialize();

  auto node = node_.lock();
  if (!node)
    throw std::runtime_error{"cohan_layers::StaticAgentLayer: failed to lock node"};

  declareParameter("amplitude", rclcpp::ParameterValue(150.0));
  declareParameter("radius", rclcpp::ParameterValue(1.5));
  // Radius of the solid LETHAL_OBSTACLE core painted around every agent.
  // This is the part the local planner actually reacts to:
  // updateObstacleContainerWithCostmap() only turns cells whose cost is
  // EXACTLY LETHAL_OBSTACLE (254) into planner obstacles, so a Gaussian that
  // peaks below 254 is visible in RViz and nowhere else. Set <= 0 to disable
  // the core and get upstream's gradient-only behaviour.
  declareParameter("lethal_radius", rclcpp::ParameterValue(0.3));

  node->get_parameter(name_ + "." + "amplitude", amplitude_);
  node->get_parameter(name_ + "." + "radius", radius_);
  node->get_parameter(name_ + "." + "lethal_radius", lethal_radius_);
}

void StaticAgentLayer::updateBoundsFromAgents(double * min_x, double * min_y, double * max_x, double * max_y)
{
  for (const auto & agent : transformed_agents_)
  {
    *min_x = std::min(*min_x, agent.pose.position.x - radius_);
    *min_y = std::min(*min_y, agent.pose.position.y - radius_);
    *max_x = std::max(*max_x, agent.pose.position.x + radius_);
    *max_y = std::max(*max_y, agent.pose.position.y + radius_);
  }
}

void StaticAgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
                                   int max_j)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (!enabled_ || transformed_agents_.empty())
    return;

  nav2_costmap_2d::Costmap2D * costmap = &master_grid;
  double res = costmap->getResolution();

  for (const auto & agent : transformed_agents_)
  {
    // A moving, tracked person gets the soft Gaussian social cost; anyone else
    // (standing still, or not classified) gets a solid lethal disc so the robot
    // cannot plan through them.
    bool soft_gaussian = (agent.state >= 0) ? (agent.state < 2 && agent.type == 1) : (agent.type != 0);

    double blob_radius = soft_gaussian ? radius_
                                       : 1.5 * ((agent.type == 1) ? agent_radius_ : robot_radius_);
    // The blob must cover the lethal core even when the Gaussian is narrow.
    blob_radius = std::max(blob_radius, lethal_radius_);

    auto width = static_cast<int>(std::max(1, static_cast<int>((2 * blob_radius) / res)));
    auto height = width;

    double cx = agent.pose.position.x, cy = agent.pose.position.y;
    double ox = cx - blob_radius, oy = cy - blob_radius;

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
    double var = blob_radius;

    // cx/cy/agent.* are read here, never the cell counters - upstream shadowed
    // the outer agent index with these loop variables and then indexed
    // agents_.agents[] with a cell number.
    for (int cell_x = start_x; cell_x < end_x; cell_x++)
    {
      for (int cell_y = start_y; cell_y < end_y; cell_y++)
      {
        unsigned char old_cost = costmap->getCost(cell_x + mx, cell_y + my);
        if (old_cost == NO_INFORMATION)
          continue;

        // Solid lethal core first, regardless of soft/hard classification. This
        // is what the local planner picks up; the gradient outside it only
        // shapes the inflation layer and the RViz view.
        if (lethal_radius_ > 0.0)
        {
          double wx, wy;
          costmap->mapToWorld(cell_x + mx, cell_y + my, wx, wy);
          double ddx = wx - cx, ddy = wy - cy;
          if (std::sqrt(ddx * ddx + ddy * ddy) <= lethal_radius_)
          {
            costmap->setCost(cell_x + mx, cell_y + my, LETHAL_OBSTACLE);
            continue;
          }
        }

        unsigned char cvalue;
        if (soft_gaussian)
        {
          double x = bx + cell_x * res, y = by + cell_y * res;
          double val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);
          double rad = sqrt(-2 * var * log(val / amplitude_));
          if (rad > blob_radius)
            continue;
          cvalue = static_cast<unsigned char>(val);
        }
        else
        {
          double wx, wy;
          costmap->mapToWorld(cell_x + mx, cell_y + my, wx, wy);
          double dx = wx - cx, dy = wy - cy;
          if (std::sqrt(dx * dx + dy * dy) > blob_radius)
            continue;
          cvalue = LETHAL_OBSTACLE;
        }

        costmap->setCost(cell_x + mx, cell_y + my, std::max(cvalue, old_cost));
      }
    }
  }
}
}  // namespace cohan_layers

PLUGINLIB_EXPORT_CLASS(cohan_layers::StaticAgentLayer, nav2_costmap_2d::Layer)
