/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS. All rights reserved. (BSD-3, see LICENSE)
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 * ROS 2 / nav2 port.
 *********************************************************************/

#ifndef AGENT_VISIBILITY_LAYER_H
#define AGENT_VISIBILITY_LAYER_H

#include <cohan_layers/agent_layer.h>

namespace cohan_layers
{
class AgentVisibilityLayer : public AgentLayer
{
public:
  AgentVisibilityLayer() {}

  void onInitialize() override;
  void updateBoundsFromAgents(double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
};
}  // namespace cohan_layers

#endif  // AGENT_VISIBILITY_LAYER_H
