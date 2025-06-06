cmake_minimum_required(VERSION 3.10)
project(hateb_local_planner)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(G2O REQUIRED)
# find_package(SUITESPARSE REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom dependencies
find_package(cohan_msgs REQUIRED)
find_package(agent_path_prediction REQUIRED)
find_package(costmap_converter REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleMsg.msg"
  "msg/OptimizationCost.msg"
  "msg/OptimizationCostArray.msg"
  "msg/TrajectoryPointMsg.msg"
  "msg/TrajectoryMsg.msg"
  "msg/FeedbackMsg.msg"
  "srv/Approach.srv"
  "srv/Optimize.srv"
  "srv/getPlanning.srv"
  "srv/setPlanning.srv"
  "srv/getOptimVel.srv"
  DEPENDENCIES geometry_msgs std_msgs costmap_converter cohan_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  # ${SUITESPARSE_INCLUDE_DIRS}
)

# External libraries
# set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${Boost_LIBRARIES})
set(EXTERNAL_LIBS ${G2O_LIBRARIES} ${Boost_LIBRARIES})

# Library
add_library(hateb_local_planner
  src/timed_elastic_band.cpp
  src/optimal_planner.cpp
  src/obstacles.cpp
  src/visualization.cpp
  src/recovery_behaviors.cpp
  src/hateb_config.cpp
  src/homotopy_class_planner.cpp
  src/hateb_local_planner_ros.cpp
  src/graph_search.cpp
  src/backoff.cpp
)

ament_target_dependencies(hateb_local_planner
  rclcpp
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  interactive_markers
  cohan_msgs
  agent_path_prediction
  costmap_converter
)

target_link_libraries(hateb_local_planner ${EXTERNAL_LIBS})

# Executables
# foreach(exec_name IN ITEMS test_optim_node static_visualize get_optim_vel local_planner)
#   add_executable(${exec_name} src/${exec_name}.cpp)
#   target_link_libraries(${exec_name} hateb_local_planner ${EXTERNAL_LIBS})
#   ament_target_dependencies(${exec_name}
#     rclcpp
#     pluginlib
#     tf2
#     tf2_ros
#     geometry_msgs
#     nav_msgs
#     std_msgs
#     visualization_msgs
#     cohan_msgs
#     agent_path_prediction
#   )
# endforeach()

add_executable(test_optim_node src/test_optim_node.cpp)
target_link_libraries(test_optim_node hateb_local_planner ${EXTERNAL_LIBS})
ament_target_dependencies(${exec_name}
  rclcpp
  pluginlib
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cohan_msgs
  agent_path_prediction
)

# Install targets
install(TARGETS
  hateb_local_planner
  test_optim_node
  static_visualize
  get_optim_vel
  local_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)

install(FILES
  hateb_local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)



install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Python scripts (if needed)
# install(PROGRAMS
#   scripts/cmd_vel_to_ackermann_drive.py
#   scripts/export_to_mat.py
#   scripts/export_to_svg.py
#   scripts/publish_dynamic_obstacle.py
#   scripts/publish_test_obstacles.py
#   scripts/publish_viapoints.py
#   scripts/visualize_velocity_profile.py
#   scripts/save_to_files.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export the pluginlib plugin
pluginlib_export_plugin_description_file(nav_core hateb_local_planner_plugin.xml)

# Package
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(hateb_local_planner)
ament_export_targets(hateb_local_planner)

ament_package()
