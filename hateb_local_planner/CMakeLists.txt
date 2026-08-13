cmake_minimum_required(VERSION 3.8)
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
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(esc_move_base_msgs REQUIRED)
find_package(tidup_move_base_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
# find_package(G2O REQUIRED)
# find_package(SuiteSparse REQUIRED)

# Custom dependencies
find_package(cohan_msgs REQUIRED)
# find_package(agent_path_prediction REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)

if(NOT G2O_INCLUDE_DIRS)
  set(G2O_INCLUDE_DIRS /opt/ros/humble/include/g2o)
endif()
if(NOT G2O_LIBRARIES)
  set(G2O_LIBRARIES
    /home/$ENV{USER}/ros/$ENV{ROS_DISTRO}/system/src/g2o/build/lib/libg2o_core.so
    /home/$ENV{USER}/ros/$ENV{ROS_DISTRO}/system/src/g2o/build/lib/libg2o_stuff.so
    /home/$ENV{USER}/ros/$ENV{ROS_DISTRO}/system/src/g2o/build/lib/libg2o_solver_cholmod.so
    # add others if needed
  )
endif()

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})


# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  /usr/include/suitesparse 
  SYSTEM
)

# Set dependencies for ament_target_dependencies
set(dependencies
  rclcpp
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  std_srvs
  visualization_msgs
  interactive_markers
  cohan_msgs
  # agent_path_prediction
  costmap_converter
  pedsim_msgs
  nav2_costmap_2d
  esc_move_base_msgs
  tidup_move_base_msgs
)

# External libraries
# set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${Boost_LIBRARIES})
set(EXTERNAL_LIBS ${G2O_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hateb_local_planner_node
  src/hateb_local_planner.cpp
  src/timed_elastic_band.cpp
  src/optimal_planner.cpp
  src/obstacles.cpp
  src/visualization.cpp
  src/recovery_behaviors.cpp
  src/homotopy_class_planner.cpp
  src/graph_search.cpp
  # src/hateb_config.cpp   - replaced by include/hateb_params.h
  # src/backoff.cpp        - backoff recovery deliberately not ported
)

ament_target_dependencies(hateb_local_planner_node ${dependencies})
target_link_libraries(hateb_local_planner_node
     ${EXTERNAL_LIBS}
    #  -lcsparse
    cholmod
    blas
    lapack
    amd
    colamd
    camd
    ccolamd
    fmt
  )

# Install targets
install(TARGETS
  hateb_local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Header-only sanity check on hateb_params.h. No gtest, no ROS.
  add_executable(test_hateb_params test/params_sanity.cpp)
  add_test(NAME hateb_params_sanity COMMAND test_hateb_params)
endif()

ament_package()
