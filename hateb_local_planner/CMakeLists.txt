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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
# find_package(G2O REQUIRED)
# find_package(SuiteSparse REQUIRED)

# Custom dependencies
find_package(cohan_msgs REQUIRED)
# find_package(agent_path_prediction REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)

if(NOT G2O_INCLUDE_DIRS)
  set(G2O_INCLUDE_DIRS /usr/include/g2o)
endif()
if(NOT G2O_LIBRARIES)
  set(G2O_LIBRARIES
    /home/sasm/g2o/build/lib/libg2o_core.so
    /home/sasm/g2o/build/lib/libg2o_stuff.so
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
  std_msgs
  std_srvs
  visualization_msgs
  interactive_markers
  cohan_msgs
  # agent_path_prediction
  costmap_converter
  pedsim_msgs
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
  # src/recovery_behaviors.cpp
  # src/hateb_config.cpp
  # src/homotopy_class_planner.cpp
  # src/graph_search.cpp
  # src/backoff.cpp
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
     )

# Install targets
install(TARGETS
  hateb_local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(test_optim_node src/test_optim_node.cpp)
# target_link_libraries(test_optim_node hateb_local_planner ${EXTERNAL_LIBS})
# ament_target_dependencies(${exec_name}
#   rclcpp
#   pluginlib
#   tf2
#   tf2_ros
#   geometry_msgs
#   nav_msgs
#   std_msgs
#   visualization_msgs
#   cohan_msgs
#   # agent_path_prediction
# )



# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/
# )


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

ament_package()
