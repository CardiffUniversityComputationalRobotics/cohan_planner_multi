cmake_minimum_required(VERSION 3.8)
project(cohan_layers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  angles
  cohan_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/agent_layer.cpp
  src/static_agent_layer.cpp
  src/agent_visibility_layer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
