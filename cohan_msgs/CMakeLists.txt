cmake_minimum_required(VERSION 3.5)
project(cohan_msgs)

# Use C++11 or later
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add message files
set(msg_files
  "msg/AgentPath.msg"
  "msg/AgentPathArray.msg"
  "msg/AgentTimeToGoal.msg"
  "msg/AgentTimeToGoalArray.msg"
  "msg/AgentTrajectory.msg"
  "msg/AgentTrajectoryArray.msg"
  "msg/AgentType.msg"
  "msg/TrackedSegment.msg"
  "msg/TrackedAgent.msg"
  "msg/TrackedAgents.msg"
  "msg/TrackedSegmentType.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryArray.msg"
  "msg/TrajectoryPoint.msg"
  "msg/TrajectoryPointMsg.msg"
  "msg/TrajectoryMsg.msg"
  "msg/StateArray.msg"
  "msg/AgentMarker.msg"
  "msg/AgentMarkerStamped.msg"
  "msg/PointArray.msg"
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# Export dependencies to downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
