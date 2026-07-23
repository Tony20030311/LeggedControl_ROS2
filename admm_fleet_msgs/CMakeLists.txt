cmake_minimum_required(VERSION 3.16)
project(admm_fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentState.msg"
  "msg/EdgeXi.msg"
  "msg/EdgeZ.msg"
  "msg/FleetPlan.msg"
  "msg/CycleStats.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
