cmake_minimum_required(VERSION 3.15)
project(legged_admm_fleet)

find_package(ament_cmake REQUIRED)
find_package(osqp REQUIRED CONFIG)   # ros-jazzy-osqp-vendor (OSQP 0.6.2; source pinned 0.6.3, same 0.6.x API)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED CONFIG)

# ── ADMM C++ core: ported verbatim from legged_upper_control_pkg (include prefix kept) ──
add_library(admm_core STATIC
  src/admm_core/constants.cpp
  src/admm_core/qp_common.cpp
  src/admm_core/rti.cpp
  src/admm_core/node_qp.cpp
  src/admm_core/edge_qp.cpp
  src/admm_core/formation.cpp
  src/admm_core/motion_adapter.cpp
  src/admm_core/coordinator.cpp
  src/admm_core/reference.cpp
  src/admm_core/astar_planner.cpp
  src/admm_core/fleet_config.cpp
  src/agent_core.cpp
)
target_compile_features(admm_core PUBLIC cxx_std_17)
target_include_directories(admm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(admm_core PUBLIC osqp::osqp Eigen3::Eigen)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  target_link_libraries(admm_core PUBLIC OpenMP::OpenMP_CXX)
endif()
set_target_properties(admm_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── pybind module: the Python test oracle drives the C++ core through this ──
pybind11_add_module(admm_core_cpp src/admm_core/bindings.cpp)
target_link_libraries(admm_core_cpp PRIVATE admm_core)

install(TARGETS admm_core_cpp
  LIBRARY DESTINATION lib/${PROJECT_NAME}/python)
install(DIRECTORY python/admm
  DESTINATION lib/${PROJECT_NAME}/python)
install(DIRECTORY include/ DESTINATION include)

# ── centralized fleet node (rclcpp port of ocs2_fleet_publisher_node) ──
find_package(rclcpp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)  # ocs2_core 的 export 引用 Boost:: targets 但未 find_dependency
find_package(ocs2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(fleet_centralized_node src/fleet_centralized_node.cpp)
target_link_libraries(fleet_centralized_node admm_core)
ament_target_dependencies(fleet_centralized_node
  rclcpp ocs2_msgs ocs2_core nav_msgs geometry_msgs visualization_msgs)
install(TARGETS fleet_centralized_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# ── distributed per-robot node (G4): one process per dog, ADMM over DdsTransport ──
find_package(admm_fleet_msgs REQUIRED)
find_package(Threads REQUIRED)
add_executable(admm_agent_node src/admm_agent_node.cpp)
target_link_libraries(admm_agent_node admm_core Threads::Threads)
ament_target_dependencies(admm_agent_node
  rclcpp ocs2_msgs ocs2_core nav_msgs geometry_msgs admm_fleet_msgs)
install(TARGETS admm_agent_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# ── standalone formation slot allocator: /formation/goal -> /formation/plan (no dog is special) ──
add_executable(fleet_coordinator_node src/fleet_coordinator_node.cpp)
target_link_libraries(fleet_coordinator_node admm_core)
ament_target_dependencies(fleet_coordinator_node
  rclcpp nav_msgs geometry_msgs admm_fleet_msgs)
install(TARGETS fleet_coordinator_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config worlds rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/start_gaits.sh
        scripts/fleet_viz_markers.py scripts/gen_arena_world.py
        DESTINATION lib/${PROJECT_NAME})

# ── G1 parity gate: distributed AgentCore vs centralized coordinator (bit-identical) ──
find_package(Threads REQUIRED)
add_executable(test_distributed_parity test/test_distributed_parity.cpp)
target_link_libraries(test_distributed_parity admm_core Threads::Threads)
add_executable(test_agent_timeout test/test_agent_timeout.cpp)
target_link_libraries(test_agent_timeout admm_core Threads::Threads)

ament_package()
