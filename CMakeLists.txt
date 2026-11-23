cmake_minimum_required(VERSION 3.16)
project(helios_obstacle_detection LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Only enable when the binary runs on the build machine (vehicle PC).
option(HELIOS_NATIVE_OPTIMIZATION "Compile with -march=native" OFF)
if(HELIOS_NATIVE_OPTIMIZATION AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search kdtree sample_consensus segmentation)

# ---------------------------------------------------------------- interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/StageTiming.msg"
  "msg/ProcessingStats.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/CollisionWarning.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(${PROJECT_NAME}_cpp_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------------------------------------------------------------- algorithm library (ROS-light)
add_library(helios_obstacle_core
  src/thread_pool.cpp
  src/cloud_processor.cpp
  src/ground_segmenter.cpp
  src/czm_ground_segmenter.cpp
  src/obstacle_clusterer.cpp
  src/bounding_box.cpp
  src/ego_motion_estimator.cpp
  src/hungarian.cpp
  src/multi_object_tracker.cpp
  src/static_map.cpp
  src/collision_risk.cpp
  src/collision_warning.cpp
  src/sensor_health.cpp
  src/resource_monitor.cpp
  src/perception_pipeline.cpp
  src/performance_monitor.cpp
  src/marker_builder.cpp
)
target_include_directories(helios_obstacle_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(helios_obstacle_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(helios_obstacle_core PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  Threads::Threads
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${visualization_msgs_TARGETS}
)

# ---------------------------------------------------------------- synthetic Helios 32 simulator
# Used by the tests and the benchmark (ground truth for accuracy metrics).
add_library(helios_sim src/lidar_simulator.cpp src/tracking_metrics.cpp)
target_link_libraries(helios_sim PUBLIC helios_obstacle_core)

# ---------------------------------------------------------------- node
add_executable(obstacle_detector src/detection_node.cpp)
target_link_libraries(obstacle_detector
  helios_obstacle_core
  rclcpp::rclcpp
  ${diagnostic_updater_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
  "${${PROJECT_NAME}_cpp_typesupport}"
)
# pcl_conversions on Jazzy may not export targets; fall back to its include dirs.
if(NOT pcl_conversions_TARGETS)
  target_include_directories(obstacle_detector PRIVATE ${pcl_conversions_INCLUDE_DIRS})
endif()

# ---------------------------------------------------------------- offline benchmark
add_executable(helios_benchmark src/benchmark_main.cpp)
target_link_libraries(helios_benchmark
  helios_sim
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
)

install(TARGETS helios_obstacle_core helios_sim obstacle_detector helios_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/obstacle_report.py
  scripts/bag_tool.py
  scripts/perf_report.py
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------- tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  foreach(t test_pipeline test_tracking test_ego_motion test_ground_czm test_risk_health
      test_static_map test_system test_tracking_v03 test_collision_warning)
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 300)
    if(TARGET ${t})
      target_link_libraries(${t} helios_sim)
    endif()
  endforeach()

  # ROS graph level: node starts, consumes synthetic clouds, publishes tracks + diagnostics,
  # and never advertises actuation topics.
  add_launch_test(test/test_node_launch.py TIMEOUT 120)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
