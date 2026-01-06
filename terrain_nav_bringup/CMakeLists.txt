cmake_minimum_required(VERSION 3.8)
project(terrain_nav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pcd_to_gridmap_node src/pcd_to_gridmap_node.cpp)
target_include_directories(pcd_to_gridmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pcd_to_gridmap_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  grid_map_ros
  grid_map_msgs
  grid_map_pcl
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pcd_to_gridmap_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/simple_mbf_goal_bridge.py scripts/grid_map_to_mesh.py scripts/pcd_publisher.py scripts/simple_localization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
