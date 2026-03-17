cmake_minimum_required(VERSION 3.8)
project(hnurm_decision_gesture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hnurm_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Define source files
set(PLUGIN_SOURCES
  plugins/action/PubRobotStatus.cpp
  plugins/condition/GameStart.cpp
  plugins/condition/Status.cpp
)

# Create executable
add_executable(decision_node 
  src/decision.cpp
  ${PLUGIN_SOURCES}
)

# Set target dependencies
ament_target_dependencies(decision_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  ament_index_cpp
  hnurm_interfaces
  visualization_msgs
  std_srvs
)

# Install executable
install(TARGETS decision_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install param files
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
