cmake_minimum_required(VERSION 3.8)
project(gimbal_rotation_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hnurm_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 云台旋转服务节点
add_executable(gimbal_rotation_server src/gimbal_rotation_server.cpp)
ament_target_dependencies(gimbal_rotation_server
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
)

target_include_directories(gimbal_rotation_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 模拟裁判系统节点
add_executable(referee src/referee.cpp)
ament_target_dependencies(referee
  rclcpp
  std_msgs
  hnurm_interfaces
)

target_include_directories(referee PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 姿态打印节点
add_executable(gesture_printer src/gesture_printer.cpp)
ament_target_dependencies(gesture_printer
  rclcpp
  hnurm_interfaces
)

target_include_directories(gesture_printer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# test_tf 测试节点
add_executable(test_tf src/test_tf.cpp)
ament_target_dependencies(test_tf
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_include_directories(test_tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS gimbal_rotation_server referee gesture_printer test_tf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()