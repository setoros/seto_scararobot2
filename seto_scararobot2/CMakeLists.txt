################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(seto_scararobot2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(libcreate REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

#add_library(${PROJECT_NAME}_lib
#  "src/move_test1.cpp"
#  "src/move_test1_1.cpp"
#)

#target_link_libraries(${PROJECT_NAME}_lib)
#ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(move_test1 src/move_test1.cpp)
#target_link_libraries(move_arm ${PROJECT_NAME}_lib)
ament_target_dependencies(move_test1
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

add_executable(move_test1_1 src/move_test1_1.cpp)
#target_link_libraries(move_arm ${PROJECT_NAME}_lib)
ament_target_dependencies(move_test1_1
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

add_executable(move_arm src/move_arm.cpp)
#target_link_libraries(move_arm ${PROJECT_NAME}_lib)
ament_target_dependencies(move_arm
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

add_executable(seto_scararobot2 src/seto_scararobot2.cpp)
#target_link_libraries(seto_scararobot2 ${PROJECT_NAME}_lib)
ament_target_dependencies(seto_scararobot2
  "geometry_msgs"
  "libcreate"
  "rclcpp"
  "rclcpp_lifecycle"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
)

################################################################################
# Install
################################################################################

install(TARGETS 
  move_test1
  move_test1_1
  move_arm
  seto_scararobot2
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
