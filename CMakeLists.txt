cmake_minimum_required(VERSION 3.5)
project(switch_controller2)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Franka 0.9.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(switch_controller2_interface REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_libraries(Franka::Franka Threads::Threads)


set(
SRC_FILES 
src/main.cpp 
src/switch_controller.cpp 
src/panda_interface.cpp
)

set(
MSG_DEPENDS
sensor_msgs 
geometry_msgs 
trajectory_msgs
switch_controller2_interface
)

add_executable(main ${SRC_FILES})
ament_target_dependencies(main rclcpp ${MSG_DEPENDS}) # std_msgs



install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})
ament_package()
