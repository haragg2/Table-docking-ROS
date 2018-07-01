cmake_minimum_required(VERSION 2.8.3)
project(aitech_task)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_geometry
  tf
  tf2
  sensor_msgs
  visualization_msgs
  move_base_msgs
  actionlib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(laser2pc src/laser2pc.cpp)
add_executable(controller src/controller.cpp)
add_executable(sendgoal src/sendgoal.cpp)

target_link_libraries(laser2pc
  ${catkin_LIBRARIES}
)

target_link_libraries(controller
  ${catkin_LIBRARIES}
)

target_link_libraries(sendgoal
  ${catkin_LIBRARIES}
)
