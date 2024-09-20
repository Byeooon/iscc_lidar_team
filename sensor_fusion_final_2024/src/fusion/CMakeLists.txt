cmake_minimum_required(VERSION 3.0.2)
project(fusion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  YoloTrafficCone.msg
  clustering_points.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




