cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)

add_message_files(
  FILES
  Encoders.msg
  CmdDiffVel.msg
  RawImu.msg
  Drive.msg
  Feedback.msg
  BaseFeedback.msg
  BaseStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
