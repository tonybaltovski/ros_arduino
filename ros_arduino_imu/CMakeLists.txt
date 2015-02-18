cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_imu)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs ros_arduino_msgs)


find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs ros_arduino_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## Declare a cpp library
add_library(raw_imu_bridge src/raw_imu_bridge.cpp)
add_dependencies(raw_imu_bridge ros_arduino_msgs_generate_messages_cpp)
target_link_libraries(raw_imu_bridge ${catkin_LIBRARIES})

add_executable(raw_imu_bridge_node src/raw_imu_bridge_node.cpp)
add_dependencies(raw_imu_bridge_node raw_imu_bridge ros_arduino_msgs_generate_messages_cpp)
target_link_libraries(raw_imu_bridge_node raw_imu_bridge ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS raw_imu_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple_magnometer_calibration scripts/simple_magnometer_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
