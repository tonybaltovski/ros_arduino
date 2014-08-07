cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_base)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs ros_arduino_msgs tf roslaunch roslint)
find_package(Boost REQUIRED COMPONENTS system thread signals)


catkin_package(
	 DEPENDS
	 CATKIN_DEPENDS roscpp geometry_msgs tf ros_arduino_msgs
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
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
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) 

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) 

roslint_cpp()
roslint_add_test()
roslaunch_add_file_check(launch/base_bringup.launch)

