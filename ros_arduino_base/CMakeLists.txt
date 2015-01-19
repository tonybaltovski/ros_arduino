cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_base)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs ros_arduino_msgs tf genmsg dynamic_reconfigure roslaunch roslint)
find_package(Boost REQUIRED COMPONENTS system thread signals)

add_service_files(
  FILES
  UpdateGains.srv
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(cfg/MotorGains.cfg)

catkin_package(
   DEPENDS
   CATKIN_DEPENDS roscpp geometry_msgs ros_arduino_msgs tf dynamic_reconfigure
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
#add_dependencies(${PROJECT_NAME} ros_arduino_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
#add_dependencies(${PROJECT_NAME}_node ros_arduino_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
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
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h") 

install(DIRECTORY launch config cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) 

file(GLOB ${PROJECT_NAME}_LINT_SRCS
  src/*.cpp)
roslint_cpp(${LINT_SRCS})
roslint_add_test()

roslaunch_add_file_check(launch/base_bringup.launch)

