cmake_minimum_required(VERSION 2.8.3)
project(arduino_imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  arduinoImu.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(arduino_imu src/arduino_imu.cpp)
target_link_libraries(arduino_imu
  ${catkin_LIBRARIES}
)
add_dependencies(arduino_imu arduino_imu_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS arduino_imu
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


