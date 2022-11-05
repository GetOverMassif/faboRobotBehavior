cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")   
endif(CMAKE_COMPILER_IS_GNUCXX)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  rm_msgs
  message_generation
  roslib
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Arms.msg
)

## Generate services in the 'srv' folder
add_service_files(
 FILES
 Hand_Control.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
#    rm_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES service_demo
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../PerformModule/include
#  /usr/include/
)

#link_directories(/usr/lib/)

## Declare a C++ library
add_library(
  ArmsActionManager
  include/ArmsActionManager.h
  src/ArmsActionManager.cpp
)

add_library(
  ArmsControl
  include/ArmsControl.h
  src/ArmsControl.cpp
)

add_library(
  ArmController
  include/ArmController.h
  src/ArmController.cpp
)

target_link_libraries(ArmsActionManager jsoncpp ${catkin_LIBRARIES})

target_link_libraries(ArmsControl ArmController ArmsActionManager ${catkin_LIBRARIES})

add_executable(arm_control src/arm_control.cpp)
target_link_libraries(arm_control ArmsActionManager ArmsControl ${catkin_LIBRARIES} )
