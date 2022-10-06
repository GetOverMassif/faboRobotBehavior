cmake_minimum_required(VERSION 3.0.2)
project(PerformModule)
set(CMAKE_BUILD_TYPE "Debug")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -pthread -gdwarf-2" )

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  HeadMotion_msg.msg
  WheelMotion_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
#    rm_msgs
)

MESSAGE("****catkin dir :${catkin_INCLUDE_DIRS} ")
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /home/lj/Documents/Indoor-mobile-robot-with-arms/devel/include
)

find_package(Threads) 

catkin_package(
    LIBRARIES bluetooth_demo 
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

add_library(
    BtManager
    include/BtManager.h
    src/BtManager.cpp
)

add_library(
    BtRecall
    include/BtRecall.h
    src/BtRecall.cpp
)

add_executable(blueteeth_demo_node src/blueteeth_node.cpp)
target_link_libraries(blueteeth_demo_node BtManager BtRecall ${catkin_LIBRARIES}  yaml-cpp ${CMAKE_THREAD_LIBS_INIT})

add_executable(fabo_teleop_key src/fabo_teleop_key.cpp)
target_link_libraries(fabo_teleop_key ${catkin_LIBRARIES}  yaml-cpp ${CMAKE_THREAD_LIBS_INIT})