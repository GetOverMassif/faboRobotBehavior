cmake_minimum_required(VERSION 3.0.2)
project(PerformModule)
set(CMAKE_BUILD_TYPE "Debug")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -pthread -gdwarf-2" )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
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
    # LIBRARIES robot_platform
    # CATKIN_DEPENDS  message_runtime
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

add_executable(blueteeth_demo_node src/blueteeth_node.cpp )
target_link_libraries(blueteeth_demo_node BtManager BtRecall ${catkin_LIBRARIES}  yaml-cpp ${CMAKE_THREAD_LIBS_INIT})
