cmake_minimum_required(VERSION 3.0.2)
project(livelybot_gazebo_motor_controller)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    livelybot_msg
)

catkin_package(
    CATKIN_DEPENDS
    livelybot_msg
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
)
file(GLOB_RECURSE SRC_LIST
"src/*.cpp"
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library( livelybot_gazebo_motor_control 
    src/motor_controller.cpp
    ${SRC_LIST}
)
add_dependencies(livelybot_gazebo_motor_control livelybot_msg_gencpp)