cmake_minimum_required(VERSION 3.0.2)
project(bfmc_output)

add_compile_options(-std=c++17 -Wall -O2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    bfmc_interface)
catkin_package(CATKIN_DEPENDS)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(serial_handler_node
    src/serial_handler/serial_handler_node.cpp
    src/serial_handler/Message.cpp
    src/serial_handler/ResponseHandler.cpp
    src/serial_handler/SerialComManager.cpp
    src/serial_handler/SerialPortHandler.cpp
)

target_link_libraries(serial_handler_node ${catkin_LIBRARIES})
