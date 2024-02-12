cmake_minimum_required(VERSION 3.0.2)
project(bfmc_interface)

add_compile_options(-std=c++17 -Wall -O2)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    sensor_msgs
    geometry_msgs
    std_msgs
)

# Generate messages in the 'msg' folder.
add_message_files(DIRECTORY msg)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

# Specify additional locations of header files.
# Your package locations should be listed before other locations.
include_directories(${catkin_INCLUDE_DIRS})
