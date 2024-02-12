cmake_minimum_required(VERSION 3.0.2)
project(bfmc_action)

add_compile_options(-std=c++17 -Wall -O2)

set(CATKIN_DEPENDENCIES
    roscpp
    roslib
    std_msgs
    bfmc_interface
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
set(target "planner_node")

add_executable("${target}"
    src/planner/planner_node.cpp
    src/planner/Environment.cpp
    src/planner/Command.cpp
    src/planner/Actions.cpp
    src/planner/AStar.cpp
    src/planner/AStarReadFile.cpp
)

target_link_libraries("${target}" ${catkin_LIBRARIES})
