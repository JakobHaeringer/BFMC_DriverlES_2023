cmake_minimum_required(VERSION 3.0.2)
project(bfmc_input)
add_compile_options(-std=c++17 -Wall -O2)

find_package(Boost 1.40.0 REQUIRED system)
find_package(Threads REQUIRED)

set(CATKIN_DEPENDENCIES
    cv_bridge
    roscpp
    roslib
    rospy
    std_msgs
    bfmc_interface
)

# The outermost if condition is only temporary.
# The if condition can be omitted, once the camera perception code is executed as part of the ROS network during real-world operation.
if(DEFINED ITMOVES_SIMULATOR)
    if(NOT DEFINED ITMOVES_SIMULATOR)
        find_package(realsense2 REQUIRED)
    endif()

    find_package(CUDA REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(OpenMP REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

set(PACKAGE_DEPENDENCIES Boost)

# The outermost if condition is only temporary.
if(DEFINED ITMOVES_SIMULATOR)
    if(NOT DEFINED ITMOVES_SIMULATOR)
        list(APPEND PACKAGE_DEPENDENCIES realsense2)
    endif()

    list(APPEND PACKAGE_DEPENDENCIES CUDA OpenCV)
endif()

catkin_package()

include_directories(
    "include/"
    ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

# #####################################################################################################
# Wheel Speed Node                                                                                    #
# #####################################################################################################
add_executable(wheel_speed_node "src/wheel_speed/wheel_speed_node.cpp")

if(NOT DEFINED ITMOVES_SIMULATOR)
    target_link_libraries(wheel_speed_node
        -lrt
        -lwiringPi
    )
endif()

# #####################################################################################################
# Camera Perection Listener Node                                                                      #
# #####################################################################################################

# This node doesn't do anything when compiled inside the simulator but is still included because the simulator
# uses the existing run_car_autonomous.launch file which expects this node to be present.
add_executable(camera_perception_listener_node "src/camera_perception/camera_listener_node.cpp")

target_link_libraries(camera_perception_listener_node
    ${Boost_LIBRARIES}
    Threads::Threads
)

# #####################################################################################################
# Camera Perception Node                                                                              #
# #####################################################################################################

# The outermost if condition is only temporary.
# Currently this node is only meant to run inside the simulator because the actual vehicle hardware
# runs the camera perception code on a dedicated piece of hardware which is not part of the ROS network.
if(DEFINED ITMOVES_SIMULATOR)
    add_executable(camera_perception_node
        "src/camera_perception/camera_perception_node.cpp"
        "src/camera_perception/Camera.cpp"
        "src/camera_perception/LaneDetection.cpp"
        "src/camera_perception/SignDetection.cpp"
    )

    target_include_directories(camera_perception_node PRIVATE
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        "/usr/local/include/yolov5-tensorrt/"
    )

    target_link_libraries(camera_perception_node
        opencv_dnn
        OpenMP::OpenMP_CXX
        Threads::Threads
        yolov5-tensorrt
        nvinfer
        nvonnxparser
        ${CUDA_CUDART_LIBRARY}
        ${OpenCV_LIBRARIES}
    )

    if(NOT DEFINED ITMOVES_SIMULATOR)
        target_link_libraries(camera_perception_node ${realsense2_LIBRARY})
    endif()

    # The camera perception node uses YOLOv5 to detect signs. YOLOv5 requires an
    # .engine file describing the internal neural network used. This .engine file is
    # hardware dependent and is compiled as part of the build process below.
    set(YOLOV5_INPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/camera_perception/yolov5.onnx")
    set(YOLOV5_OUTPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/camera_perception/yolov5.engine")

    # This is only executed when the dependencies specified changed or the output file is not present.
    add_custom_command(
        MAIN_DEPENDENCY "${YOLOV5_INPUT_FILE}"
        OUTPUT "${YOLOV5_OUTPUT_FILE}"
        COMMAND bash -c "build_engine --model ${YOLOV5_INPUT_FILE} --output ${YOLOV5_OUTPUT_FILE}"
    )
    add_custom_target(yolov5_engine DEPENDS "${YOLOV5_OUTPUT_FILE}")
    add_dependencies(camera_perception_node yolov5_engine)
endif()
