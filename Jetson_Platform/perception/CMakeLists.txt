cmake_minimum_required(VERSION 3.1.0)
project(Perception)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
add_compile_options(-std=c++17 -Wall -O2)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)   # Version 4.5.5 with CUDA=ON and CUDNN=ON
find_package(realsense2 REQUIRED)
find_package(Boost 1.40.0 REQUIRED system)     # works with up to 1.71.0
find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /usr/local/include/yolov5-tensorrt
)

set(SOURCES
    src/main.cpp
    src/LaneDetection.cpp
    src/ObjectDetection.cpp
)

add_executable(perception_process ${SOURCES})
target_link_libraries(perception_process ${realsense2_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBRARIES} Threads::Threads OpenMP::OpenMP_CXX yolov5-tensorrt opencv_dnn nvinfer nvonnxparser ${CUDA_CUDART_LIBRARY})
