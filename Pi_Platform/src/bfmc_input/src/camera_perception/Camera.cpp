/**
 * @file Camera.cpp
 * @author Constantin Blessing
 * @brief This file implements the main process for the perception of the vehicle.
 * @version 1.0
 * @date 2023-08-16
 */
#include "camera_perception/Camera.hpp"
#include <cv_bridge/cv_bridge.h>

Camera::Camera(ros::NodeHandle& nodeHandle, cv::Size colorFrameSize, cv::Size depthFrameSize) :
    colorFrameSize(colorFrameSize),
    depthFrameSize(depthFrameSize) {
#ifdef ITMOVES_SIMULATOR
    newFramesReceived         = false;
    simulatedCameraSubscriber = nodeHandle.subscribe<bfmc_interface::SimulatorCamera>("/simulator/camera", 1, &Camera::simulatedCameraCallback, this);
#else
    rs2::context ctx;

    if (ctx.query_devices().size() == 0) {
        throw std::runtime_error("No Intel RealSense camera connected to the board.");
    }

    // Add desired streams to configuration.
    config.enable_stream(RS2_STREAM_COLOR, colorFrameSize.width, colorFrameSize.height, RS2_FORMAT_RGB8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, depthFrameSize.width, depthFrameSize.height, RS2_FORMAT_Z16, 30);
    // Instruct pipeline to start streaming with the requested configuration.
    pipeline.start(config);

    // Camera warmup - dropping several initial frames to let auto-exposure stabilize.
    for (auto iteration = 0; iteration < 30; iteration++) {
        pipeline.wait_for_frames();
    }
#endif
}

void Camera::waitForNewFrames() {
#ifdef ITMOVES_SIMULATOR
    // Spinning like that might be unnecessarily taxing on performance?
    do {
        ros::spinOnce();
    }
    while (!newFramesReceived);

    newFramesReceived = false;

#else
    static rs2::align alignToColor(RS2_STREAM_COLOR);

    auto frameset  = alignToColor.process(pipeline.wait_for_frames());
    lastColorFrame = FrameToMatrix(frameset.get_color_frame());
    // Convert RealSense depth frame to an OpenCV matrix of doubles with distances in [m].
    lastDepthFrame = FrameToMatrix(frameset.get_depth_frame());
    lastDepthFrame.convertTo(lastDepthFrame, CV_64F);
    lastDepthFrame = lastDepthFrame * frameset.get_depth_frame().get_units();
#endif
}

const cv::Mat& Camera::getLastColorFrame() const {
    return lastColorFrame;
}

const cv::Mat& Camera::getLastDepthFrame() const {
    return lastDepthFrame;
}

const cv::Size& Camera::getColorFrameSize() const {
    return colorFrameSize;
}

const cv::Size& Camera::getDepthFrameSize() const {
    return depthFrameSize;
}

#ifdef ITMOVES_SIMULATOR
void Camera::simulatedCameraCallback(const bfmc_interface::SimulatorCameraConstPtr& msg) {
    // Convert and assign frames. See http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages for more info.
    cv::resize(cv_bridge::toCvCopy(msg->color)->image, lastColorFrame, colorFrameSize);
    cv::resize(cv_bridge::toCvCopy(msg->depth)->image, lastDepthFrame, colorFrameSize);
    newFramesReceived = true;
}
#else
cv::Mat Camera::FrameToMatrix(const rs2::frame& frame) {
    using namespace cv;
    using namespace rs2;

    auto videoFrame = frame.as<video_frame>();
    auto width      = videoFrame.get_width();
    auto height     = videoFrame.get_height();

    Mat  matrix;
    auto rgbMatrix = Mat(Size(width, height), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP);

    switch (frame.get_profile().format()) {
    case RS2_FORMAT_BGR8:
        matrix = Mat(Size(width, height), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP);
        break;
    case RS2_FORMAT_RGB8:
        cvtColor(rgbMatrix, matrix, COLOR_RGB2BGR);
        break;
    case RS2_FORMAT_Z16:
        matrix = Mat(Size(width, height), CV_16UC1, (void*)frame.get_data(), Mat::AUTO_STEP);
        break;
    case RS2_FORMAT_Y8:
        matrix = Mat(Size(width, height), CV_8UC1, (void*)frame.get_data(), Mat::AUTO_STEP);
        break;
    case RS2_FORMAT_DISPARITY32:
        matrix = Mat(Size(width, height), CV_32FC1, (void*)frame.get_data(), Mat::AUTO_STEP);
        break;
    default:
        throw std::runtime_error("Frame format is not supported yet!");
        break;
    }

    // Flip image around x/y-axis because camera is mounted upside down.
    flip(matrix, matrix, -1);
    return matrix;
}
#endif