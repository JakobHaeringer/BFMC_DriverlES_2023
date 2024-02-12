/**
 * @file Camera.hpp
 * @author Constantin Blessing
 * @brief The header file of the perception process implemented in ROS.
 * @version 1.0
 * @date 2023-08-16
 */
#pragma once

#ifndef ITMOVES_SIMULATOR
    #include <librealsense2/rs.hpp>
#else
    #include "bfmc_interface/SimulatorCamera.h"
#endif

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <ros/ros.h>
class Camera {
public:
    Camera(ros::NodeHandle& nodeHandle, cv::Size colorFrameSize, cv::Size depthFrameSize);

    void            waitForNewFrames();
    const cv::Size& getColorFrameSize() const;
    const cv::Size& getDepthFrameSize() const;
    const cv::Mat&  getLastColorFrame() const;
    const cv::Mat&  getLastDepthFrame() const;

private:
    cv::Size colorFrameSize;
    cv::Size depthFrameSize;
    cv::Mat  lastColorFrame;
    cv::Mat  lastDepthFrame;

#ifndef ITMOVES_SIMULATOR
    rs2::pipeline pipeline;
    rs2::config   config;
#else
    bool            newFramesReceived;
    ros::Subscriber simulatedCameraSubscriber;
#endif

#ifndef ITMOVES_SIMULATOR
    /**
     * @brief Turns a realsense frame object into a OpenCV matrix.
     *
     * @param f The realsense frame that should be transformed.
     * @return cv::Mat The resulting OpenCV matrix object.
     */
    static cv::Mat FrameToMatrix(const rs2::frame& Frame);
#else
    // clang-format off
    void simulatedCameraCallback(const bfmc_interface::SimulatorCameraConstPtr& msg);
    // clang-format on
#endif
};