/**
 * @file camera_perception_node.cpp
 * @author Constantin Blessing, Jakob Haeringer, Noah Koehler
 * @brief This file implements the ROS node for the perception of the vehicle implemented in ROS.
 * @version 1.0
 * @date 2023-08-16
 */
#include "bfmc_interface/LaneDetection.h"
#include "bfmc_interface/SignDetection.h"
#include "camera_perception/Camera.hpp"
#include "camera_perception/LaneDetection.hpp"
#include "camera_perception/SignDetection.hpp"
#include <ros/package.h>
#include <ros/ros.h>

Camera createCamera(ros::NodeHandle& nodeHandle) {
    return Camera(nodeHandle, {960, 540}, {1280, 720});
}

LaneDetection createLaneDetection(Camera& camera) {
    // Define the region of interest.
    cv::Point2f perspectiveSource[] = {cv::Point2f(275, 180), cv::Point2f(675, 180), cv::Point2f(0, camera.getColorFrameSize().height), cv::Point2f(camera.getColorFrameSize().width, camera.getColorFrameSize().height)};
    // Define the transformed points of the region of interest.
    cv::Point2f perspectiveDestination[] = {cv::Point2f(0, 0), cv::Point2f(400, 0), cv::Point2f(0, 225), cv::Point2f(400, 225)};
    // Calculate the perspective Matrix.
    cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(perspectiveSource, perspectiveDestination);

    camera.waitForNewFrames();

    return LaneDetection(camera.getLastColorFrame(), perspectiveMatrix);
}

SignDetection createSignDetection() {
    return SignDetection(ros::package::getPath("bfmc_input") + "/config/camera_perception/yolov5.engine");
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "camera_perception");
    ros::NodeHandle               nodeHandle;
    ros::Publisher                laneDetectionPublisher = nodeHandle.advertise<bfmc_interface::LaneDetection>("/input/camera/lane_detection", 1);
    ros::Publisher                signDetectionPublisher = nodeHandle.advertise<bfmc_interface::SignDetection>("/input/camera/sign_detection", 5);
    bfmc_interface::LaneDetection laneDetectionMsg;
    bfmc_interface::SignDetection signDetectionMsg;

    auto camera        = createCamera(nodeHandle);
    auto laneDetection = createLaneDetection(camera);
    auto signDetection = createSignDetection();

    std::cout << "Initialization of lane/object detection succeeded." << std::endl;

    while (ros::ok()) {
        camera.waitForNewFrames();
#pragma omp parallel sections num_threads(2)
        {
#pragma omp section
            {
                // Run lane detection.
                laneDetection.setInputImage(camera.getLastColorFrame());
                laneDetection.laneDetectAlgo();

                laneDetectionMsg.header.stamp     = ros::Time::now();
                laneDetectionMsg.midDistance      = laneDetection.getLaneCenterDist();
                laneDetectionMsg.curveCoefficient = laneDetection.getCurveCoefficient();
                laneDetectionPublisher.publish(laneDetectionMsg);
            }
#pragma omp section
            {
                // Detect and send signs/objects from sign detection.
                for (const auto& detection : signDetection.detect(camera.getLastColorFrame(), camera.getLastDepthFrame())) {
                    signDetectionMsg.header.stamp        = ros::Time::now();
                    signDetectionMsg.sign_class          = detection.className;
                    signDetectionMsg.x_bounding_box      = detection.x;
                    signDetectionMsg.y_bounding_box      = detection.y;
                    signDetectionMsg.width_bounding_box  = detection.w;
                    signDetectionMsg.height_bounding_box = detection.h;
                    signDetectionMsg.confidence          = detection.confidence;
                    signDetectionMsg.distance            = detection.distance;
                    signDetectionMsg.area                = detection.side;
                    signDetectionPublisher.publish(signDetectionMsg);
                }
            }
        }
    }
    return 0;
}
