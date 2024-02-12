// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// For parallelisation
#include <omp.h>

// include OpenCV header for recording
#include <opencv2/videoio.hpp>

// For UDP communication
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>

#include "Cv-helpers.hpp"
#include "LaneDetection.h"
#include "ObjectDetection.h"

#define COLOR_WIDTH  960
#define COLOR_HEIGHT 540

#define DEPTH_WIDTH  1280
#define DEPTH_HEIGHT 720

#define IPADDRESS "192.168.2.2" // Always use the Raspi IPv4 in both, Server and Client
#define UDP_PORT  13251         // UDP Port that is used for sending data -> Test if we can use one port/process and let socket open

using boost::asio::ip::address;
using boost::asio::ip::udp;

std::chrono::high_resolution_clock::time_point tStart;
std::chrono::high_resolution_clock::time_point tEnd;

int main() {
    rs2::context ctx;
    auto         list = ctx.query_devices();
    if (list.size() == 0) throw std::runtime_error("No RealSense device connected to the Board.");

    // Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);

    // Define the ROI
    cv::Point2f perspectiveSrc[] = {cv::Point2f(275, 180), cv::Point2f(675, 180), cv::Point2f(0, COLOR_HEIGHT), cv::Point2f(COLOR_WIDTH, COLOR_HEIGHT)};

    // Define the transformed points of the ROI
    cv::Point2f perspectiveDst[] = {cv::Point2f(0, 0), cv::Point2f(400, 0), cv::Point2f(0, 225), cv::Point2f(400, 225)};

    // Calculate the perspective Matrix
    cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);

    // Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frameset;
    for (int i = 0; i < 30; i++) {
        frameset = pipe.wait_for_frames();
    }

    // Initialize a LaneDetection Object
    rs2::frame colorFrame = pipe.wait_for_frames().get_color_frame();
    cv::Mat    initMat    = cv::Mat(cv::Size(COLOR_WIDTH, COLOR_HEIGHT), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    // Flip image around x- and y-axis because camera is upside down
    cv::flip(initMat, initMat, -1);
    LaneDetection LaneAlgo(initMat, perspectiveMatrix);

    // For Video Output
    cv::Mat testMat;

    // Initialize ObjectDetection
    ObjectDetection ObjectDetection("../model/final_model.engine");

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // UDP Socket Init and Open
    boost::asio::io_service io_service;
    udp::socket             socket(io_service);
    udp::endpoint           remote_endpoint = udp::endpoint(address::from_string(IPADDRESS), UDP_PORT); // Define socket endpoint -- Client Rasperry Pi
    socket.open(udp::v4());
    boost::system::error_code err;

    std::cout << "Initialisation of Lane and Objectdetection succesful" << std::endl;

    // Get each frame
    while (1) {
        // Start time measurement
        tStart = std::chrono::high_resolution_clock::now();

        rs2::frameset frameset = align_to_color.process(pipe.wait_for_frames());
        cv::Mat       colorMat = frame_to_mat(frameset.get_color_frame());
        cv::Mat       depthMat = depth_frame_to_meters(frameset.get_depth_frame());

        cv::flip(colorMat, colorMat, -1);
        cv::flip(depthMat, depthMat, -1);

        omp_set_num_threads(2);
#pragma omp parallel
        {
#pragma omp sections
            {
                {
                    // Detect and send signs and objects from ObjectDetection
                    auto detections = ObjectDetection.detect(colorMat, depthMat);
                    for (Detection& d : detections) {
                        socket.send_to(boost::asio::buffer(toObjectDetectionMessage(d)), remote_endpoint, 0, err);
                        // std::cout << "AFTER FILTER: Class Name: " << d.className << "\tConfidence: " << d.confidence << "\tArea: " << d.side << std::endl;
                    }
                }
#pragma omp section
                {
                    // Execute LaneDetection
                    LaneAlgo.setInputImage(colorMat);
                    LaneAlgo.laneDetectAlgo();

                    float laneDistant      = LaneAlgo.getLaneCenterDist();
                    float curveCoefficient = LaneAlgo.getCurveCoefficient();
                    int   lanePos          = LaneAlgo.getLaneCurvaturePos();
                    socket.send_to(boost::asio::buffer(LaneAlgo.toLaneDetectionMessage(laneDistant, curveCoefficient)), remote_endpoint, 0, err);
                    // std::cout << "Dist: " << laneDistant << "\tCoeff: " << curveCoefficient << "\tPos: " << lanePos << std::endl;
                }
            }
        }

        // End of time measurement
        tEnd                = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
    }

    // Close UDP Socket on exit
    socket.close();
    return 0;
}
