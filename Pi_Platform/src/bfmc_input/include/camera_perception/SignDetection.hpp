/**
 * @file SignDetection.hpp
 * @author Julius Baechle, Kadir Oezer, Noah Koehler
 * @brief The header file of the sign detection used in the BFMC 2023.
 * @version 1.0
 * @date 2023-06-15
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <yolov5_detector.hpp>

/** @brief This enum defines the ROI in which a camera frame is divided into, to obtain a pseudo object tracking. */
enum Side { Left,
            Middle,
            Right };

/** @brief Holds all the attributes which are relevant for a detected object. */
struct Detection {
    /** @brief The class ID to identify the object class. */
    int classId;
    /** @brief The written class name of the object class. */
    std::string className;
    /** @brief The row in which the center point of the bounding box from the detected object is in. */
    int x;
    /** @brief The column in which the center point of the bounding box from the detected object is in. */
    int y;
    /** @brief The width of the bounding box from the detected object in pixel. */
    int w;
    /** @brief The height of the bounding box from the detected object in pixel. */
    int h;
    /** @brief The confidence that the class of the detected object is correct. */
    float confidence;
    /** @brief The distance in meter to the detected object. */
    float distance;
    /** @brief The side of the frame the center of the object is in. */
    Side side;
};

/**
 * @brief Creates a string message that contains the attributes of the detected object.
 *
 * @param d The detected object from which a string of its attributes should be created.
 * @return std::string The finished resulting string.
 */
std::string toSignDetectionMessage(Detection d);

/** @brief This class holds all the functions to detect and evaluate objects in a single frame.
 *
 * For detecting objects the YOLOv5s network is used, which has been retrained with data that we created on our own.
 * The network can distinguish 15 different object classes in a single frame, and is loaded as a TensorRT model for an
 * increased performance in the runtime. Each of the detected objects will then be saved into a individual detection
 * struct. To lower the rate of false detected objects caused by environmental impact, this class also holds several
 * functions that filter the detections. Through that the sign detection has a very low error rate.
 */
class SignDetection {
public:
    /**
     * @brief Construct a new Sign Detection object.
     *
     * @param engine_path The path to the .engine file, which holds the trained YOLO network.
     */
    SignDetection(std::string engine_path);
    /**
     * @brief Main function executing the sign detection and calling necessary filtering functions.
     *
     * @param color_img The current RGB camera frame as an OpenCV matrix.
     * @param depth_img The current depth camera image as an OpenCV matrix.
     * @return std::vector<Detection> A vector that holds all the detected objects.
     */
    std::vector<Detection> detect(cv::Mat color_img, cv::Mat depth_img);

private:
    /**
     * @brief Maps all detected objects in the frame into an individual Detection object accordingly.
     *
     * @param yolov5Detections A vector that holds all the raw detected objects of the YOLO network.
     * @return std::vector<Detection> The vector that holds all the assigned Detection objects.
     */
    std::vector<Detection> mapDetections(std::vector<yolov5::Detection> yolov5Detections);
    /**
     * @brief Determines the distance of the detections to the vehicle and adds that information to the individual Detection object.
     *
     * @param detections The vector that holds all the detected objects.
     * @param depth_img The current depth camera frame.
     * @return std::vector<Detection> The vector with all the updated detected objects.
     */
    std::vector<Detection> evaluateDistance(std::vector<Detection> detections, cv::Mat depth_img);
    /**
     * @brief Determines in which side of the image the detections are in and adds that information to the individual Detection object.
     *
     * @param detections The vector that holds all the detected objects.
     * @param width The width of the camera frame in pixels.
     * @return std::vector<Detection> The vector with all the updated detected objects.
     */
    std::vector<Detection> evaluateSide(std::vector<Detection> detections, int width);
    /**
     * @brief Runs every Detection object through a series of if statements that remove false detections to a certain degree.
     *
     * @param detections The vector that holds all the detected objects.
     * @return std::vector<Detection> The vector with all the updated remaining detected objects.
     */
    std::vector<Detection> filterMisdetections(std::vector<Detection> detections);

private:
    /** @brief This is the member variable in which the .engine file is loaded into. */
    yolov5::Detector m_detector;
    /** @brief This variable is a filter that is used to count the number of times an object class is detected consecutively. */
    std::map<std::string, int> m_detectionCounters;
};
