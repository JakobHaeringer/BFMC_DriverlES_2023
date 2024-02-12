/**
 * @file ObjectDetection.cpp
 * @author Julius Baechle, Kadir Oezer, Noah Koehler
 * @brief This file implements the object detection algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-06-15
 */

#include "ObjectDetection.h"
#include "AvgDistImage.h"
#include <math.h>

/** @brief The least confidence a detected object needs, to count as valid detection except the class stop-line. */
#define CONFIDENCE_THRESHOLD 0.5
/** @brief The least confidence a detected object from the class stop-line needs, to count as a valid detection. */
#define STOP_LINE_CONFIDENCE_THRESHOLD 0.2
/** @brief The minimal distance a detected object needs, to count as a valid detection. */
#define MIN_DISTANCE 0.0
/** @brief The maximum distance a detected object is allowed to have, to count as a valid detection with a few exceptions. */
#define MAX_DISTANCE_GENERIC 1.0
/** @brief The maximum distance a detected object from the class car or crosswalk-sign is allowed to have, to count as a valid detection. */
#define MAX_DISTANCE_CROSSWALK_CAR 2.0
/** @brief The maximum distance a detected object from the class pedestrian is allowed to have, to count as a valid detection. */
#define MAX_DISTANCE_PEDESTRIAN 3.0
/** @brief The maximum distance a detected object from the class stop-line is allowed to have, to count as a valid detection.*/
#define MAX_DISTANCE_STOP_LINE 1.5 // 1.0
/** @brief A helper function that returns the smaller value of to given integer values. */
#define min(x, y) (((x) > (y)) ? (y) : (x))

/** @brief This vector holds all class-types of possible object classes that are looked for in a frame. */
const std::vector<std::string> classes = {
    "car",
    "closed-road-stand",
    "crosswalk-sign",
    "highway-entry-sign",
    "highway-exit-sign",
    "no-entry-road-sign",
    "one-way-road-sign",
    "parking-sign",
    "parking-spot",
    "pedestrian",
    "priority-sign",
    "round-about-sign",
    "stop-line",
    "stop-sign",
    "traffic-light"};

ObjectDetection::ObjectDetection(std::string engine_path) {
    m_detector.init();
    m_detector.loadEngine(engine_path);
    assert(m_detector.isEngineLoaded());

    m_detector.setScoreThreshold(0.2);
    m_detector.setNmsThreshold(0.4);

    yolov5::Classes classes;
    classes.load(::classes);
    m_detector.setClasses(classes);
}
/**
 * The function takes in two frames, one RGB and one depth frame. The RGB frame is then given to the YOLOv5 network which
 * then looks trough the image for all the 15 possible classes. If one or multiple objects are detected, the detections
 * are being saved in a vector. Upon finishing the search for objects, the detections are mapped to a vector of
 * that holds objects of the custom Detection class. In addition to the information the YOLO detection already has, the
 * distance to the detected object as well as the side of the object in the frame is saved into the detections vector.
 * As last step all the detected objects are being filtered and then finally returned by this function.
 */
std::vector<Detection> ObjectDetection::detect(cv::Mat color_img, cv::Mat depth_img) {
    std::vector<yolov5::Detection> yolov5Detections;
    m_detector.detect(color_img, &yolov5Detections);

    auto detections = mapDetections(yolov5Detections);
    detections      = evaluateDistance(detections, depth_img);
    detections      = evaluateSide(detections, color_img.cols);
    return filterMisdetections(detections);
}

std::vector<Detection> ObjectDetection::mapDetections(std::vector<yolov5::Detection> yolov5Detections) {
    std::vector<Detection> result;
    for (const auto& yolov5Detection : yolov5Detections) {
        Detection d;
        d.classId    = yolov5Detection.classId();
        d.className  = yolov5Detection.className();
        d.x          = yolov5Detection.boundingBox().x + yolov5Detection.boundingBox().width / 2;
        d.y          = yolov5Detection.boundingBox().y + yolov5Detection.boundingBox().height / 2;
        d.w          = yolov5Detection.boundingBox().width;
        d.h          = yolov5Detection.boundingBox().height;
        d.confidence = yolov5Detection.score();
        result.push_back(d);
        // std::cout << "BEFORE FILTER: Class Name: " << d.className << "\tConfidence: " << d.confidence << std::endl;
    }
    return result;
}

/**
 * Since it isn't optimal to alway use the center from the bounding box of the detected object, the function differentiates
 * the object class of the detection and depending on the type, extracts the distance to the object from the depth frame.
 * If the object is of the type stop-line, parking-spot or car, the row number of the lower edge of the detection is being
 * used to look up the distance of the ground truth values in the helper vector "average_depth_img".
 */
std::vector<Detection> ObjectDetection::evaluateDistance(std::vector<Detection> detections, cv::Mat depth_img) {
    for (Detection& d : detections) {
        if ((d.className == "stop-line") || (d.className == "parking-spot") || (d.className == "car")) {
            int index  = min(d.y + 0.5 * d.h, average_depth_img.size() - 1);
            d.distance = average_depth_img[index];
        }
        else {
            d.distance = cv::mean(depth_img(cv::Rect(d.x - 1, d.y - 1, 3, 3)))[0];
        }
    }
    return detections;
}

/**
 * By spliting the image into three evenly spaced parts, across the width of the image, the side of the detection in the
 * frame can be determined. This helps to implement a pseudo object tracking, when the previous side of a detected object
 * class is compared to the new one. The limitation of this method is reached when there are two detections of the same
 * class type.
 */
std::vector<Detection> ObjectDetection::evaluateSide(std::vector<Detection> detections, int width) {
    for (Detection& d : detections)
        d.side = (Side)((3 * d.x) / width);
    return detections;
}

/**
 * if distance is between 0 and 1 m and it's not on the left side (for most classes)
 *
 * Filtering out error detections is an important step. This reduces the amount of data that need to be send to the
 * Raspberry Pi. It also reduces the required CPU usage of the Raspberry, but most importantly it minimizes the
 * misbehavior of the vehicle since false detections aren't forwarded to the planning algorithm.
 * Following checks are made to the reduce the error rate:
 * - check if the object class has been detected in at least three consecutive frames
 * - check the distance of the object compared to the ground truth to remove reflections
 * - check the side of the detected object in the frame. If e.g. the class is a traffic sign it is not forwarded
 *   if it is on the left side of the frame
 * With those checks there are almost no error detections forwarded to the Raspberry Pi.
 */
std::vector<Detection> ObjectDetection::filterMisdetections(std::vector<Detection> detections) {
    for (auto const& c : classes) {
        bool detected          = std::find_if(detections.begin(), detections.end(), [&](Detection& d) { return d.className == c; }) != detections.end();
        m_detectionCounters[c] = detected ? m_detectionCounters[c] + 1 : 0;
    }

    std::vector<Detection> result;
    for (auto const& d : detections) {
        if (d.confidence < (d.className == "stop-line" ? STOP_LINE_CONFIDENCE_THRESHOLD : CONFIDENCE_THRESHOLD)) continue;
        // Currenty we are not using the detection of parking spots
        if (d.className == "parking-spot") continue;
        // Is the distance less then zero
        if (d.distance <= MIN_DISTANCE) continue;
        // Has the object been detected at least two times consecutively
        if (m_detectionCounters[d.className] < 2 && d.className != "stop-line") continue;
        // Has the object been detected on the left side of the road and is type "sign"
        if (d.side == Left && (d.className == "stop-sign" || d.className == "priority-sign" || d.className == "crosswalk-sign" || d.className == "round-about-sign" || d.className == "parking-sign" || d.className == "traffic-light")) continue;
        // Different distance constraints for the different object types
        if (d.distance > MAX_DISTANCE_STOP_LINE && d.className == "stop-line") continue;
        if (d.distance > MAX_DISTANCE_GENERIC && (d.className == "stop-sign" || d.className == "priority-sign" || d.className == "round-about-sign" || d.className == "parking-sign" || d.className == "closed-road-stand" || d.className == "traffic-light")) continue;
        if (d.distance > MAX_DISTANCE_CROSSWALK_CAR && (d.className == "car" || d.className == "crosswalk-sign")) continue;
        if (d.distance > MAX_DISTANCE_PEDESTRIAN && d.className == "pedestrian") continue;
        if (abs(d.distance - average_depth_img[d.y]) < 0.05 && ((d.className != "stop-line") && (d.className != "parking-spot") && (d.className != "car"))) continue;
        result.push_back(d);
    }
    return result;
}

std::string toObjectDetectionMessage(Detection d) {
    std::string msg = "2;";
    msg += d.className + ";";
    msg += std::to_string(d.x) + ";";
    msg += std::to_string(d.y) + ";";
    msg += std::to_string(d.w) + ";";
    msg += std::to_string(d.h) + ";";
    msg += std::to_string(d.confidence) + ";";
    msg += std::to_string(d.distance) + ";";
    msg += std::to_string((int)d.side);
    return msg;
}
