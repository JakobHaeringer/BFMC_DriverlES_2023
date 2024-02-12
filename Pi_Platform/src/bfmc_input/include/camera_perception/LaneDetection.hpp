/**
 * @file LaneDetection.hpp
 * @author Noah Koehler
 * @brief The header file of the lane detection used in the BFMC 2023.
 * @version 1.0
 * @date 2023-06-15
 */

#pragma once

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief This class implements the lane search algorithm.
 *
 * It is based on a simple, but fast sliding window search.
 * From the X- and Y-values found during this search, a singular value decomposition (SVD) is performed to find the
 * best fitting coefficients for a third degree polynomial that fits the shape of the detected lane.
 * The result of this calculation, as well as the distance of the car to the center of the road ahead is then built
 * into a string message which can be communicated to other processes.
 */
class LaneDetection {
private:
    /**
     * @defgroup LaneDetectionGroup1 Preprocessing Matrixes
     *
     * This group contains all the OpenCV matrixes that are used to hold provisional results of the preprocessing routine.
     * @{ */

    /** @brief The matrix that is used to transform the camera frame into the birds eye view. */
    cv::Mat perspectiveMatrix;
    /** @brief The most current RGB camera frame in its original format. */
    cv::Mat originalImage;
    /** @brief The birds eye view transformed original RGB frame. */
    cv::Mat warpOriginalImage;
    /** @brief The resulting frame of the canny edge detection of the birds eye view. */
    cv::Mat warpEdgeImage;
    /** @brief The vector holds the separate color channels of the original frame. */
    std::vector<cv::Mat> imageChannels;
    /** @brief The binary frame of the green image channel. */
    cv::Mat GreenBinary;
    /** @brief The merge image of the canny edge detection and the binary image of the green image channel. */
    cv::Mat mergeImage;
    /** @brief A frame that shows the histogram of the detected white pixels per column. */
    cv::Mat histogramImage;
    /** @} */

    /**
     * @defgroup LaneDetectionGroup2 Lane Search Points
     *
     * This group contains all member variables that are used for saving starting points for the lane search in the current frame.
     * @{ */
    /** @brief The histogram of the ROI for finding the distance of the car to the center of the lane. */
    std::vector<int> histogramDistance;
    /** @brief The histogram of the ROI for finding the starting point of the lane at the bottom of the image. */
    std::vector<int> histogramCurvature;
    /** @brief The row in which the ROI is set for finding lanes to determine the distance to the road center. */
    int midHeightDistance;
    /** @brief The row in which the ROI is set for finding the starting point of the lanes in the bottom of the image. */
    int midHeightCurvature;
    /** @brief The column of the right lane boundary for determining the distance to the road center. */
    int rightLaneDistancePos;
    /** @brief The column of the right lane boundary for determining the starting point of the lane in the bottom of the frame. */
    int rightLaneCurvaturePos;
    /** @brief The column of the right lane boundary for determining the starting point of the lane in the bottom of the previous frame.*/
    int oldRightLaneCurvaturePos = 0;
    /** @}*/

    /** @brief This vector holds the column values of the detected lane points. */
    std::vector<cv::Point2f> laneR;
    /** @brief This vector holds the y value from the estimated third degree parabola. */
    std::vector<cv::Point2f> curvePointsR;
    /** @brief The number of detected lane pixels which is used as a threshold to reduce false detections. */
    int laneRcount;
    /** @brief The middle column of the image. */
    int midPoint;
    /** @brief A counter that is used as iterator for the curveCoefRecordR vector. */
    int recordCounter = 0;
    /** @brief The step size that is used for skipping columns in the current searching window to save performance time. */
    int stepY;
    /** @brief A counter to stop the lane search when there is no lane detected in a few consecutive search windows. */
    int detectedWindows = 0;
    /** @brief A counter used to signal the algorithm that it can start to average the new curve coefficients with the past five ones. */
    short initRecordCount = 0;
    /** @brief The coefficients as a result of the estimation of the parabola for the detected current lane. */
    Eigen::Vector3d curveCoefR;
    /** @brief The vector holds the past five resulting coefficients to smoothen out the result of the current calculation. */
    Eigen::Vector3d curveCoefRecordR[5];
    /** @brief The offset that is added to the distance to the road center when the left lane is detected instead of the right. */
    float laneOffset = 0.0f;

    // Unnecessary
    bool                                           failDetectFlag = true; // To indicate whether the road marks is detected successfully.
    bool                                           updateValues   = false;
    float                                          oldResult      = 0.0f;
    float                                          oldLaneOffset  = 0.0f;
    const int                                      blockNum;   // Number of windows per line.
    const int                                      windowSize; // Window Size (Horizontal).
    cv::Mat                                        mergeImageRGB;
    cv::Mat                                        edgeImage; // The result of applying canny edge detection
    cv::Mat                                        maskImage; // The curve image used to blend to the original image.
    cv::Mat                                        finalResult;
    cv::Mat                                        maskImageWarp;
    std::chrono::high_resolution_clock::time_point tStart;
    std::chrono::high_resolution_clock::time_point tEnd;
    float                                          result = 0.0f;

    /**
     * @brief Creates a histogram of the number of white pixels per column.
     */
    void calHist();

    /**
     * @brief Identify's the starting position of the lanes in the current frame.
     */
    void boundaryDetection();

    /**
     * @brief Creates a vector of y values for each row whereas the y value is the position of the lane in that row.
     *
     * @param lanePos Y-position to start the lane search at
     * @param _line The vector that holds the resulting y-values
     * @param lanecount The number of detected lane pixels
     * @param curvePoints TODO: Not in use, check if it can be removed
     * @param dir TODO: Not in use, check if it can be removed
     */
    void laneSearch(const int& lanePos, std::vector<cv::Point2f>& _line, int& lanecount, std::vector<cv::Point2f>& curvePoints, char dir);

    /**
     * @brief Estimates the coefficients of the 3rd degree polynomial that fits the detected lane the best.
     *
     * @return true A estimation of the lane curvature was made
     * @return false Not enough pixels of the lane could be found for a precise estimation
     */
    bool laneCoefEstimate();

    /**
     * @brief Solves the third degree polynomial with the estimated coefficients for every X-Value.
     */
    void laneFitting();

public:
    /**
     * @brief Standard constructor to construct a new LaneDetection::LaneDetection object
     */
    LaneDetection();

    /**
     * @brief Construct a new LaneDetection::LaneDetection object.
     *
     * @param _oriImage The most current camera frame
     * @param _perspectiveMatrix The rotation matrix that will be used for the birds eye view transformation
     */
    LaneDetection(cv::Mat _oriImage, cv::Mat _perspectiveMatrix);

    /**
     * @brief Destroy the lane Detection object
     */
    ~LaneDetection();

    /**
     * @brief Main function executing the preprocessing routine and calling necessary functions for detecting lanes.
     */
    void laneDetectAlgo();

    /**
     * @defgroup Lane Detection Getter Methods
     *
     * This group contains all the member functions that return OpenCV matrix objects from different time points in the
     * preprocessing routine of the current camera frame.
     *
     * @{
     */

    /**
     * @brief Returns the current canny edge image.
     *
     * @return The matrix that holds the canny frame
     */
    cv::Mat getEdgeDetectResult();

    /**
     * TODO: Check if necessary
     * @brief Returns the birds eye view of the current canny edge image.
     *
     * @return The matrix that holds the canny frame
     */
    cv::Mat getWarpEdgeDetectResult();

    /**
     * @brief Returns the green channel of the current image.
     *
     * @return The matrix that holds the green image channel of the current frame
     */
    cv::Mat getGreenChannel();

    /**
     * @brief Returns the green binary image.
     *
     * @return The matrix that holds the binary frame of the green image channel
     */
    cv::Mat getGreenBinary();

    /**
     * @brief Returns the merged image of the green binary image and the canny edge image.
     *
     * @return The matrix that holds the merged image of the canny transformed frame and the green
     * channel binary image
     */
    cv::Mat getMergeImage();

    /**
     * @brief Returns the histogram of the counted white pixels per column.
     *
     * @return The matrix that holds the histogram
     */
    cv::Mat getHistImage();

    /**
     * TODO: Check if necessary
     * @brief
     *
     * @return The matrix that holds the canny frame
     */
    cv::Mat getMaskImage();

    /**
     * TODO: Check if necessary
     * @brief
     *
     * @return The matrix that holds the canny frame
     */
    cv::Mat getWarpMask();

    /**
     * TODO: Check if necessary
     * @brief
     *
     * @return The OpenCV matrix that holds the canny frame
     */
    cv::Mat getFinalResult();
    /** @} */

    /**
     * @brief Returns the distance the car has to the road center.
     *
     * @return float Holds the distance in meter
     */
    float getLaneCenterDist();

    /**
     * @brief Returns the quadratic polynomial coefficient.
     *
     * @return float Holds the coefficient value, usually has values bellow 1
     */
    float getCurveCoefficient();

    /**
     * @brief This function sets the frame, in which the algorithm will try to detect lanes.
     *
     * @param image The matrix that holds the frame
     */
    void setInputImage(const cv::Mat& image);

    /**
     * @brief Creates a string of the current distance to the road center and the curve coefficient.
     *
     * @param distToMid Float value of the distance to the center of the road in meter
     * @param curveCoeff Float value of the quadratic polynomial curve coefficient of the lane.
     * @return std::string A string in the format, so it can be sent to the RaspberryPi
     */
    std::string toLaneDetectionMessage(float distToMid, float curveCoeff);

    /**
     * @brief Returns the column at which the lane search has started in the current frame.
     *
     * @return int Column of the lane position, somewhere between 0 and 400
     */
    int getLaneCurvaturePos();

    /**
     * @brief Returns the column at which the lane is found in the area the algorithm looks for to find the distance to the
     * road center.
     *
     * @return int Column of the lane position, somewhere between 0 and 400
     */
    int getLaneDistancePos();
};
