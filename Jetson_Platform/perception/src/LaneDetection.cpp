/**
 * @file LaneDetection.cpp
 * @author Noah Koehler
 * @brief This file implements the lane detection algorithm used in the BFMC 2023.
 * @version 1.0
 * @date 2023-06-15
 */
#include "../include/LaneDetection.h"
#include <opencv2/imgproc/imgproc_c.h>

/** @brief [Pixel] Height of the transformed image. */
#define TRANSFORMED_IMAGE_HEIGHT 225
/** @brief [Pixel] Height of the transformed image. */
#define TRANSFORMED_IMAGE_WIDTH 400
/** @brief [Meter] Can be read in the documentation from Intel, RealSense RGB Sensor is shifted to the right on HW; S.140 https://www.intelrealsense.com/wp-content/uploads/2022/11/Intel-RealSense-D400-Series-Datasheet-November-2022.pdf*/
#define CAMERA_POS_OFFSET_TO_CAR_CENTER 0.0325
/** @brief [Pixel] - Check if the steepest curve can be detected entirely, otherwise raise this number. */
#define SEARCH_WINDOW_WIDTH 75
/** @brief [Pixel] Depends on image height, must be less then half the image height. */
#define NUMBER_OF_SEARCH_WINDOWS 25
/** @brief [Percentage] Is the percent of the image height, where the distance shall be calculated to. Is camera dependent. */
#define HORIZONTAL_HEIGHT_FACTOR 0.35
/** @brief [Percentage] Is the percent of the image height, where the curvature origin shall be searched for. */
#define LANE_FINDING_FACTOR 0.1
/** @brief [Meter] Measure lane width with a ruler, point of reference is the middle of each white lane. */
#define LANE_WIDTH 0.37
/** @brief [Pixel] Measure by looking at delta of rightLaneCurvaturePos at left/right lane detection edge. */
#define LANE_WIDTH_PX 270
/** @brief [Meter] Measure, by positioning the car in the middle of a straight lane. */
#define RIGHT_LANE_OFFSET 0.0
/** @brief [Meter] Measure, same as above, but shift the car so far to the left, so the left lane is detected. */
#define LEFT_LANE_OFFSET 0.37

/** @brief Variable to change the color of the search windows so the user can see if currently the left or the right lane is detected.*/
cv::Scalar testCol(255, 0, 0);

LaneDetection::LaneDetection() :
    blockNum(NUMBER_OF_SEARCH_WINDOWS), windowSize(SEARCH_WINDOW_WIDTH) {}

LaneDetection::LaneDetection(const cv::Mat _oriImage, const cv::Mat _perspectiveMatrix) :
    perspectiveMatrix(_perspectiveMatrix), originalImage(_oriImage), blockNum(NUMBER_OF_SEARCH_WINDOWS), windowSize(SEARCH_WINDOW_WIDTH) {
    histogramDistance.resize(TRANSFORMED_IMAGE_WIDTH);
    histogramCurvature.resize(TRANSFORMED_IMAGE_WIDTH);
    midPoint           = TRANSFORMED_IMAGE_WIDTH >> 1;
    midHeightDistance  = TRANSFORMED_IMAGE_HEIGHT * HORIZONTAL_HEIGHT_FACTOR;
    midHeightCurvature = TRANSFORMED_IMAGE_HEIGHT * LANE_FINDING_FACTOR;
    stepY              = TRANSFORMED_IMAGE_HEIGHT / blockNum;
    laneOffset         = RIGHT_LANE_OFFSET;
}

LaneDetection::~LaneDetection() {}

/**
 * This function is the main function of the lane detection class. This means that all the necessary functions for
 * the lane detection are beeing executed from here. Starting with the preprocessing routine, which consists of creating the
 * birds eye view through a perspective warp, then gray scaling the frame, next removing noise by adding a gaussian blur to the image
 * and then detecting the edges with a canny edge detection. After that the image will be transformed into a binary image.
 * For better error tolerance a second image gets created that, by using the green color channel of the original frame gets
 * transformed into a binary image.
 * After the preprocessing the lane detection is executed.
 */
void LaneDetection::laneDetectAlgo() {
    cv::Mat originalImageGray;

    cv::warpPerspective(originalImage, originalImage, perspectiveMatrix, originalImage.size());

    originalImage = originalImage(cv::Rect(0, 0, 400, 225));

    cv::cvtColor(originalImage, originalImageGray, cv::COLOR_RGB2GRAY);

    cv::GaussianBlur(originalImageGray, originalImageGray, cv::Size(5, 5), 0);

    cv::Canny(originalImageGray, edgeImage, 200, 230, 3);

    warpEdgeImage = edgeImage.clone();

    cv::inRange(warpEdgeImage, cv::Scalar(1), cv::Scalar(255), warpEdgeImage);

    cv::split(originalImage, imageChannels);

    cv::inRange(imageChannels[1], cv::Scalar(200), cv::Scalar(255), GreenBinary);

    cv::add(warpEdgeImage, GreenBinary, mergeImage);
    cv::cvtColor(mergeImage, mergeImageRGB, cv::COLOR_GRAY2RGB);

    calHist();

    boundaryDetection();

    laneSearch(rightLaneCurvaturePos, laneR, laneRcount, curvePointsR, 'R');

    laneCoefEstimate();

    laneFitting();
}

/**
 * To later identify the starting position of the lane search, the amount of white pixels in each column of the binary
 * frame are being counted. Those values then are stored in two dimensional vectors. Since not all rows of pixel are needed
 * there are two region of interests (ROI) defined. One for finding the start of the lanes and one for identifying
 * the delta distance to the middle of the road.
 */
void LaneDetection::calHist() {
    histogramDistance.clear();
    histogramCurvature.clear();

    for (int i = 0; i < mergeImage.size().width; i++) {
        cv::Mat ROIDistance = mergeImage(cv::Rect(i, originalImage.size().height - midHeightDistance - 1, 1, 50));
        cv::Mat dstDistance;
        cv::divide(255, ROIDistance, dstDistance);
        histogramDistance.push_back((int)(cv::sum(dstDistance)[0]));

        cv::Mat ROICurvature = mergeImage(cv::Rect(i, originalImage.size().height - midHeightCurvature - 1, 1, midHeightCurvature));
        cv::Mat dstCurvature;
        cv::divide(255, ROICurvature, dstCurvature);
        histogramCurvature.push_back((int)(cv::sum(dstCurvature)[0]));
    }
}

/**
 * Depending on the column with the most white pixels in the specific ROI, the position of the lane gets set.
 * Since we used a camera with a small field of view, we are aiming for only detecting one lane at a time. The function
 * will at first look at the right half of the transformed image. If no lane can be found according to the given
 * searching rules, the function will look in the left half of the image. Depending on the previous results it will
 * determine if the detected lane is the median strip of the road, or the right outer lane and add an offset to the distance
 * to the middle of the road.
 */
void LaneDetection::boundaryDetection() {
    testCol = cv::Scalar(255, 0, 0);
    std::vector<int>::iterator maxRPtrDistance;
    maxRPtrDistance      = std::max_element(histogramDistance.begin() + midPoint, histogramDistance.end());
    rightLaneDistancePos = std::distance(histogramDistance.begin(), maxRPtrDistance);

    std::vector<int>::iterator maxRPtrCurvature;
    maxRPtrCurvature      = std::max_element(histogramCurvature.begin() + midPoint, histogramCurvature.end());
    rightLaneCurvaturePos = std::distance(histogramCurvature.begin(), maxRPtrCurvature);

    // If the detected lane is less, or more then a specific x-value, search again in the other half of the image
    if (rightLaneCurvaturePos <= 220 || rightLaneCurvaturePos >= 380) {
        testCol               = cv::Scalar(0, 255, 0);
        maxRPtrCurvature      = std::max_element(histogramCurvature.begin(), histogramCurvature.end() - midPoint + 20);
        rightLaneCurvaturePos = std::distance(histogramCurvature.begin(), maxRPtrCurvature);
    }

    // If we cant detect a lane, assume the same lane position as in the last frame
    if (rightLaneCurvaturePos != 0) {
        // When there is a big difference between old an new pos, then the algorithm must have detected the other lane
        // if so, with the help of the delta value of old and new, it can be recognized what lane was detected
        if (rightLaneCurvaturePos - oldRightLaneCurvaturePos > 150) laneOffset = RIGHT_LANE_OFFSET;
        else if (rightLaneCurvaturePos - oldRightLaneCurvaturePos < -150) laneOffset = LEFT_LANE_OFFSET;
        else laneOffset = laneOffset;
        oldRightLaneCurvaturePos = rightLaneCurvaturePos;
    }
    else {
        rightLaneCurvaturePos = oldRightLaneCurvaturePos;
    }
}

/**
 * This function searches for the lane by using a searching box. This rectangle is used to delimit the searching area of the
 * lane. It starts the search at the previously found starting point of the lane and then continues to iterate over the frame
 * by moving the next searching box depending on the result of the previous one. When the function doesn't find the lane in
 * the current searching window it will just move the next box on top of the previous one. If this happens three times in
 * a row the function will terminate.
 */
void LaneDetection::laneSearch(const int& lanePos, std::vector<cv::Point2f>& _line, int& lanecount, std::vector<cv::Point2f>& curvePoints, char dir) {
    _line.clear();

    int       stopSearch        = 0;
    int       validationCounter = 0;
    const int skipStep          = 1;
    int       nextPosX          = lanePos;
    int       xLU = 0, yLU = 0;
    int       xRB = 0, yRB = 0;
    int       sumX     = 0;
    int       xcounter = 0;
    lanecount          = 0;

    for (int i = 0; i < blockNum; i++) {
        if (stopSearch >= 2) break;
        xLU = nextPosX - (windowSize >> 1);
        yLU = stepY * (blockNum - i - 1);
        xRB = xLU + windowSize;
        yRB = yLU + stepY - 1;

        if ((xLU < 0)) {
            xLU = 0;
            xRB = xLU + windowSize;
        }
        if (xRB > (mergeImage.size().width - 1)) {
            xRB = (mergeImage.size().width - 1);
            xLU += ((mergeImage.size().width - 1) - xRB);
        }
        if (xRB - xLU > 0 && xRB >= 0 && xLU >= 0) {
            sumX     = 0;
            xcounter = 0;
            uchar* matPtr;
            for (int j = yLU; j <= yRB; j += skipStep) {
                matPtr = mergeImage.data + (j * mergeImage.size().width);
                for (int k = xLU; k <= xRB; k += skipStep) {
                    if (*(matPtr + k) == 255) {
                        sumX += k;
                        xcounter++;
                    }
                }
            }
            if (xcounter != 0) sumX /= xcounter;
            else sumX = nextPosX;

            // Modify the window position based on previous calculated average x coordinate.
            nextPosX = sumX;
            xLU      = ((nextPosX - (windowSize >> 1)) > 0) ? (nextPosX - (windowSize >> 1)) : 0;                             // If the lane is too far in the left of the image, xLU will be set so the window wont move out of the image
            xRB      = ((xLU + windowSize) < (mergeImage.size().width)) ? (xLU + windowSize) : (mergeImage.size().width - 1); // same asm above, just for xRB
            if (xRB - xLU > 0 && xRB >= 0 && xLU >= 0) {
                for (int j = yLU; j <= yRB; j += skipStep) {
                    matPtr = mergeImage.data + (j * mergeImage.size().width);
                    for (int k = xLU; k <= xRB; k += skipStep) {
                        if (*(matPtr + k) == 255) {
                            lanecount++;
                            validationCounter++;
                            _line.push_back(cv::Point2f(k, j));
                        }
                    }
                }
                if (validationCounter <= 10) {
                    stopSearch++;
                }
                else {
                    stopSearch = 0;
                }
                validationCounter = 0;
            }
            cv::rectangle(mergeImageRGB, cv::Point2f(xLU, yLU), cv::Point2f(xRB, yRB), testCol, 5);
        }
        detectedWindows = i;
    }
}

/**
 * For a good result the function creates a linear system and solves it using SVD
 */
bool LaneDetection::laneCoefEstimate() {
    int countThreshold = 300;
    if (laneRcount > countThreshold && detectedWindows > 2) {
        Eigen::VectorXd xValueR(laneRcount);
        Eigen::MatrixXd rightMatrix(laneRcount, 3);

        for (int i = 0; i < laneRcount; i++) {
            xValueR(i)        = laneR[i].x;
            rightMatrix(i, 0) = pow(laneR[i].y, 2);
            rightMatrix(i, 1) = laneR[i].y;
            rightMatrix(i, 2) = 1;
        }

        curveCoefR                      = (rightMatrix.transpose() * rightMatrix).ldlt().solve(rightMatrix.transpose() * xValueR);
        curveCoefRecordR[recordCounter] = curveCoefR;
        recordCounter                   = (recordCounter + 1) % 5;

        if (initRecordCount < 5) initRecordCount++;
        failDetectFlag = false;
        return true;
    }
    else {
        failDetectFlag = true;
        return false;
    }
}

/**
 * To obtain a more stable result the new parabola coefficients are averaged over the past five results.
 * This leads to reducing the effect of e.g. a wrong detected lane in a single frame because of noise or reflections.
 */
void LaneDetection::laneFitting() {
    maskImage.create(mergeImage.size().height, mergeImage.size().width, CV_8UC3);
    maskImage = cv::Scalar(0, 0, 0);
    curvePointsR.clear();

    int test = 0;

    if (initRecordCount == 5) {
        curveCoefR = (curveCoefRecordR[0] + curveCoefRecordR[1] + curveCoefRecordR[2] + curveCoefRecordR[3] + curveCoefRecordR[4]) / 5;
    }

    int xR;
    for (int i = 0; i < mergeImage.size().height; i++) {
        xR = pow(i, 2) * curveCoefR(0) + i * curveCoefR(1) + curveCoefR(2);
        if (xR < 0) xR = 0;
        if (xR >= mergeImage.size().width) xR = mergeImage.size().width - 1;
        curvePointsR.push_back(cv::Point2f(xR, i));
        if (i > 160 && i < 180) test += xR;
    }
    test = test / 20;
    std::vector<int>::iterator maxRPtrDistance;
    maxRPtrDistance      = std::max_element(histogramDistance.begin() + test - 50, histogramDistance.begin() + test + 50);
    rightLaneDistancePos = std::distance(histogramDistance.begin(), maxRPtrDistance);
}

cv::Mat LaneDetection::getEdgeDetectResult() {
    return edgeImage;
}

cv::Mat LaneDetection::getWarpEdgeDetectResult() {
    return warpEdgeImage;
}

cv::Mat LaneDetection::getGreenChannel() {
    return imageChannels[1];
}

cv::Mat LaneDetection::getGreenBinary() {
    return GreenBinary;
}

cv::Mat LaneDetection::getMergeImage() {
    return mergeImageRGB;
}

cv::Mat LaneDetection::getHistImage() {
    return histogramImage;
}

cv::Mat LaneDetection::getMaskImage() {
    return maskImage;
}

cv::Mat LaneDetection::getWarpMask() {
    return maskImageWarp;
}

cv::Mat LaneDetection::getFinalResult() {
    cv::addWeighted(maskImageWarp, 0.5, originalImage, 1, 0, finalResult);
    return finalResult;
}

void LaneDetection::setInputImage(cv::Mat& image) {
    originalImage = image.clone();
}

float LaneDetection::getCurveCoefficient() {
    return curveCoefR[0];
}

float LaneDetection::getLaneCenterDist() {

    float laneCenter  = rightLaneDistancePos - (LANE_WIDTH_PX / 2);
    float imageCenter = mergeImageRGB.size().width / 2;

    result = (laneCenter - imageCenter) * (LANE_WIDTH / LANE_WIDTH_PX) + CAMERA_POS_OFFSET_TO_CAR_CENTER + laneOffset;

    return result;
}

std::string LaneDetection::toLaneDetectionMessage(float distToMid, float curveCoeff) {
    return std::string("1;" + std::to_string(distToMid) + ";" + std::to_string(curveCoeff));
}

int LaneDetection::getLaneCurvaturePos() {
    return rightLaneCurvaturePos;
}

int LaneDetection::getLaneDistancePos() {
    return rightLaneDistancePos;
}
