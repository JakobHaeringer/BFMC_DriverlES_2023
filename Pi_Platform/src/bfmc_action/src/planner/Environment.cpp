/**
 * @file Environment.cpp
 * @author Jakob Haeringer and Noah Koehler
 * @brief This file implements the Environment class used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */
#include "planner/Environment.hpp"

#define MIN_DISTANCE_PEDESTRIAN_ON_ROAD      0.5f    ///< [m] Safety distance to a pedestrian (~30 cm (Camera to front of car) + ~20cm).
#define MAX_DISTANCE_PEDESTRIAN_ON_ROAD      0.8f    ///< [m] Maximum distance to react on pedestrians on the road (~30 cm (Camera to front of car) + ~50cm).
#define MAX_DISTANCE_PEDESTRIAN_ON_CROSSWALK 2.5f    ///< [m] Maximum distance to react on pedestrians on a crosswalk.
#define OFFSET_DISTANCE_SIGNS                0.08f   ///< [m] Offset of the calculated distance to traffic signs.
#define OFFSET_DISTANCE_STOP_LINE            0.30f   ///< [m] Offset of the calculated distance to the stop lines
#define PITCH_THRESHOLD_UPWARDS              -0.075f ///< [°] Threshold for detecting the vehicle is going up the ramp.
#define PITCH_THRESHOLD_DOWNWARDS            0.08f   ///< [°] Threshold for detecting the vehicle is going down the ramp.
#define DISTANCE_TO_START_LANE_CHANGE        0.85f   ///< [m] Minimum distance for entering the state lane change.
#define MAX_TIME_FOR_VALID_STOP_LINE         1.5f    ///< [s] Maximum passed time a stop line was detected to be valid.

ITMOVES::Environment::Environment(ros::NodeHandle& handle) {
    // ---------------------------------------------------  Private --------------- ------------------------------------
    this->subs = {
        handle.subscribe<bfmc_interface::LaneDetection>("/input/camera/lane_detection", 1, &ITMOVES::Environment::laneDetectionCallback, this),
        handle.subscribe<bfmc_interface::SignDetection>("/input/camera/sign_detection", 5, &ITMOVES::Environment::signDetectionCallback, this),
        handle.subscribe<bfmc_interface::IMU>("/input/imu", 1, &ITMOVES::Environment::imuCallback, this),
        handle.subscribe<bfmc_interface::WheelSpeed>("/input/wheel_speed", 1, &ITMOVES::Environment::wheelSpeedCallback, this),
        handle.subscribe<bfmc_interface::TrafficLight>("/input/environment/traffic_light/start", 1, &ITMOVES::Environment::trafficLightStartCallback, this),
        handle.subscribe<bfmc_interface::TrafficLight>("/input/environment/traffic_light/east", 1, &ITMOVES::Environment::trafficLightEastCallback, this),
        handle.subscribe<bfmc_interface::TrafficLight>("/input/environment/traffic_light/south", 1, &ITMOVES::Environment::trafficLightSouthCallback, this),
        handle.subscribe<bfmc_interface::TrafficLight>("/input/environment/traffic_light/west", 1, &ITMOVES::Environment::trafficLightWestCallback, this),
        handle.subscribe<std_msgs::UInt8>("/input/remote_state_control", 1, &ITMOVES::Environment::stateCallback, this)};

    this->egoPosePub = handle.advertise<bfmc_interface::EgoPose>("/output/ego_pose", 1);

    // ---------------------------------------------------  Public --------------- ------------------------------------
    // Initialize variables which are not set in launch file
    this->initROIs();

    // GroupEgo Vehicle state
    this->currentPose               = {0, 0.0f, 0.0f, CAR_FACES_NORTH, 0.0f}; // ID, x, y, yaw, velo
    this->travelledDistance         = 0.0f;
    this->rampInfo.drivingDownwards = false;
    this->rampInfo.drivingUpwards   = false;
    this->carIsParked               = false;
    this->tStart                    = ros::Time::now();
    this->egoPoseMsg.header.stamp   = ros::Time::now();
    this->egoPoseMsg.nodeID         = 0;
    this->egoPoseMsg.x              = 0.0f;
    this->egoPoseMsg.y              = 0.0f;
    this->egoPoseMsg.yaw            = CAR_FACES_NORTH;

    // GroupObjects Signs/ Objects/ Lane Detection
    this->signObjectReset();
    this->trafficLightState = TrafficLight::UNDEFINED;
    // this->competitorCarPose = {{0, 0, 0, 0}};
    this->distanceToObject       = 0.0f;
    this->distanceToStopLine     = 0.0f;
    this->timeStopLineDetected   = ros::Time::now();
    this->distanceToCar          = 0.0f;
    this->distanceToParkingSpot  = 0.0f;
    this->distanceToTrafficLight = 0.0f;
    this->overtakeDistance       = 0.0f;
    this->savedTravelledDistance = 0.0f;
    this->curveCoeff             = 0.0f;
    this->midDistance            = 0.0f;

    // GroupIntersection Routeplanning/ Intersection variables
    this->plannedRoute         = {{0, 0.0f, 0.0f, true, 0.0f}};
    this->currentNodeID        = 0;
    this->targetNodeID         = 0;
    this->localRouteIterator   = 0;
    this->globalRouteIterator  = 0;
    this->intersectionCrossing = UNDEFINED_INTERSECTION;
    this->rightTurn            = {
        {0.03, 5.0},
        {0.06, 10.0},
        {0.13, 15.0},
        {0.75, 20.0},
        {0.85, 22.0},
        {0.95, 22.0},
        {1.05, 20.0},
        {1.15, 15.0}};
    this->leftTurn = {
        {0.07, 0.0},
        {0.14, -5.0},
        {0.21, -11.0},
        {0.28, -15.0},
        {0.35, -19.0},
        {0.98, -21.0},
        {1.05, -22.0},
        {1.12, -22.0},
        {1.26, -21.0},
        {1.33, -19.0},
        {1.43, -19.0}};
    this->straightIntersection = {
        {0.08, 0.0},
        {0.15, 0.0},
        {0.65, 0.0},
        {0.75, 0.0},
        {0.85, 0.0},
        {0.95, 0.0},
        {1.05, 0.0},
        {1.15, 0.0},
        {1.30, 0.0},
        {1.50, 0.0}};

    this->startTrafficLightPassed = false;
    this->southTrafficLightPassed = false;
    this->oneWayRoadPassed        = false;
    this->raceMode                = false;
    this->hardReset               = false;
};

ITMOVES::Environment::~Environment() {}

void ITMOVES::Environment::initROIs() {
    this->crosswalksROI        = {{5.88, 0.85, 7.01, 3.1},   // Node IDs: 173/251, 199/269
                                  {6.16, 3.1, 7.6, 4.73},    // Node IDs: 279/269, 298/343
                                  {0.0, 11.77, 1.38, 12.8}}; // Node IDs: {0.0}/95, 80
    this->parkingROI           = {{1.69, 0.0, 5.12, 3.76}};  // Node IDs: 184/{0.0}, 310/86
    this->roundaboutROI        = {{7.76, 2.09, 11.59, 5.22}};
    this->noEntriesROI         = {{10.5, 3.1, 13.09, 5.26},    // Node IDs: 307/269, 237/340
                                  {1.38, 6.61, 3.96, 7.98}};   // Node IDs: 61, 147
    this->roadBlockROI         = {{1.76, 6.97, 4.0, 10.84}};   // Node IDs: 144, 112
    this->highwayExitsROI      = {{9.65, 5.1, 11.21, 7.42},    // Node IDs: 344, 233/335
                                  {4.9, 12.14, 7.62, 15.0}};   // Node IDs: 108, 312
    this->highwayEntriesROI    = {{4.9, 12.8, 7.62, 15.0},     // Node IDs: 5, 310/86
                                  {8.35, 5.43, 10.53, 8.18}};  // Node IDs: 300/345, 333
    this->singleOneWayROI      = {{7.87, 13.33, 10.22, 15.0}}; // Node IDs: 421, 429/{15.0}
    this->doubleOneWayROI      = {{1.72, 9.13, 4.0, 10.84}};   // Node IDs: 111/140, 112
    this->trafficLightStartROI = {0.0, 13.50, 1.76, 15.0};     // Node IDs: {0.0}/78, 78/{15.0}
    this->trafficLightSouthROI = {2.5, 11.10, 3.63, 12.80};    // Node IDs: 2, 5/98
    this->trafficLightWestROI  = {0.82, 9.9, 2.68, 11.41};     // Node IDs: 71, 3
    this->trafficLightEastROI  = {3.05, 9.89, 4.91, 11.39};    // Node IDs: 7, 35
    this->intersectionSignsROI = {{0.0, 5.28, 6.87, 15.0},     // Node IDs: {0}/149, 310/{15.0}
                                  {10.5, 3.74, 13.09, 5.52}};  // Node IDs: 307, 237/340
}

/**
 * It is first checked for the "StartUp", "ChangeLane" and "Intersection" states, as these must not be interrupted.
 * Each sign/ object is checked against there ROIs. If the ROIs are valid the states are set accordingly to the detected signs/ objects and their specific conditions.
 * For each state different variables needs to be stored like the currently travelled distance, the area of the detected objects the current time etc.
 * The callback is triggered by the topic "/perception/signdetection".
 */
void ITMOVES::Environment::signDetectionCallback(const bfmc_interface::SignDetectionConstPtr& msg) {
    // ROS_INFO("Class: %s, Distance: %f, Confidence: %f, Area: %d, Box_X: %d, Box_Y: %d", msg->sign_class.c_str(), msg->distance, msg->confidence, msg->area, msg->x_bounding_box, msg->y_bounding_box);
    if (this->currentState != State::START_UP && this->currentState != State::CHANGE_LANE && this->currentState != State::INTERSECTION) {
        if (msg->sign_class == "stop-sign" && this->validateDetection(intersectionSignsROI, msg->distance)) {
            if (!this->stopSign && msg->distance >= OFFSET_DISTANCE_SIGNS) {
                this->stopSign               = true;
                this->savedTravelledDistance = this->travelledDistance;
                // If the car has detected a stop line longer than 1.5 seconds ago (not relevant for currentState) - update distanceToStopLine with detected distance to object
                if ((ros::Time::now() - this->timeStopLineDetected).toSec() > MAX_TIME_FOR_VALID_STOP_LINE) this->distanceToStopLine = msg->distance - OFFSET_DISTANCE_SIGNS;
                this->currentState = State::STOP;
                // SingleOneWay road ends with stop sign - reset onewayRoadSign and distToCar here because state changes when detecting stop sign
                this->carDetected    = false;
                this->oneWayRoadSign = false;
                this->distanceToCar  = 0;
            }
        }
        else if (msg->sign_class == "priority-sign" && this->validateDetection(intersectionSignsROI, msg->distance)) {
            if (!this->prioritySign && msg->distance >= OFFSET_DISTANCE_SIGNS) {
                this->prioritySign           = true;
                this->savedTravelledDistance = this->travelledDistance;
                // If the car has detected a stop line longer than 1.5 seconds ago (not relevant for currentState) - update distanceToStopLine with detected distance to object
                if ((ros::Time::now() - this->timeStopLineDetected).toSec() > MAX_TIME_FOR_VALID_STOP_LINE) this->distanceToStopLine = msg->distance - OFFSET_DISTANCE_SIGNS;
                this->currentState = State::PRIORITY_ROAD;
            }
        }
        else if (msg->sign_class == "parking-sign" && this->validateDetection(parkingROI, msg->distance)) {
            if (!this->parkingSign && msg->distance >= OFFSET_DISTANCE_SIGNS) {
                this->parkingSign            = true;
                this->distanceToObject       = msg->distance;
                this->savedTravelledDistance = this->travelledDistance;
            }
        }
        else if (msg->sign_class == "crosswalk-sign" && this->validateDetection(crosswalksROI, msg->distance)) {
            if (!this->crossWalkSign && msg->distance >= OFFSET_DISTANCE_SIGNS) {
                this->crossWalkSign          = true;
                this->savedTravelledDistance = this->travelledDistance;
                // If the car has detected a stop line longer than 1.5 seconds ago - (not relevant for currentState) - update distanceToStopLine with detected distance to object
                if ((ros::Time::now() - this->timeStopLineDetected).toSec() > MAX_TIME_FOR_VALID_STOP_LINE) this->distanceToStopLine = msg->distance - OFFSET_DISTANCE_SIGNS;
                this->currentState = State::CROSSWALK;
            }
        }
        else if (msg->sign_class == "pedestrian") {
            // To detect pedestrians in a given distance - especially for the crosswalk behavior on the left and right
            if (msg->distance >= MAX_DISTANCE_PEDESTRIAN_ON_ROAD && msg->distance < MAX_DISTANCE_PEDESTRIAN_ON_CROSSWALK) {
                this->pedestrian.detected    = true;
                this->pedestrian.enteredArea = msg->area; // Entered area will be updated
            }
            // Handling detecting the pedestrian randomly on the road
            else if (this->currentState != State::CROSSWALK && msg->distance >= MIN_DISTANCE_PEDESTRIAN_ON_ROAD && msg->distance < MAX_DISTANCE_PEDESTRIAN_ON_ROAD) {
                this->pedestrian.detected    = true;
                this->pedestrian.enteredArea = msg->area;
                this->pedestrian.currentArea = msg->area;
                this->savedTravelledDistance = this->travelledDistance;
                // Set distance to approach pedestrian
                this->distanceToObject = msg->distance - MIN_DISTANCE_PEDESTRIAN_ON_ROAD;
                this->tStart           = ros::Time::now();
                this->currentState     = State::PEDESTRIAN;
            }
            // Update position of pedestrian in the frame when already detected
            else if (this->pedestrian.detected) {
                this->pedestrian.currentArea = msg->area;
            }
        }
        else if (msg->sign_class == "stop-line" && this->currentState != State::PARKING && this->currentState != State::ROUNDABOUT && this->currentState != State::INTERSECTION) {
            this->distanceToStopLine     = msg->distance - OFFSET_DISTANCE_STOP_LINE;
            this->savedTravelledDistance = this->travelledDistance;
            this->timeStopLineDetected   = ros::Time::now();
        }
        else if (msg->sign_class == "car") {
            if (!this->carDetected && this->parkingSign && msg->area == LEFT_AREA && !this->carIsParked) {
                this->distanceToCar = msg->distance + this->travelledDistance - this->savedTravelledDistance;
                this->currentState  = State::PARKING;
                this->carDetected   = true;
            }
            // When following car in one way lane update distance to car
            else if (this->currentState == State::SINGLE_ONE_WAY) {
                this->distanceToCar = msg->distance;
            }
            // When driving in FollowLane and detecting a car in front of us, keep updating the dist to car
            else if (this->currentState == State::FOLLOW_LANE && msg->area == MIDDLE_AREA && msg->distance < DISTANCE_TO_START_LANE_CHANGE && !this->changeLanes) {
                this->overtakeDistance       = 2 * CHANGE_LANE_DIST;
                this->changeLanes            = true;
                this->savedTravelledDistance = this->travelledDistance;
                this->currentState           = State::CHANGE_LANE;
                this->changeLaneState        = LaneChange::CHANGE_LANE;
            }
            else if (!this->carDetected && msg->distance < DISTANCE_STOP_FOR_CAR && msg->area == MIDDLE_AREA) {
                this->carDetected   = true;
                this->distanceToCar = msg->distance;
                this->tStart        = ros::Time::now();
            }
        }
        else if (msg->sign_class == "round-about-sign" && this->validateDetection(roundaboutROI, msg->distance)) {
            if (!this->roundAboutSign) {
                this->roundAboutSign = true;
                this->currentState   = State::ROUNDABOUT;
            }
        }

        else if (msg->sign_class == "closed-road-stand" && this->validateDetection(roadBlockROI, msg->distance)) {
            if (!this->changeLanes && msg->distance < DISTANCE_TO_START_LANE_CHANGE && msg->area == MIDDLE_AREA && this->currentState == State::FOLLOW_LANE) {
                this->closedRoadStand        = true;
                this->changeLanes            = true;
                this->savedTravelledDistance = this->travelledDistance;
                this->changeLaneState        = LaneChange::CHANGE_LANE;
                this->currentState           = State::CHANGE_LANE;
            }
        }

        else if (msg->sign_class == "highway-entry-sign" && this->validateDetection(highwayEntriesROI, msg->distance)) {
            if (!this->onHighWay) this->onHighWay = true;
        }
        else if (msg->sign_class == "highway-exit-sign" && this->validateDetection(highwayExitsROI, msg->distance)) {
            if (this->onHighWay) {
                this->onHighWay = false;
            }
            if (!this->oneWayRoadSign && !this->oneWayRoadPassed) {
                this->oneWayRoadSign         = true;
                this->oneWayRoadPassed       = true;
                this->savedTravelledDistance = this->travelledDistance;
                this->distanceToObject       = msg->distance;
                this->currentState           = State::SINGLE_ONE_WAY;
            }
        }

        else if (msg->sign_class == "no-entry-road-sign" && this->validateDetection(noEntriesROI, msg->distance)) {
            if (!this->noEntryRoadSign) {
                this->noEntryRoadSign = true;
            }
        }

        else if (msg->sign_class == "one-way-road-sign") {
            if (!this->oneWayRoadSign && this->validateDetection(singleOneWayROI, msg->distance) && !this->oneWayRoadPassed) {
                this->oneWayRoadSign         = true;
                this->oneWayRoadPassed       = true;
                this->savedTravelledDistance = this->travelledDistance;
                this->distanceToObject       = msg->distance;
                this->currentState           = State::SINGLE_ONE_WAY;
            }
            else if (!this->oneWayRoadSign && this->validateDetection(doubleOneWayROI, msg->distance)) {
                this->oneWayRoadSign = true;
            }
        }

        else {
            ROS_INFO("SignClass unknown/ ROI not valid!");
        }
    }
}

void ITMOVES::Environment::laneDetectionCallback(const bfmc_interface::LaneDetectionConstPtr& msg) {
    this->curveCoeff  = msg->curveCoefficient;
    this->midDistance = msg->midDistance;
}

/**
 * Update current global Pose based on the delta of the travelled distance and the yaw angle.
 * This is done by using the conversion from polar to cartesian coordinates with respect to the current Pose.
 * Stop the vehicle by entering the state Finish if the maximum driving distance is reached.
 */

void ITMOVES::Environment::wheelSpeedCallback(const bfmc_interface::WheelSpeedConstPtr& msg) {
    static float oldDistance   = 0;
    float        deltaDistance = this->travelledDistance - oldDistance;
    oldDistance                = this->travelledDistance;
    this->currentPose.x        = deltaDistance * std::cos(this->currentPose.yaw) + this->currentPose.x;
    this->currentPose.y        = deltaDistance * std::sin(this->currentPose.yaw) + this->currentPose.y;
    this->currentPose.speed    = msg->speed;
    this->travelledDistance    = msg->totalDistance;

    if (this->travelledDistance >= this->distanceToDrive) this->currentState = State::FINISH;

    this->egoPoseMsg.header.stamp = ros::Time::now();
    this->egoPoseMsg.x            = this->currentPose.x;
    this->egoPoseMsg.y            = this->currentPose.y;
}

/**
 * Update the current yaw angle if the value is > 0 (IMU reads partially random negative or zero values) and limit the value between 0 and 2PI.
 * Set rampInfo according to the vehicle's pitch value and hardReset based on a roll angle threshold.
 */
void ITMOVES::Environment::imuCallback(const bfmc_interface::IMUConstPtr& msg) {
    if (msg->yaw > 0) this->currentPose.yaw = this->boundYawWithinZeroAnd2PI(msg->yaw + this->initialCarYaw);
    if (msg->pitch < PITCH_THRESHOLD_UPWARDS) this->rampInfo.drivingUpwards = true;
    else if (msg->pitch > PITCH_THRESHOLD_DOWNWARDS) this->rampInfo.drivingDownwards = true;
    else {
        this->rampInfo.drivingUpwards   = false;
        this->rampInfo.drivingDownwards = false;
    }
    if (std::abs(msg->roll) > 0.5f) this->hardReset = true;
    else this->hardReset = false;
    this->egoPoseMsg.yaw = this->currentPose.yaw;
}

void ITMOVES::Environment::trafficLightStartCallback(const bfmc_interface::TrafficLightConstPtr& msg) {
    // <node id="86">
    //   <data key="d0">0.83</data>
    //   <data key="d1">14.67</data>
    // Yaw needs to be within 240° - 300° - 270° +- 30°
    if (!this->startTrafficLightPassed) {
        // Update state of traffic light within State::TRAFFIC_LIGHT
        if (this->currentState == State::TRAFFIC_LIGHT) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
        }
        else if (this->isInROI(this->trafficLightStartROI, this->currentPose.x, this->currentPose.y) && (this->currentPose.yaw > CAR_FACES_NORTH - CAR_FACING_TOLERANCE) && (this->currentPose.yaw < CAR_FACES_NORTH + CAR_FACING_TOLERANCE)) {
            // Don't enter TrafficLight State right after leaving traffic light state - complete intersection maneuver
            if (this->currentState != State::INTERSECTION) {
                this->trafficLightState = static_cast<TrafficLight>(msg->state);
                this->currentState      = State::TRAFFIC_LIGHT;
            }
        }
    }
}

void ITMOVES::Environment::trafficLightEastCallback(const bfmc_interface::TrafficLightConstPtr& msg) {
    if (this->isInROI(this->trafficLightEastROI, this->currentPose.x, this->currentPose.y) && (this->currentPose.yaw > CAR_FACES_WEST - CAR_FACING_TOLERANCE) && (this->currentPose.yaw < CAR_FACES_WEST + CAR_FACING_TOLERANCE)) {
        // Don't enter TrafficLight State right after leaving traffic light state - complete intersection maneuver
        if (this->currentState != State::INTERSECTION) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
            this->currentState      = State::TRAFFIC_LIGHT;
        }
        // Update state of traffic light within State::TRAFFIC_LIGHT
        else if (this->currentState == State::TRAFFIC_LIGHT) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
        }
    }
}

void ITMOVES::Environment::trafficLightSouthCallback(const bfmc_interface::TrafficLightConstPtr& msg) {
    if (!this->southTrafficLightPassed) {
        if (this->currentState == State::TRAFFIC_LIGHT) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
        }
        else if (this->isInROI(this->trafficLightSouthROI, this->currentPose.x, this->currentPose.y) && (this->currentPose.yaw > CAR_FACES_NORTH - CAR_FACING_TOLERANCE) && (this->currentPose.yaw < CAR_FACES_NORTH + CAR_FACING_TOLERANCE)) {
            // Don't enter TrafficLight State right after leaving traffic light state - complete intersection maneuver
            if (this->currentState != State::INTERSECTION) {
                this->trafficLightState = static_cast<TrafficLight>(msg->state);
                this->currentState      = State::TRAFFIC_LIGHT;
            }
        }
    }
}

void ITMOVES::Environment::trafficLightWestCallback(const bfmc_interface::TrafficLightConstPtr& msg) {
    if (this->isInROI(this->trafficLightWestROI, this->currentPose.x, this->currentPose.y) && ((this->currentPose.yaw > CAR_FACES_EAST - CAR_FACING_TOLERANCE + 2 * M_PI) || (this->currentPose.yaw < CAR_FACES_EAST + CAR_FACING_TOLERANCE))) {
        // Don't enter TrafficLight State right after leaving traffic light state - complete intersection maneuver
        if (this->currentState != State::INTERSECTION) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
            this->currentState      = State::TRAFFIC_LIGHT;
        }
        // Update state of traffic light within State::TRAFFIC_LIGHT
        else if (this->currentState == State::TRAFFIC_LIGHT) {
            this->trafficLightState = static_cast<TrafficLight>(msg->state);
        }
    }
}

void ITMOVES::Environment::stateCallback(const std_msgs::UInt8ConstPtr& msg) {
    this->currentState = static_cast<State>(msg->data);
}

void ITMOVES::Environment::signObjectReset() {
    this->stopSign      = false;
    this->parkingSign   = false;
    this->crossWalkSign = false;
    this->prioritySign  = false;
    this->pedestrian.reset();
    this->carDetected            = false;
    this->closedRoadStand        = false;
    this->onHighWay              = false;
    this->noEntryRoadSign        = false;
    this->oneWayRoadSign         = false;
    this->roundAboutSign         = false;
    this->distanceToObject       = 0.0f;
    this->distanceToStopLine     = 0.0f;
    this->distanceToCar          = 0.0f;
    this->distanceToParkingSpot  = 0.0f;
    this->distanceToTrafficLight = 0.0f;
    this->changeLanes            = false;
    this->changeLaneState        = LaneChange::DEFAULT;
}

bool ITMOVES::Environment::validateDetection(const std::vector<DetectionROI> detectionROI, const float signDistance) {
    double globalX = this->currentPose.x + cos(this->currentPose.yaw) * signDistance;
    double globalY = this->currentPose.y + sin(this->currentPose.yaw) * signDistance;
    for (const auto& roi : detectionROI) {
        if (isInROI(roi, globalX, globalY)) {
            return true;
        }
        else return false;
    }
    return true;
}

bool ITMOVES::Environment::isInROI(const DetectionROI roi, const double globalX, const double globalY) {
    return (globalX >= roi.x1 && globalX <= roi.x2 && globalY >= roi.y1 && globalY <= roi.y2);
}

float ITMOVES::Environment::boundYawWithinZeroAnd2PI(float angle) {
    if (angle < 2 * M_PI) return angle;
    else return angle - 2 * M_PI;
}

void ITMOVES::Environment::publish() {
    this->egoPosePub.publish(this->egoPoseMsg);
}
