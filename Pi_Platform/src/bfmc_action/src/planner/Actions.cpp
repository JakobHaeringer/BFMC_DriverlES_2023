/**
 * @file Actions.cpp
 * @author Jakob Haeringer and Noah Koehler
 * @brief The file implements actions used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */
#include "planner/Actions.hpp"
#include <ros/package.h>

#define DRIVE_STRAIGHT_DISTANCE          0.3f    ///< [m] Driving distance to go straight ahead - used for approaching intersections and crossing the crosswalk.
#define YAW_INTERSECTION_TOLERANCE       13      ///< [°] Tolerance for meeting the desired yaw angle while turning.
#define YAW_INTERSECTION_MULTIPLIER      0.2f    ///< [None] Multiplier for correcting the steering angle when the vehicle isn't perfectly orthogonal at the intersection.
#define FIRST_PARKING_SPOT_OFFSET        -1.28f  ///< [m] Distance to the second parking spot in respect to the position of the parking sign. (ES=-1.28f, Cluj=-1.58f)
#define SECOND_PARKING_SPOT_OFFSET       -0.28f  ///< [m] Distance to the first parking spot in respect to the position of the parking sign. (ES=-0.28f, Cluj=-0.58f)
#define DIST_PARK_IN                     1.35f   ///< [m] Driving distance when entering the parking spot.
#define DIST_PARK_OUT                    1.25f   ///< [m] Driving distance when reversing out of the parking spot.
#define PARKING_TIME                     2.0f    ///< [s]  Time the vehicle waits in the parking spot.
#define WAIT_AFTER_PEDESTRIAN_LEFT       2.0f    ///< [s] Time the vehicle waits before proceeding after the pedestrian has left the road/crosswalk.
#define CURVE_COEFFICIENT_DEFAULT_VALUE  0.0025f ///< [None] Smallest curve coefficient for determining the P-values.// was 0.00125
#define CURVE_COEFFICIENT_DRIVE_STRAIGHT 0.0015f ///< [None] Curve coefficient representing a straight lane.// was 0.00125
#define FOLLOWCAR_MIN_DISTANCE           0.5f    ///< [m] Minimal distance of a car ahead at which the minimum speed is set.
#define FOLLOWCAR_MAX_DISTANCE           1.5f    ///< [m] Maximum distance of a car ahead at which the maximum speed is set.
#define WAIT_CAR_CANT_CHANGE_LANE        2.0f    ///< [s] Time to reset the detected vehicle to check if the sign detection still detects a car.
#define ROUNDABOUT_ENTER_DISTANCE        0.6f    ///< [m] Driving distance for static driving of the roundabout.
#define ROUNDABOUT_EXIT_DISTANCE         0.4f    ///< [m] Driving distance for static driving of the roundabout.
#define ROUNDABOUT_INSIDE_DISTANCE       1.75f   ///< [m] Driving distance for static driving of the roundabout.

ITMOVES::Actions::Actions() {
    this->AStar.getReadFile().setFilename(ros::package::getPath("bfmc_action") + "/config/planner/competition_track.graphml");
    this->AStar.getReadFile().readFile();
    this->P    = 0;
    this->oldP = 0;
}

ITMOVES::Actions::~Actions() {}

void ITMOVES::Actions::StartUp(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    double elapsedTime = (ros::Time::now() - env->tStart).toSec();
    if (elapsedTime > this->duration.startUp) {
        ROS_INFO("LEAVE STARTUP");
        env->tStart = ros::Time::now();
        this->RoutePlanning(env);
        env->currentState = State::FOLLOW_LANE;
    }
}

/**
 * Speed value is set to the default value if the vehicle has been stopped. If the vehicle is on the highWay the speed is increased.
 * To obtain the correct steering angle, a P value is determined based on the curve coefficient. The P value gets multiplied by the distance to the center of the lane and a transition factor. If the curve coefficient is unreasonable high, the old P value is used.
 * Emergency braking is performed when a vehicle is within a safety distance. The detected vehicle is reset after 2s to check if the vehicle is still there, otherwise the vehicle continues.
 */
void ITMOVES::Actions::FollowLane(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->onHighWay) cmd->speed = cmd->speedHighWay;
    if (cmd->speed == SPEED_NEUTRAL) cmd->speed = cmd->speedDefault;

    if (std::abs(env->curveCoeff) <= CURVE_COEFFICIENT_DEFAULT_VALUE) this->P = 0.8;
    else if (std::abs(env->curveCoeff) <= 0.005) this->P = 1.1;
    else if (std::abs(env->curveCoeff) <= 0.01) this->P = 1.25;
    else if (std::abs(env->curveCoeff) <= 0.35) this->P = 1.6;
    else this->P = this->oldP;
    this->oldP = this->P;

    cmd->steering = env->midDistance * TRANSITION_FACTOR * P;
    if (cmd->steering > STEERING_MAX_RIGHT) cmd->steering = STEERING_MAX_RIGHT;
    else if (cmd->steering < STEERING_MAX_LEFT) cmd->steering = STEERING_MAX_LEFT;

    if (env->carDetected && env->distanceToCar < DISTANCE_STOP_FOR_CAR) {
        double elapsedTime = (ros::Time::now() - env->tStart).toSec();
        cmd->speed         = SPEED_NEUTRAL;
        if (elapsedTime >= WAIT_CAR_CANT_CHANGE_LANE) env->carDetected = false;
    }
}

/**
 * Approach the crosswalk at reduced speed while running "FollowLane" to obtain the correct steering angle. Switch to the state Pedestrian when a pedestrian is detected and save the current time (for maximum pedestrian time).
 * Otherwise, drive 30cm straight ahead to cross the crosswalk, as the lane detection causes unpredictable behavior of the vehicle at crosswalks and exit the function by entering the state FollowLane.
 */
void ITMOVES::Actions::Crosswalk(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine) {
        cmd->speed = cmd->speedReduced;
        this->FollowLane(env, cmd);
    }
    else {
        if (env->pedestrian.detected) {
            env->tStart       = ros::Time::now();
            env->currentState = State::PEDESTRIAN;
        }
        else {
            if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine + DRIVE_STRAIGHT_DISTANCE) {
                cmd->steering = STEERING_NEUTRAL;
            }
            else {
                ROS_INFO("RESET CROSSWALK");
                env->signObjectReset();
                env->currentState = State::FOLLOW_LANE;
            }
        }
    }
}

/**
 * Approach the pedestrian at reduced speed while running "FollowLane" to obtain the correct steering angle. In addition, the current time is saved.
 * When the pedestrian is at a safe distance, stop the vehicle by setting the speed and steering to neutral.
 * Depending on the entered and current location(area) of the pedestrian and the previous state, the vehicle will continue.
 * For the previous state beeing the crosswalk, there are two conditions that will cause the vehicle to continue.
 * First, if the pedestrian is detected on the left side (enteredArea), it will wait until the pedestrian is on the right side (currentArea).
 * And second, if the pedestrian is detected on the right side or in the middle, wait until the pedestrian is on the left side.
 * If the pedestrian is detected on the road, the entered area of the pedestrian must be unequal to the current area to proceed; this only works sufficiently if the pedestrian is detected in the middle of the frame.
 * Regardless of the previous state, the time is saved when the conditions to continue are met, and the vehicle waits for another 2s before exiting the function.
 * As a safety backup, a maximum time of 15s (duration.pedestrian) also causes the function to exit.
 * If the car is at a crosswalk before exiting the function, the lane detection causes the car to behave unpredictably, so the speed value is set to default and the steering value is set to go straight. Exit the function by entering the state FollowLane.
 */
void ITMOVES::Actions::Pedestrian(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToObject) {
        cmd->speed  = cmd->speedReduced;
        env->tStart = ros::Time::now();
        this->FollowLane(env, cmd);
    }
    else if (cmd->speed != SPEED_NEUTRAL && !env->pedestrian.crossedRoad) {
        cmd->speed    = SPEED_NEUTRAL;
        cmd->steering = STEERING_NEUTRAL;
    }
    else {
        if (env->pedestrian.enteredArea == LEFT_AREA && env->pedestrian.currentArea == RIGHT_AREA && !env->pedestrian.crossedRoad && env->crossWalkSign) {
            env->tStart                 = ros::Time::now();
            env->pedestrian.crossedRoad = true;
        }
        else if ((env->pedestrian.enteredArea == MIDDLE_AREA || env->pedestrian.enteredArea == RIGHT_AREA) && env->pedestrian.currentArea == LEFT_AREA && !env->pedestrian.crossedRoad && env->crossWalkSign) {
            env->tStart                 = ros::Time::now();
            env->pedestrian.crossedRoad = true;
        }
        else if (env->pedestrian.enteredArea != env->pedestrian.currentArea && !env->crossWalkSign && !env->pedestrian.crossedRoad) {
            env->tStart                 = ros::Time::now();
            env->pedestrian.crossedRoad = true;
        }
        double elapsedTime = (ros::Time::now() - env->tStart).toSec();
        if (elapsedTime > this->duration.pedestrian) env->pedestrian.crossedRoad = true;

        if (env->pedestrian.crossedRoad && elapsedTime > WAIT_AFTER_PEDESTRIAN_LEFT) {
            if ((env->travelledDistance < env->savedTravelledDistance + env->distanceToObject + DRIVE_STRAIGHT_DISTANCE) && env->crossWalkSign) {
                cmd->speed    = cmd->speedDefault;
                cmd->steering = STEERING_NEUTRAL;
            }
            else {
                ROS_INFO("RESET PEDESTRIAN");
                env->pedestrian.reset();
                env->crossWalkSign = false;
                env->currentState  = State::FOLLOW_LANE;
            }
        }
    }
}

/**
 * Drive to the stop line with reduced speed using the "FollowLane" function to obtain the correct steering angle.
 * Steer the vehicle straight ahead for the last 30 cm, otherwise the lane detection will try to turn because of the rounded intersections.
 * If the lane detection does not encounter a rounded intersection due to the current curve coefficient, continue correcting the steering angle.
 * Save the current time while approaching the stop line to slow down the vehicle at the stop line and wait there for 3 seconds.
 * Check which direction to turn at the intersection before exiting the function.
 * Exit the function by entering the state FollowLane if the curve coefficient is straight and the vehicle should go straight.
 * Otherwise, the yaw angle of the vehicle at the intersection is determined, the travelled distance is saved, and the state Intersection is entered.
 */
void ITMOVES::Actions::Stop(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine - DRIVE_STRAIGHT_DISTANCE) {
        this->FollowLane(env, cmd);
        cmd->speed  = cmd->speedReduced;
        env->tStart = ros::Time::now();
    }
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine) {
        if (std::abs(env->curveCoeff) < CURVE_COEFFICIENT_DRIVE_STRAIGHT) this->FollowLane(env, cmd);
        else cmd->steering = STEERING_NEUTRAL;
        cmd->speed  = cmd->speedReduced;
        env->tStart = ros::Time::now();
    }
    else {
        double elapsedTime = (ros::Time::now() - env->tStart).toSec();
        cmd->speed         = SPEED_NEUTRAL;
        cmd->steering      = STEERING_NEUTRAL;
        if (elapsedTime > this->duration.stop) {
            ROS_INFO("RESET STOP");
            env->signObjectReset();
            getDirectionToDrive(env);
            if (std::abs(env->curveCoeff) < CURVE_COEFFICIENT_DRIVE_STRAIGHT && env->intersectionCrossing == STRAIGHT) env->currentState = State::FOLLOW_LANE;
            else {
                env->intersectionStartYaw   = int(env->currentPose.yaw * 180 / M_PI) % 90;
                env->savedTravelledDistance = env->travelledDistance;
                env->currentState           = State::INTERSECTION;
            }
        }
    }
}

/**
 * Drive to the stop line using the "FollowLane" function to obtain the correct steering angle.
 * Steer the vehicle straight ahead for the last 30 cm, otherwise the lane detection will try to turn because of the rounded intersections.
 * If the lane detection does not encounter a rounded intersection due to the current curve coefficient, continue correcting the steering angle.
 * Check which direction to turn at the intersection before exiting the function.
 * Exit the function by entering the state FollowLane if the curve coefficient is straight and the vehicle should go straight.
 * Otherwise, the yaw angle of the vehicle at the intersection is determined, the travelled distance is saved, and the state Intersection is entered.
 */
void ITMOVES::Actions::PriorityRoad(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine - DRIVE_STRAIGHT_DISTANCE) {
        this->FollowLane(env, cmd);
    }
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine) {
        if (std::abs(env->curveCoeff) < CURVE_COEFFICIENT_DRIVE_STRAIGHT) this->FollowLane(env, cmd);
        else cmd->steering = STEERING_NEUTRAL;
    }
    else {
        ROS_INFO("RESET PRIORITY ROAD");
        env->signObjectReset();
        getDirectionToDrive(env);
        if (std::abs(env->curveCoeff) < CURVE_COEFFICIENT_DRIVE_STRAIGHT && env->intersectionCrossing == STRAIGHT) env->currentState = State::FOLLOW_LANE;
        else {
            env->intersectionStartYaw   = int(env->currentPose.yaw * 180 / M_PI) % 90;
            env->savedTravelledDistance = env->travelledDistance;
            env->currentState           = State::INTERSECTION;
        }
    }
}

/**
 * Depending on the value of the env->intersectionCrossing a static route vector with steering commands is assigned.
 * Iterates over the static route until the next continuing node is found depending on the distance travelled.
 * Depending on the yaw difference (normalized to 0-90°), a correction factor is added to the steering angle to compensate for skewed vehicles at the intersection.
 * If the vehicle has stopped before entering the intersection state, a high PWM signal is given to the motor to overcome the inertia of the motor.
 * The function is exited with entering the state FollowLane when the desired yaw angle (+- 13°) is reached (applies only to left/right turns) and at least all but three values of the static route have been driven.
 */
void ITMOVES::Actions::Intersection(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    switch (env->intersectionCrossing) {
    case STRAIGHT:
        env->staticIntersectionRoute = env->straightIntersection;
        break;
    case LEFT_TURN:
        env->staticIntersectionRoute = env->leftTurn;
        break;
    case RIGHT_TURN:
        env->staticIntersectionRoute = env->rightTurn;
        break;
    }

    while (env->travelledDistance >= env->savedTravelledDistance + env->staticIntersectionRoute[env->localRouteIterator].totalDistance) {
        if (env->localRouteIterator >= env->staticIntersectionRoute.size() - 1) break;
        else env->localRouteIterator++;
    }

    if (env->intersectionStartYaw > 70) env->intersectionYawOffset = YAW_INTERSECTION_MULTIPLIER * (90.0 - env->intersectionStartYaw);
    else if (env->intersectionStartYaw < 20) env->intersectionYawOffset = -YAW_INTERSECTION_MULTIPLIER * env->intersectionStartYaw;
    else env->intersectionYawOffset = 0.0;

    cmd->steering = env->staticIntersectionRoute[env->localRouteIterator].steeringAngle + env->intersectionYawOffset;
    cmd->speed    = cmd->speedDefault;

    if (env->localRouteIterator >= env->staticIntersectionRoute.size() - 3 && ((((int(env->currentPose.yaw * 180 / M_PI) % 90) - env->intersectionStartYaw) <= YAW_INTERSECTION_TOLERANCE) || env->intersectionCrossing == STRAIGHT)) {
        ROS_INFO("RESET INTERSECTION");
        env->localRouteIterator = 0;
        env->currentState       = State::FOLLOW_LANE;
    }
}

/**
 * Calculate the shortest route between the currentNodeID and the targetNodeID using an AStar algorithm.
 * The planned route contains all nodes (ID, x and y coordinates, a boolean value for dashed or solid lane).
 * For each node, the sum of the distances between each node, starting with zero at the first node, is calculated and added to the planned route data.
 */
void ITMOVES::Actions::RoutePlanning(ITMOVES::Environment* env) {
    env->globalRouteIterator = 0;
    env->plannedRoute.clear();

    this->AStar.aStarAlgorithm(env->currentNodeID, env->targetNodeID, env);
    for (unsigned long int i = 0; i < env->plannedRoute.size(); i++) {
        if (i == 0)
            env->plannedRoute[i].totalDistance = 0;
        else
            env->plannedRoute[i].totalDistance = env->plannedRoute[i - 1].totalDistance +
                                                 std::sqrt(std::pow(env->plannedRoute[i].x - env->plannedRoute[i - 1].x, 2) + std::pow(env->plannedRoute[i].y - env->plannedRoute[i - 1].y, 2));
        ROS_INFO("ID: %d\t x: %f\t y:%f, Dist %f", env->plannedRoute[i].nodeID, env->plannedRoute[i].x, env->plannedRoute[i].y, env->plannedRoute[i].totalDistance);
    }
}

/**
 * Turns the vehicle from the highway to the one-way street and then follows a car and controls the speed based on the distance to the car ahead.
 * To maintain the correct steering angle, the "FollowLane" function is used.
 * The vehicle exits this function when the stop sign is detected at the end of the road (the state will change by itself).
 */
void ITMOVES::Actions::SingleOneWay(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    this->FollowLane(env, cmd);

    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToObject) {
        this->FollowLane(env, cmd);
        if (cmd->steering < -4.0) cmd->steering = STEERING_NEUTRAL;
    }

    float distanceToCar = std::clamp(env->distanceToCar, FOLLOWCAR_MIN_DISTANCE, FOLLOWCAR_MAX_DISTANCE);
    float interpolant   = (distanceToCar - FOLLOWCAR_MIN_DISTANCE) / (FOLLOWCAR_MAX_DISTANCE - FOLLOWCAR_MIN_DISTANCE);
    cmd->speed          = SPEED_NEUTRAL + (cmd->speedMax - SPEED_NEUTRAL) * interpolant;
}

/**
 * Finds the free parking spot (cross parking) based on the distance between the parking sign (env->distanceToObject) and the detected vehicle (env->distanceToCar) in the parking spot.
 * If the distance is less than one meter (Esslingen) or 20cm (Cluj-Napoca), the closer parking spot is occupied.
 * Depending on the free parking spot env->distanceToParkingSpot is set.
 * The vehicle drives the remaining distance with the function "FollowLane" and then turns left into the parking spot.
 * The vehicle waits 2s in the parking spot and then reverses out of the parking spot and leaves the function with entering the state FollowLane.
 */
void ITMOVES::Actions::ParkCar(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (!env->carIsParked) {
        // Choosing the free parking spot according to the distances on the testtrack in Esslingen
        /*if (env->distanceToCar - env->distanceToObject <= 1.0 && env->distanceToParkingSpot == 0) {
            env->distanceToParkingSpot = SECOND_PARKING_SPOT_OFFSET + env->distanceToCar + env->savedTravelledDistance;
        }
        else if (env->distanceToCar - env->distanceToObject > 1.0 && env->distanceToParkingSpot == 0) {
            env->distanceToParkingSpot = FIRST_PARKING_SPOT_OFFSET + env->distanceToCar + env->savedTravelledDistance;
        }*/
        // Choosing the free parking spot according to the distances on the competition track in Cluj-Napoca
        if (env->distanceToCar - env->distanceToObject <= 0.2 && env->distanceToParkingSpot == 0) {
            env->distanceToParkingSpot = SECOND_PARKING_SPOT_OFFSET + env->distanceToCar + env->savedTravelledDistance;
        }
        else if (env->distanceToCar - env->distanceToObject > 0.2 && env->distanceToParkingSpot == 0) {
            env->distanceToParkingSpot = FIRST_PARKING_SPOT_OFFSET + env->distanceToCar + env->savedTravelledDistance;
        }
        if (env->travelledDistance <= env->distanceToParkingSpot) {
            env->savedTravelledDistance = env->travelledDistance;
            this->FollowLane(env, cmd);
        }
        else if (env->travelledDistance <= env->savedTravelledDistance + DIST_PARK_IN) {
            cmd->speed    = cmd->speedParking;
            cmd->steering = -20.0;
        }
        else {
            env->carIsParked            = true; // Do not reset this parameter, otherwise the vehicle may try to park again.
            env->savedTravelledDistance = env->travelledDistance;
            cmd->speed                  = SPEED_NEUTRAL;
            cmd->steering               = STEERING_NEUTRAL;
            env->tStart                 = ros::Time::now();
        }
    }
    else {
        double elapsedTime = (ros::Time::now() - env->tStart).toSec();
        if (elapsedTime >= PARKING_TIME) {
            if (env->travelledDistance < env->savedTravelledDistance + DIST_PARK_OUT) {
                cmd->speed    = -cmd->speedParking;
                cmd->steering = -21.5;
            }
            else {
                ROS_INFO("RESET PARKING");
                env->signObjectReset();
                cmd->speed        = cmd->speedDefault;
                env->currentState = State::FOLLOW_LANE;
            }
        }
    }
}

/**
 * In this function the variable env->changeLaneState is used to change between the states of a lane change.
 * With this argument it is possible to change the lane and return to the original lane or to change only to the other lane e.g. in case of a road block.
 * For each state of the env->changeLaneState, different steering and speed commands are set based on the travelled distance.
 */
void ITMOVES::Actions::ChangeLane(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    switch (env->changeLaneState) {
    case LaneChange::CHANGE_LANE:
        if (env->travelledDistance < env->savedTravelledDistance + CHANGE_LANE_DIST) {
            cmd->steering = -20.0;
        }
        else if (env->travelledDistance < env->savedTravelledDistance + 1.9 * CHANGE_LANE_DIST)
            cmd->steering = 20.0;
        else {
            env->savedTravelledDistance = env->travelledDistance;
            env->changeLaneState        = LaneChange::BACK_TO_ORIGINAL;
            // env->changeLaneState        = LaneChange::STAY_ON_LANE;
        }
        break;
    case LaneChange::STAY_ON_LANE:
        if (env->closedRoadStand) {
            ROS_INFO("CHANGE LANE COMPLETED");
            env->closedRoadStand = false;
            env->changeLanes     = false;
            env->currentState    = State::FOLLOW_LANE;
            env->changeLaneState = LaneChange::DEFAULT;
            break;
        }
        else {
            if (env->travelledDistance < env->savedTravelledDistance + env->overtakeDistance)
                this->FollowLane(env, cmd);
            else {
                env->savedTravelledDistance = env->travelledDistance;
                env->changeLaneState        = LaneChange::BACK_TO_ORIGINAL;
            }
            break;
        }
    case LaneChange::BACK_TO_ORIGINAL:
        if (env->travelledDistance < env->savedTravelledDistance + CHANGE_LANE_DIST)
            cmd->steering = 20.0;
        else if (env->travelledDistance < env->savedTravelledDistance + 1.7 * CHANGE_LANE_DIST)
            cmd->steering = -20.0;
        else {
            env->savedTravelledDistance = env->travelledDistance;
            env->changeLaneState        = LaneChange::COMPLETED;
        }
        break;
    case LaneChange::COMPLETED:
        ROS_INFO("CHANGE LANE COMPLETED");
        env->changeLanes     = false;
        env->currentState    = State::FOLLOW_LANE;
        env->changeLaneState = LaneChange::DEFAULT;
        break;
    default:
        break;
    }
    cmd->speed = cmd->speedDefault;
}

/**
 * Approach the stop line before the traffic light with the "FollowLane" function. Steer neutrally for the last 30cm (env->distanceToStopLine) to stand straight on the stop line.
 * Brake at the stop line and continue driving when the car has stopped and the traffic light is green.
 * When exiting the function, the turning direction and the yaw angle of the vehicle at the intersection are determined.
 * Furthermore, the travelled distance is saved and the state is changed to Intersection.
 */
void ITMOVES::Actions::TrafficLight(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    // Approach traffic light with follow lane
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine - DRIVE_STRAIGHT_DISTANCE) {
        this->FollowLane(env, cmd); // Continue running FollowLane to get correct steering angle
        cmd->speed  = cmd->speedReduced;
        env->tStart = ros::Time::now();
    }
    // Drive straight for the last 30 cm to stand straight before entering intersection
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine) {
        cmd->steering = STEERING_NEUTRAL;
        cmd->speed    = cmd->speedReduced;
        env->tStart   = ros::Time::now();
    }
    // Break on stop line
    else {
        double elapsedTime = (ros::Time::now() - env->tStart).toSec();
        cmd->speed         = SPEED_NEUTRAL;
    }
    // If the car has stopped and the traffic light is green switch into State::INTERSECTION
    if (env->currentPose.speed == SPEED_NEUTRAL && env->trafficLightState == TrafficLight::GREEN) {
        env->signObjectReset();
        getDirectionToDrive(env);
        env->intersectionStartYaw    = int(env->currentPose.yaw * 180 / M_PI) % 90;
        env->savedTravelledDistance  = env->travelledDistance;
        env->startTrafficLightPassed = true;
        env->currentState            = State::INTERSECTION;
    }
}

void ITMOVES::Actions::Roundabout(ITMOVES::Environment* env, ITMOVES::Command* cmd) {
    if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine) {
        this->FollowLane(env, cmd);
    }
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine + ROUNDABOUT_ENTER_DISTANCE) {
        cmd->steering = 22.0;
    }
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine + ROUNDABOUT_INSIDE_DISTANCE) {
        cmd->steering = -21.0;
    }
    else if (env->travelledDistance < env->savedTravelledDistance + env->distanceToStopLine + ROUNDABOUT_INSIDE_DISTANCE + ROUNDABOUT_EXIT_DISTANCE) {
        cmd->steering = 22.0;
    }
    else {
        env->signObjectReset();
        env->currentState = State::FOLLOW_LANE;
    }
}

void ITMOVES::Actions::getDirectionToDrive(ITMOVES::Environment* env) {
    auto node0 = env->plannedRoute[env->globalRouteIterator];
    auto node1 = env->plannedRoute[env->globalRouteIterator + 1];
    auto node2 = env->plannedRoute[env->globalRouteIterator + 2];

    float dir1 = std::atan2(node1.y - node0.y, node1.x - node0.x) * 180 / M_PI;
    float dir2 = std::atan2(node2.y - node1.y, node2.x - node1.x) * 180 / M_PI;

    float diff = dir2 - dir1;
    diff       = diff - (std::ceil((diff + 180) / 360) - 1) * 360;

    env->intersectionCrossing = STRAIGHT;
    if (diff < -30) env->intersectionCrossing = LEFT_TURN;
    if (diff > 30) env->intersectionCrossing = RIGHT_TURN;
}

void ITMOVES::Actions::resetSystem(ITMOVES::Environment* env) {
    ROS_INFO("HARD RESET");
    env->signObjectReset();

    env->trafficLightState = TrafficLight::UNDEFINED;
    env->changeLaneState   = LaneChange::DEFAULT;

    env->startTrafficLightPassed   = true;
    env->rampInfo.drivingDownwards = false;
    env->rampInfo.drivingUpwards   = false;
    env->hardReset                 = false;

    env->tStart = ros::Time::now();

    env->savedTravelledDistance = 0.0f;
    env->distanceToObject       = 0.0f;
    env->distanceToStopLine     = 0.0f;
    env->distanceToCar          = 0.0f;
    env->distanceToParkingSpot  = 0.0f;
    env->distanceToTrafficLight = 0.0f;
    env->overtakeDistance       = 0.0f;
}
