/**
 * @file Environment.hpp
 * @author Jakob Haeringer and Noah Koehler
 * @brief The header file for the environment class used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */
#ifndef _ENVIRONMENT_HPP_
#define _ENVIRONMENT_HPP_

#define CHANGE_LANE_DIST      0.5f       ///< [m] Driving distance for overtaking another vehicle.
#define DISTANCE_STOP_FOR_CAR 0.5f       ///< [m] Minimum distance to a car ahead.
#define CAR_FACES_EAST        0          ///< [°] 0° Car faces east on global map.
#define CAR_FACES_SOUTH       M_PI_2     ///< [°] 90° Car faces south on global map.
#define CAR_FACES_WEST        M_PI       ///< [°] 180° Car faces west on global map.
#define CAR_FACES_NORTH       3 * M_PI_2 ///< [°] 270° Car faces north on global map.
#define CAR_FACING_TOLERANCE  M_PI / 6   ///< [°] 30° Tolerance to determine which way the car faces.

#include "bfmc_interface/EgoPose.h"
#include "bfmc_interface/IMU.h"
#include "bfmc_interface/LaneDetection.h"
#include "bfmc_interface/SignDetection.h"
#include "bfmc_interface/TrafficLight.h"
#include "bfmc_interface/WheelSpeed.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <tuple>

/** @brief Enumeration class containing the vehicle states which are used to choose between the execution of the "Actions" functions.*/
enum class State : uint8_t {
    START_UP       = 0,  ///< State for the start.
    FOLLOW_LANE    = 1,  ///< State to follow the lane.
    INTERSECTION   = 2,  ///< State for intersections.
    STOP           = 3,  ///< State for the stop sign.
    CROSSWALK      = 4,  ///< State for the crosswalk.
    PARKING        = 5,  ///< State for parking maneuvers.
    ROUTE_PLANNING = 6,  ///< State for planning a route.
    PRIORITY_ROAD  = 7,  ///< State for the priority sign.
    PEDESTRIAN     = 8,  ///< State for handling pedestrians.
    SINGLE_ONE_WAY = 9,  ///< State for driving a single one-way street.
    CHANGE_LANE    = 10, ///< State for lane change.
    TRAFFIC_LIGHT  = 11, ///< State for traffic lights.
    ROUNDABOUT     = 12, ///< State for driving through a roundabout.
    FINISH         = 31  ///< State for finishing.
};

/** @brief Enumeration class containing the IDs of the objects that must be sent to the environment server.*/
enum class LiveTrafficSystemID : u_int8_t {
    stop                  = 1,  ///< ID for the stop sign.
    priority              = 2,  ///< ID for the priority sign.
    parking               = 3,  ///< ID for the parking sign.
    crosswalk             = 4,  ///< ID for the crosswalk sign.
    highwayEntrance       = 5,  ///< ID for the highway entry sign.
    highwayExit           = 6,  ///< ID for the highway exit sign.
    roundabout            = 7,  ///< ID for the roundabout sign.
    oneWayRoad            = 8,  ///< ID for the one way road sign.
    trafficLight          = 9,  ///< ID for the traffic light.
    staticCarOnRoad       = 10, ///< ID for a static car on the road.
    staticCarOnParking    = 11, ///< ID for a static car on the parking spot.
    pedestrianOnCrosswalk = 12, ///< ID for a pedestrian on the crosswalk.
    pedestrianOnRoad      = 13, ///< ID for a pedestrian on the road.
    roadblock             = 14, ///< ID for a road block.
    bumpyRoad             = 15  ///< ID for the ramp.
};

/** @brief Enumeration class containing the traffic light states.*/
enum class TrafficLight : uint8_t {
    RED       = 0, ///< Red traffic light.
    YELLOW    = 1, ///< Yellow traffic light.
    GREEN     = 2, ///< Green traffic light.
    UNDEFINED = 3  ///< Undefined traffic light.
};

/** @brief Enumeration of states for turning at intersections.*/
enum IntersectionCrossing {
    STRAIGHT               = 0,  ///< Go straight across the intersection.
    LEFT_TURN              = 1,  ///< Turn left at the intersection.
    RIGHT_TURN             = 2,  ///< Turn right at the intersection.
    UNDEFINED_INTERSECTION = 255 ///< Undefined intersection state.
};

/** @brief Enumeration of the area in the image where the objects are detected.*/
enum AreaOfObjects {
    LEFT_AREA      = 0,  ///< Detection in the left area of the image.
    MIDDLE_AREA    = 1,  ///< Detection in the middle of the image.
    RIGHT_AREA     = 2,  ///< Detection in the right area of the image.
    UNDEFINED_AREA = 255 ///< Detection in an undefined area.
};

/** @brief Enumeration class containing the states for a lane change.*/
enum class LaneChange : uint8_t {
    CHANGE_LANE      = 0,  ///< State when changing from the original lane.
    STAY_ON_LANE     = 1,  ///< State when staying on the other lane.
    BACK_TO_ORIGINAL = 2,  ///< State when changing back to the original lane.
    COMPLETED        = 3,  ///< State when the lane change is completed.
    DEFAULT          = 255 ///< Default state for the lane change.
};

/** @brief Struct that holds the current pose of the vehicle.*/
struct Pose {
    int   nodeID; ///< Current node ID on the global map.
    float x;      ///< Current x coordinate on the global map.
    float y;      ///< Current y coordinate on the global map.
    float yaw;    ///< Current yaw angle on the global map.
    float speed;  ///< Current calculated speed.
};

/** @brief Struct that is used for information about the planned route.*/
struct PlannedRoute {
    int    nodeID;            ///< NodeID of one element of the planned route.
    double x;                 ///< X coordinate of one element of the planned route.
    double y;                 ///< Y coordinate of one element of the planned route.
    bool   changeLineAllowed; ///< Indicates whether a lane change is allowed.
    double totalDistance;     ///< Total driving distance from the first element of the planned route to the current element.
};

/** @brief Struct that holds information about driving up/down the ramp.*/
struct Ramp {
    bool drivingUpwards;   ///< The vehicle drives up the ramp.
    bool drivingDownwards; ///< The vehicle drives down the ramp.
};

/** @brief Struct used to store information about static intersection vectors.*/
struct IntersectionRoute {
    double totalDistance; ///< Driving distance from the first element to the current element of the intersection vector.
    double steeringAngle; ///< Steering angle of the current element of the intersection vector.
};

/** @brief Struct used to store information about detected pedestrians.*/
struct Pedestrian {
    bool    detected;    ///< Boolean value indicating the detected pedestrian.
    uint8_t enteredArea; ///< Entered area of the detected pedestrian in the frame.
    uint8_t currentArea; ///< Current area of the detected pedestrian in the frame.
    bool    crossedRoad; ///< Boolean value indicating the pedestrian has crossed the road.
    /** @brief Reset the struct Pedestrian. */
    void reset() {
        detected    = false;
        enteredArea = UNDEFINED_AREA;
        currentArea = UNDEFINED_AREA;
        crossedRoad = false;
    }
};
/** @brief Struct for defining region of interests in the shape of rectangles to specify where the respective object classes appear on the global map.*/
struct DetectionROI {
    double x1; ///< X coordinate of the left upper corner of the rectangle.
    double y1; ///< Y coordinate of the left upper corner of the rectangle.
    double x2; ///< X coordinate of the bottom right corner of rectangle.
    double y2; ///< Y coordinate of the bottom right corner of rectangle.
};

/** @brief Namespace ITMOVES contains the Environment, Actions and Command classes which are used to perform the behavior planning of the vehicle during the BFMC 2023.*/
namespace ITMOVES {
    /** @brief This class combines all values from the input sensors, processes them further and then sets the vehicle states accordingly.*/
    class Environment {
    public:
        /**
         * @brief Construct a new Environment object and initialize ROS Subscriber/ Publisher, various class members, intersection vectors and ROIs for the objects.
         * @param handle Nodehandler for creating the ROS publishers and subscribers.
         */
        Environment(ros::NodeHandle& handle);
        /** @brief Destroy the Environment object.*/
        ~Environment();

        /** @brief Initialize the region of interests in the shape of rectangles to specify where the respective object classes appear on the global map.*/
        void initROIs();

        /** @defgroup GroupEgo Vehicle state
         * Initial and current information about position and state of the vehicle and the ego pose publisher.@{ */
        float      initialCarYaw;     ///< [°] Yaw angle of the vehicle at the start.
        State      currentState;      ///< Current state for selecting the execution of the actions.
        Pose       currentPose;       ///< Current ego pose (nodeID, x, y, yaw, speed)
        Pose       gpsEgoPose;        ///< GPS Ego position
        float      travelledDistance; ///< [m] Travelled distance based on the wheel speed sensors.
        Ramp       rampInfo;          ///< RampInfo based on the pitch value of the IMU.
        bool       onHighWay;         ///< Currently on the highway.
        bool       changeLanes;       ///< Currently changing lanes.
        bool       carIsParked;       ///< Vehicle has completed a parking maneuver.
        LaneChange changeLaneState;   ///< Holds the current state of the lane change.
        ros::Time  tStart;            ///< [Timestamp] Holds a timestamp that is checked against.
        /** @brief Ego pose publisher*/
        void                    publish();
        bfmc_interface::EgoPose egoPoseMsg; ///< Ego pose message [x, y, yaw, nodeID]
        /** @} */

        /** @defgroup GroupObjects Signs/ Objects/ Lane Detection
         * Stores the information about currently detected signs, objects and the information from the lane detection.@{ */
        bool                              stopSign;               ///< stopSign
        bool                              parkingSign;            ///< parkingSign
        bool                              crossWalkSign;          ///< crossWalkSign
        bool                              prioritySign;           ///< prioritySign
        Pedestrian                        pedestrian;             ///< struct Pedestrian
        bool                              carDetected;            ///< carDetected
        bool                              closedRoadStand;        ///< closedRoadStand
        bool                              noEntryRoadSign;        ///< noEntryRoadSign
        bool                              oneWayRoadSign;         ///< oneWayRoadSign
        bool                              roundAboutSign;         ///< roundAboutSign
        TrafficLight                      trafficLightState;      ///< trafficLightState
        std::unordered_map<uint8_t, Pose> competitorCarPose;      ///< Position of other cars
        float                             distanceToObject;       ///< [m] Distance to an object (sign, pedestrian).
        float                             distanceToStopLine;     ///< [m] Distance to a stop line.
        ros::Time                         timeStopLineDetected;   ///< [Timestamp] Timestamp of the last detected stop line.
        float                             distanceToCar;          ///< [m] Distance to a car.
        float                             distanceToParkingSpot;  ///< [m] Distance to a parking spot.
        float                             distanceToTrafficLight; ///< [m] Distance to a traffic light.
        float                             overtakeDistance;       ///< [m] Distance for overtaking a car.
        float                             savedTravelledDistance; ///< [m] Holds a timestamp of the travelled distance that is checked against.
        float                             distanceToDrive;        ///< [m] Maximum driving distance before entering the state Finish.
        float                             curveCoeff;             ///< Current curve coefficient
        float                             midDistance;            ///< [m] Current distance to the middle of the lane
        /** @brief This function will reset all detected signs and objects as well as their saved distances and specific variables.*/
        void signObjectReset();
        /** @} */

        /** @defgroup GroupIntersection Routeplanning/ Intersection variables
         * Variables to store and execute route planning and intersection crossing.@{ */
        std::vector<PlannedRoute>      plannedRoute;            ///< Currently planned route.
        int                            currentNodeID;           ///< Current node ID to start route planning from.
        int                            targetNodeID;            ///< Target node ID to plan the route to.
        long unsigned int              localRouteIterator;      ///< Iteratator for intersection vector.
        long unsigned int              globalRouteIterator;     ///< Iteratator for global planned route.
        int                            intersectionCrossing;    ///< Enumeration value (IntersectionCrossing) how to cross the intersection.
        unsigned int                   intersectionStartYaw;    ///< [°] Yaw angle, how the vehicle is positioned on the intersection.
        float                          intersectionYawOffset;   ///< [°] Yaw angle offset added to the steering angle based on the vehicles position at the intersection.
        std::vector<IntersectionRoute> staticIntersectionRoute; ///< Static intersection vector.
        std::vector<IntersectionRoute> rightTurn;               ///< Static vector for the right turn.
        std::vector<IntersectionRoute> leftTurn;                ///< Static vector for the left turn.
        std::vector<IntersectionRoute> straightIntersection;    ///< Static vector for the crossing straight over.
        /** @} */

        /** @defgroup GroupROI ROI of the objects
         * Holds the information about all ROIs for each object.@{ */
        std::vector<DetectionROI> crosswalksROI;        ///< ROIs of the crosswalks.
        std::vector<DetectionROI> parkingROI;           ///< ROI of the parking spots.
        std::vector<DetectionROI> roundaboutROI;        ///< ROI of the roundabout.
        std::vector<DetectionROI> noEntriesROI;         ///< ROIs of the no entries signs.
        std::vector<DetectionROI> roadBlockROI;         ///< ROI of the road block.
        std::vector<DetectionROI> highwayExitsROI;      ///< ROIs of the highway exit signs.
        std::vector<DetectionROI> highwayEntriesROI;    ///< ROIs of the highway entry signs.
        std::vector<DetectionROI> singleOneWayROI;      ///< ROIs of the one way sign (single - countryroad).
        std::vector<DetectionROI> doubleOneWayROI;      ///< ROIs of the one way sign (double - roadblock).
        std::vector<DetectionROI> intersectionSignsROI; ///< ROIs of the intersections.
        DetectionROI              trafficLightStartROI; ///< ROI of the start traffic light.
        DetectionROI              trafficLightSouthROI; ///< ROI of the south traffic light.
        DetectionROI              trafficLightEastROI;  ///< ROI of the east traffic light.
        DetectionROI              trafficLightWestROI;  ///< ROI of the west traffic light.
        /** @} */

        bool startTrafficLightPassed;
        bool southTrafficLightPassed;
        bool oneWayRoadPassed;
        bool raceMode;
        bool hardReset;

    private:
        std::vector<ros::Subscriber> subs;       ///< Vector containing all environment subscribers.
        ros::Publisher               egoPosePub; ///< Publisher for the ego pose [x, y, yaw, nodeID].

        /**
         * @brief Set states according to detected signs/ objects within the correct ROI and store specific variables triggered by the topic "/perception/signdetection".
         * @param msg Contains the message data (sign_class | x_bounding_box ,y_bounding_box width_bounding_box height_bounding_box confidence distance area).
         */
        void signDetectionCallback(const bfmc_interface::SignDetectionConstPtr& msg);

        /**
         * @brief Update the current distance to the center and the curvature coefficient of the lane triggered by the topic "/perception/lanedetection".
         * @param msg Contains the message data (midDistance curveCoefficient).
         */
        void laneDetectionCallback(const bfmc_interface::LaneDetectionConstPtr& msg);

        /**
         * @brief Update current global Pose based on the travelled distance and yaw angle triggered by the topic "/automobile/wheelspeed" and stop the vehicle after reaching the maximum distance.
         * @param msg Contains the message data (wheelspeedLeft wheelspeedRight speed totalDistance).
         */

        void wheelSpeedCallback(const bfmc_interface::WheelSpeedConstPtr& msg);
        /**
         * @brief Update the current yaw angle (> 0), detect driving upwards/downwards and detect the reset of the system based on the topic "/automobile/imu".
         * @param msg Contains the message data (roll pitch yaw accelx accely accelz).
         * */

        void imuCallback(const bfmc_interface::IMUConstPtr& msg);

        // Callback for the traffic light on start - - car needs to face to the north
        void trafficLightStartCallback(const bfmc_interface::TrafficLightConstPtr& msg);
        // Callback for the traffic light on the east of the map - car needs to face to the west
        void trafficLightEastCallback(const bfmc_interface::TrafficLightConstPtr& msg);
        // Callback for the traffic light on the south of the map - car needs to face to the north
        void trafficLightSouthCallback(const bfmc_interface::TrafficLightConstPtr& msg);
        // Callback for the traffic light on the west of the map - car needs to face to the east
        void trafficLightWestCallback(const bfmc_interface::TrafficLightConstPtr& msg);

        /**
         * @brief Sets the current state based on topic "/testState".
         * @param msg Contains the state request.
         */
        void stateCallback(const std_msgs::UInt8ConstPtr& msg);
        /**
         * @brief Determines if the detected object is within the corresponding ROIs.
         * @param detectionROI ROIs of the detected object
         * @param signDistance Distance to the detected object
         * @return true if the detected object is within the corresponding ROIs
         * @return false if the detected object is not within the corresponding ROIs
         */
        bool validateDetection(const std::vector<DetectionROI> detectionROI, const float signDistance);
        /**
         * @brief Determines if point(x,y) is inside a given ROI.
         * @param roi ROI to check
         * @param globalX X coordinate of point
         * @param globalY Y coordinate of point
         * @return true if point(x,y) is inside ROI
         * @return false if point(x,y) is not inside ROI
         */
        bool isInROI(const DetectionROI roi, double globalX, double globalY);
        /**
         * @brief Normalizes the yaw angle between 0 and 2PI.
         * @param angle Yaw angle
         * @return Yaw angle between 0 and 2PI
         */
        float boundYawWithinZeroAnd2PI(float angle);
    };
}
#endif
