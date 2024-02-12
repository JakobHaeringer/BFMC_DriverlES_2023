/**
 * @file wheel_speed_node.cpp
 * @author Jakob Haeringer
 * @brief This file implements the logi to handle the wheelspeed sensor data.
 * @version 1.0
 * @date 2023-07-17
 */
#define _USE_MATH_DEFINES
#include <math.h>
#include <optional>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#ifdef ITMOVES_SIMULATOR
    #include "bfmc_interface/SimulatorWheelSpeed.h"
#else
    #include <wiringPi.h>
#endif

#include "bfmc_interface/WheelSpeed.h"

// For wiringPi Pin Layout see https://www.digikey.com/en/maker/blogs/2019/how-to-use-gpio-on-the-raspberry-pi-with-c
/** @brief GPIO number referring to the look up table in the wiringPi library.*/
#define GPIO_WHEELSPEED_RIGHT 4
/** @brief GPIO number referring to the look up table in the wiringPi library.*/
#define GPIO_WHEELSPEED_LEFT 5
/** @brief Number of signal flanks for one entire rotation of a wheel. */
#define WHEEL_TICKS_PER_ROTATION 16
/** @brief [Meter] Diameter of the wheel*/
#define WHEEL_DIAMETER 0.062
/** @brief [Meter] Circumference of the wheel. */
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
/** @brief [Meter] Traveled distance per registered flank.*/
#define DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_ROTATION)

/** @brief The message object that is used as a template for the ROS wheelspeed message. */
bfmc_interface::WheelSpeed wheelspeedMsg;

/**
 * @brief This class implements the logic to receive the data from the wheel speed sensors and to provide them to the ROS network running on the Raspberry Pi Platform.
 *
 * First the mode in which the GPIO pins will run is set to input. After that a callback function is assigned
 * to the GPIO that gets triggered when a flank is registered on the GPIO by the wheelspeed sensor.
 * The callback function increases a counter by +1 every time the function is called.
 */
class WheelSpeed {
public:
    /** @brief The number of counted flanks for the left wheel since the instantiation of the WheelSpeed object.*/
    static uint32_t totalWheelTicksLeft;
    /** @brief The number of counted flanks for the right wheel since the instantiation of the WheelSpeed object.*/
    static uint32_t totalWheelTicksRight;

    /**
     * @brief Construct a new Wheel Speed object and initialize the GPIO pins, as well as the ROS publisher.
     *
     * @param nodeHandle The ROS node handler object which is supposed to publish the data.
     */
    WheelSpeed(ros::NodeHandle& nodeHandle) {
#ifdef ITMOVES_SIMULATOR
        frontLeftWheelSpeedSubscriber = nodeHandle.subscribe<bfmc_interface::SimulatorWheelSpeed>(
            "/simulator/wheel_speed/front_left", 1, &WheelSpeed::frontLeftWheelSpeedCallback, this);
        frontRightWheelSpeedSubscriber = nodeHandle.subscribe<bfmc_interface::SimulatorWheelSpeed>(
            "/simulator/wheel_speed/front_right", 1, &WheelSpeed::frontRightWheelSpeedCallback, this);
#else
        // Set up wiringPi Setup of included library
        wiringPiSetup();
        // Set connected GPIO's on Pi to Input
        pinMode(GPIO_WHEELSPEED_LEFT, INPUT);
        pinMode(GPIO_WHEELSPEED_RIGHT, INPUT);
        // Initialize interrupt routine for GPIOs
        wiringPiISR(GPIO_WHEELSPEED_LEFT, INT_EDGE_BOTH, &wheelTicksLeftCallback);
        wiringPiISR(GPIO_WHEELSPEED_RIGHT, INT_EDGE_BOTH, &wheelTicksRightCallback);
#endif
    }

private:
#ifdef ITMOVES_SIMULATOR
    ros::Subscriber frontLeftWheelSpeedSubscriber;
    ros::Subscriber frontRightWheelSpeedSubscriber;

    void frontLeftWheelSpeedCallback(const bfmc_interface::SimulatorWheelSpeedConstPtr& msg) {
        static std::optional<ros::Time> lastTimestamp;
        wheelspeedMsg.wheelspeedLeft = msg->value;

        if (!lastTimestamp.has_value()) {
            lastTimestamp = msg->header.stamp;
            return;
        }

        extern float distanceLeft;
        auto         elapsedTime = msg->header.stamp - lastTimestamp.value();
        distanceLeft += abs(msg->value) * elapsedTime.toSec();
        lastTimestamp = msg->header.stamp;
    }

    void frontRightWheelSpeedCallback(const bfmc_interface::SimulatorWheelSpeedConstPtr& msg) {
        static std::optional<ros::Time> lastTimestamp;
        wheelspeedMsg.wheelspeedRight = msg->value;

        if (!lastTimestamp.has_value()) {
            lastTimestamp = msg->header.stamp;
            return;
        }

        extern float distanceRight;
        auto         elapsedTime = msg->header.stamp - lastTimestamp.value();
        distanceRight += abs(msg->value) * elapsedTime.toSec();
        lastTimestamp = msg->header.stamp;
    }
#else
    /** @brief To increase the number of flanks that occurred on the left wheel.*/
    static void wheelTicksLeftCallback() {
        totalWheelTicksLeft += 1;
    }
    /** @brief To increase the number of flanks that occurred on the right wheel.*/
    static void wheelTicksRightCallback() {
        totalWheelTicksRight += 1;
    }
#endif
};

#ifdef ITMOVES_SIMULATOR
float distanceLeft  = 0.0f;
float distanceRight = 0.0f;
#else
uint32_t WheelSpeed::totalWheelTicksLeft  = 0;
uint32_t WheelSpeed::totalWheelTicksRight = 0;
#endif

/**
 * @brief Given the wheel ticks during each iteration the travelled distance and the speed of the car and each wheel is calculated and published on the ROS network.
 * @param argc
 * @param argv
 * @return int 0
 */
int main(int argc, char** argv) {
#ifndef ITMOVES_SIMULATOR
    uint32_t ticksOldLeft       = 0;
    uint32_t ticksOldRight      = 0;
    uint32_t deltaTicksLeft     = 0;
    uint32_t deltaTicksRight    = 0;
    float    deltaDistanceLeft  = 0;
    float    deltaDistanceRight = 0;
    float    distanceRight      = 0;
    float    distanceLeft       = 0;
#endif

    ros::init(argc, argv, "wheel_speed");
    ros::NodeHandle nh;
    ros::Publisher  wheelspeedPub = nh.advertise<bfmc_interface::WheelSpeed>("/input/wheel_speed", 1);

    WheelSpeed wheelspeedObject(nh);

    float     hz = 20.0f;
    ros::Rate loop_rate(hz);

    ros::Time tStart;
    ros::Time tEnd;

    while (ros::ok()) {
        tStart = ros::Time::now();

#ifndef ITMOVES_SIMULATOR
        deltaTicksLeft  = wheelspeedObject.totalWheelTicksLeft - ticksOldLeft;
        deltaTicksRight = wheelspeedObject.totalWheelTicksRight - ticksOldRight;
        ticksOldLeft    = wheelspeedObject.totalWheelTicksLeft;
        ticksOldRight   = wheelspeedObject.totalWheelTicksRight;

        deltaDistanceLeft  = deltaTicksLeft * DISTANCE_PER_TICK;
        deltaDistanceRight = deltaTicksRight * DISTANCE_PER_TICK;
        distanceRight += deltaDistanceRight;
        distanceLeft += deltaDistanceLeft;

        wheelspeedMsg.header.stamp    = ros::Time::now();
        wheelspeedMsg.wheelspeedLeft  = deltaDistanceLeft * hz;
        wheelspeedMsg.wheelspeedRight = deltaDistanceRight * hz;
#endif

        wheelspeedMsg.header.stamp  = ros::Time::now();
        wheelspeedMsg.totalDistance = (distanceLeft + distanceRight) * 0.5;
        wheelspeedMsg.speed         = (wheelspeedMsg.wheelspeedLeft + wheelspeedMsg.wheelspeedRight) * 0.5;

        // ROS_INFO("Distance Driven: %f, speed: %f", wheelspeedMsg.totalDistance, wheelspeedMsg.speed);

        wheelspeedPub.publish(wheelspeedMsg);

        tEnd = ros::Time::now();
        // double elapsedTime = (tEnd - tStart).toSec() * 1000; // conversion to ms
        // ROS_INFO("Wheelspeed: %f", elapsedTime);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
