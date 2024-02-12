/**
 * @file Command.hpp
 * @author Jakob Haeringer
 * @brief The header file for the output commands used in the BFMC 2023.
 *
 * @details Config file for steering and dc motor constraints on the Nucleo Board in BFMC/Embedded_Platform/src/main.cpp. \n
 * Steering constraints -22° bis 22° because of the chassis - -23 to +23 on Nucleo Board. \n
 * Speed constraints - Signal for DC Motor -0.3 to 0.3 (m/s) on Nucleo Board.
 * @version 1.0
 * @date 2023-07-13
 */
#ifndef _COMMAND_HPP_
#define _COMMAND_HPP_

#ifndef ITMOVES_SIMULATOR
    #include "bfmc_interface/NucleoCommand.h"
#else
    #include "bfmc_interface/SimulatorActuator.h"
#endif

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#define STEERING_NEUTRAL   0.0f   ///< [°] Neutral steering angle.
#define STEERING_MAX_LEFT  -22.0f ///< [°] Maximum left steering angle.
#define STEERING_MAX_RIGHT 22.0f  ///< [°] Maximum right steering angle.

#define SPEED_NEUTRAL 0.0f ///< [m/s] Neutral speed.

/** @brief Enumeration containing the command types.*/
enum COMMANDS {
    COMMAND_SPEED = 1, ///< Specifies that the message sent to the Nucleo contains a speed value.
    COMMAND_STEER = 2, ///< Specifies that the message sent to the Nucleo contains a steering signal.
    COMMAND_BRAKE = 3  ///< Specifies that the message sent to the Nucleo contains a signal to stop the vehicle.
};

/** @brief Namespace ITMOVES contains the Environment, Actions and Command classes which are used to perform the behavior planning of the vehicle during the BFMC 2023.*/
namespace ITMOVES {
    /** @brief This class contains the publisher for the steering and speed commands and holds various speed values.*/
    class Command {
    public:
        /**
         * @brief Construct a new Command object and initialize publisher as well as variables which are not set in the launch file.
         * @param handle Nodehandler for creating the ROS publishers and subscribers.
         */
        Command(ros::NodeHandle& handle);
        /** @brief Destroy the Command object. */
        ~Command();
        /** @brief Publishes either a command to stop the vehicle or a steering and speed command.*/
        void publish();

        float speedDefault; ///< Holds the default speed value specified in the launch file.
        float speedParking; ///< Holds the parking speed value specified in the launch file.
        float speedReduced; ///< Holds the reduced speed value specified in the launch file.
        float speedHighWay; ///< Holds the highway speed value specified in the launch file.
        float speedMax;     ///< Holds the maximum speed value specified in the launch file.

        float speed;       ///< Holds the value of the currently desired speed.
        float steering;    ///< Holds the value of the currently desired steering angle.
        bool  stopVehicle; ///< Holds the boolean value whether the vehicle should be stopped.

    private:
        ros::Publisher publisher; ///< The publisher object that is used to publish the "utils::nucleocommand" message to the topic "/automobile/command".
#ifndef ITMOVES_SIMULATOR
        bfmc_interface::NucleoCommand msg; ///< The message object that is used as a template for the ROS nucleocommand message.
#else
        bfmc_interface::SimulatorActuator msg;
#endif
    };
}
#endif
