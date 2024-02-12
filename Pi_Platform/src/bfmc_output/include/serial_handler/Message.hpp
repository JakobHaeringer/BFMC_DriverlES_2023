/**
 * @file Message.hpp
 * @author Bosch Engineering GmbH
 * @brief The header file is for the message conversion to send data to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */

#ifndef _MESSAGE_HPP_
#define _MESSAGE_HPP_

#include <complex>
#include <sstream>
#include <stdio.h>
#include <string>

/** @brief This namespace holds the functions to create messages that can be sent over serial to the STM board.
 *
 * For this it needs a command and an information that tells the STM board what to do.*/
namespace message {
    // enum for defining the actions that can be performed
    //  1 - SPEED Command
    //  2 - STEERING Command
    //  3 - BRAKE Command
    //  4 - PID ACTIVATION Command
    //  5 - ENCODER PUBLISHER Command
    //  6 - PID TUNNING Command
    //  7 - NO Command

    /** @brief The string array that holds the identifier for all possible command types sent to the STM board. */
    static std::string ActionStrings[] = {"1", "2", "3", "4", "5", "6", "7"};
    /**
     * @brief Provide the cmd key associated to an action.
     *
     * @param enumVal The integer value corresponding to the action, in the action enumeration (zero-indexed).
     * @return std::string String associated to the action.
     */
    std::string getTextForKey(int);
    /**
     * @brief Construct the string to be sent, associated to speed action.
     *
     * @param f_velocity  The velocity in meter/second.
     * @return std::string Complete string for send command.
     */
    std::string speed(float);
    /**
     * @brief Construct the string to be sent, associated to steer action.
     *
     * @param f_angle  Angle.
     * @return std::string Complete string for send command.
     */
    std::string steer(float);
    /**
     * @brief Construct the string to be sent, associated to brake action.
     *
     * @param f_angle Angle.
     * @return std::string Complete string for send command.
     */
    std::string brake(float);
    /**
     * @brief Construct the string to be sent, associated to pid activating.
     *
     * @param activate  Set PID active or not.
     * @return std::string Complete string for send command.
     */
    std::string pida(bool);
    /**
     * @brief Construct the string to be sent, associated to encoder publisher activating.
     *
     * @param activate Set ENPB active or not.
     * @return std::string Complete string for send command.
     */
    std::string enpb(bool);
    /**
     * @brief Construct the string to be sent, associated to setting the pid values.
     *
     * @param kp          Param kp.
     * @param ki          Param ki.
     * @param kd          Param kd.
     * @param tf          Param tf.
     * @return std::string Complete string for send command.
     */
    std::string pids(float, float, float, float);
};

#endif
