/**
 * @file Message.cpp
 * @author Bosch Engineering GmbH
 * @brief This file implements the message conversion for the commands sent to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#include "serial_handler/Message.hpp"

std::string message::getTextForKey(int enumVal) {
    return ActionStrings[enumVal];
}

std::string message::speed(float f_velocity) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_velocity);
    strs << "#" << getTextForKey(0) << ":" << buff;
    return strs.str();
}

std::string message::steer(float f_angle) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
    strs << "#" << getTextForKey(1) << ":" << buff;
    return strs.str();
}

std::string message::brake(float f_angle) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
    strs << "#" << getTextForKey(2) << ":" << buff;
    return strs.str();
}

std::string message::pida(bool activate) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%d;;\r\n", activate);
    strs << "#" << getTextForKey(3) << ":" << buff;
    return strs.str();
}

std::string message::enpb(bool activate) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%d;;\r\n", activate);
    strs << "#" << getTextForKey(4) << ":" << buff;
    return strs.str();
}

std::string message::pids(float kp, float ki, float kd, float tf) {
    std::stringstream strs;
    char              buff[100];
    snprintf(buff, sizeof(buff), "%.5f;%.5f;%.5f;%.5f;;\r\n", kp, ki, kd, tf);
    strs << "#" << getTextForKey(5) << ":" << buff;
    return strs.str();
}
