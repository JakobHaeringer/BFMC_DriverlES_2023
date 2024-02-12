/**
 * @file Command.cpp
 * @author Jakob Haeringer
 * @brief This file implements the output commands used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */
#include "planner/Command.hpp"

ITMOVES::Command::Command(ros::NodeHandle& handle) {
#ifndef ITMOVES_SIMULATOR
    this->publisher = handle.advertise<bfmc_interface::NucleoCommand>("/output/actuator_command", 1);
#else
    this->publisher = handle.advertise<bfmc_interface::SimulatorActuator>("/simulator/actuator", 1);
#endif
    this->speed       = SPEED_NEUTRAL;
    this->steering    = STEERING_NEUTRAL;
    this->stopVehicle = false;
}

ITMOVES::Command::~Command() {}

void ITMOVES::Command::publish() {
#ifndef ITMOVES_SIMULATOR
    if (this->stopVehicle) {
        msg.header.stamp = ros::Time::now();
        msg.command_type = COMMAND_BRAKE;
        msg.value        = 0;
        publisher.publish(msg);
    }
    else {
        msg.command_type = COMMAND_STEER;
        msg.value        = this->steering;
        publisher.publish(msg);

        msg.header.stamp = ros::Time::now();
        msg.command_type = COMMAND_SPEED;
        msg.value        = this->speed;
        publisher.publish(msg);
    }
#else
    if (this->stopVehicle) {
        msg.header.stamp = ros::Time::now();
        msg.id           = "motor";
        msg.values.clear();
        msg.values.emplace_back(0);
        publisher.publish(msg);
    }
    else {
        msg.header.stamp = ros::Time::now();
        msg.id           = "motor";
        msg.values.clear();
        msg.values.emplace_back(this->speed);
        publisher.publish(msg);

        msg.header.stamp = ros::Time::now();
        msg.id           = "steering";
        msg.values.clear();
        msg.values.emplace_back(this->steering);
        publisher.publish(msg);
    }
#endif
}
