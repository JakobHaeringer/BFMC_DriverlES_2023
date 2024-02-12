/**
 * @file serial_handler_node.cpp
 * @author Jakob Haeringer and Noah Koehler
 * @brief This file holds the logic for converting the ROS command messages to messages that can be sent to the STM board.
 * @version 1.0
 * @date 2023-07-17
 */
#include "serial_handler/serial_handler_node.hpp"

serialHandlerNode::serialHandlerNode(SerialComManager& f_comManager, ResponseHandler& f_reponseHandler) :
    m_comManager(f_comManager),
    m_responseHandler(f_reponseHandler) {}

serialHandlerNode::~serialHandlerNode() {
    m_responseHandler.detach(std::string("1"), m_callbackFncObj);
    m_responseHandler.detach(std::string("2"), m_callbackFncObj);
    m_responseHandler.detach(std::string("3"), m_callbackFncObj);
    m_responseHandler.detach(std::string("4"), m_callbackFncObj);
    m_responseHandler.detach(std::string("5"), m_callbackFncObj);
    m_responseHandler.detach(std::string("6"), m_callbackFncObj);

    delete m_callbackFncObj;
    // Close all threads
    m_comManager.closeAll();
}

void serialHandlerNode::print(std::string str) {
    // std::cout<<str<<std::endl;
}

void serialHandlerNode::init(ros::NodeHandle* nh) {
    m_callbackFncObj = ResponseHandler::createCallbackFncPtr(&serialHandlerNode::print, this);

    // Attach the callback function to the following messages.
    m_responseHandler.attach(std::string("1"), m_callbackFncObj);
    m_responseHandler.attach(std::string("2"), m_callbackFncObj);
    m_responseHandler.attach(std::string("3"), m_callbackFncObj);
    m_responseHandler.attach(std::string("4"), m_callbackFncObj);
    m_responseHandler.attach(std::string("5"), m_callbackFncObj);
    m_responseHandler.attach(std::string("6"), m_callbackFncObj);

    Subscribing = nh->subscribe("/output/actuator_command", 1, &serialHandlerNode::funcCallback, this);

    // PID speed control
    m_comManager.sendPidParam(0.05, 0.9, 0.000222, 0.040000);
    m_comManager.sendPidState(1);
}

void serialHandlerNode::funcCallback(const bfmc_interface::NucleoCommandConstPtr& msg) {
    uint8_t command = msg->command_type;
    float   value   = msg->value;

    if (command) {
        if (command == 1) m_comManager.sendSpeed(value);
        else if (command == 2) m_comManager.sendSteer(value);
        else if (command == 3) m_comManager.sendBrake(value);
        else ROS_INFO("Unknown command type");
    }
    else ROS_INFO("Command not set");
}

int main(int argc, char** argv) {
#ifndef ITMOVES_SIMULATOR
    ros::init(argc, argv, "serial_handler");
    ros::NodeHandle nh;

    ResponseHandler   l_responseHandler;
    SerialComManager  l_communicationManager(l_responseHandler);
    serialHandlerNode commandObject(l_communicationManager, l_responseHandler);

    commandObject.init(&nh);

    ros::spin();
#else
    return 0;
#endif
}
