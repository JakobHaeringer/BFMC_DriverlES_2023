/**
 * @file serial_handler_node.hpp
 * @author Bosch Engineering GmbH
 * @brief The header file holds the class for converting the ROS command messages to messages that can be sent to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */

#include "bfmc_interface/NucleoCommand.h"
#include "serial_handler/Message.hpp"
#include "serial_handler/ResponseHandler.hpp"
#include "serial_handler/SerialComManager.hpp"
#include "serial_handler/SerialPortHandler.hpp"
#include <boost/function.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief This class implements the translation from the Raspberry Pi ROS messages to messages that the STM board can handle.
 *
 * Since the STM board isn't running ROS, the data that are transmitted to the STM board first need to be mapped from the ROS message
 * type to a custom message type. For this, the ROS topic command is being subscribed. By distinguishing the command the handler then triggers the function
 * that sends the command to the STM board.
 */
class serialHandlerNode {
public:
    /** @brief Construct a new serial Handler Node object. */
    serialHandlerNode(SerialComManager&, ResponseHandler&);
    /** @brief Destroy the serial Handler Node object and detach all the actions from the responseHandler callback. */
    ~serialHandlerNode();
    /** @brief The ROS subscriber that receives the command messages. */
    ros::Subscriber Subscribing;
    /**
     * @brief Subscribe to the ROS command messages and attach all the actions to the responseHandler callback.
     *
     * @param nh The ROS NodeHandler that is used for subscribing to the command topic.
     */
    void init(ros::NodeHandle*);
    /**
     * @brief The callback function that send a command to the STM board upon receiving a command by the ROS subscriber.
     *
     * @param msg The ROS message that has been received by the subscriber.
     */
    void funcCallback(const bfmc_interface::NucleoCommandConstPtr& msg);
    /**
     * @brief When receiving an error from the STM board as an answer to a command, the function outputs the error message.
     *
     * @param str The error that the STM board send.
     */
    void print(std::string str);

private:
    /** @brief The communication manager that sends commands to the STM board. */
    SerialComManager& m_comManager;
    /** @brief The response handler that receives error messages from the STM board. */
    ResponseHandler& m_responseHandler;
    /** @brief The callback function pointer that holds the address of the function to execute when an error message is received. */
    ResponseHandler::CallbackFncPtrType m_callbackFncObj;
};
