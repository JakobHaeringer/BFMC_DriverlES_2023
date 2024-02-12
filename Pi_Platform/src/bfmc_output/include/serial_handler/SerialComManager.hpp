/**
 * @file SerialComManager.hpp
 * @author Bosch Engineering GmbH
 * @brief The header file holds the class for sending commands to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#ifndef SERIAL_COM_MANAGER_HPP
#define SERIAL_COM_MANAGER_HPP

#include "serial_handler/Message.hpp"
#include "serial_handler/ResponseHandler.hpp"
#include "serial_handler/SerialPortHandler.hpp"
#include <boost/asio.hpp>

/**
 * @brief This class holds the functions that sends commands to the STM board.
 *
 * By opening a connection to the STM board via serial, the Raspberry Pi is able to send commands to the STM board, which will the execute them and
 * apply it to the actuators on the vehicle. For this custom messages are designed, that contain a key and a body. The key identifies what command
 * to execute and the body holds the information on how to control the actuator.
 */
class SerialComManager {
public:
    /**
     * @brief Construct a new Serial Com Manager object and establish a connection to the desired device if possible.
     *
     * @param f_baudrate The baudrate that should be used for the communication.
     * @param f_dev The device name to which a connection should be established.
     * @param f_responseHandler A ResponseHandler object, that will deal with the feedback acquired from the STM board.
     */
    SerialComManager(unsigned int, const std::string, BaseResponseHandler&);

    /** @brief Construct a new Serial Com Manager object. */
    SerialComManager(BaseResponseHandler&);

    /** @brief Destroy the Serial Com Manager object. */
    virtual ~SerialComManager();

    /** @brief Closes the connection to the STM board. */
    void closeAll();

    /** @brief Send the desired speed to the STM board
     *
     * @param f_vel The desired velocity value.
     */
    void sendSpeed(float);
    /** @brief Send the desired steering angle to the STM board
     *
     * @param f_ster The desired steering angle value.
     */
    void sendSteer(float);
    /** @brief Send the desired brake value to the STM board
     *
     * @param f_angle The desired brake value.
     */
    void sendBrake(float);
    /** @brief Send the desired PID state to the STM board
     *
     * @param f_activate The desired PID state value.
     */
    void sendPidState(bool);
    /** @brief Send the desired encoder publisher state to the STM board
     *
     * @param f_activate The desired encoder publisher state value.
     */
    void sendEncoderPublisher(bool);
    /** @brief Send the desired settings for the pid controller to the STM board
     *
     * @param f_kp The desired proportional influence of the PID.
     * @param f_ki The desired integral influence of the PID.
     * @param f_kd The desired differential influence of the PID.
     * @param f_tf The desired time frequency of the PID.
     */
    void sendPidParam(float, float, float, float);

private:
    /** @brief The ResponseHandler object that handles the responses the Raspberry Pi receives to each sent message. */
    BaseResponseHandler& m_responseHandler;
    /** @brief The handler of the serial connection thread. */
    boost::asio::io_service m_io_service;
    /** @brief The thread for the transmitting part of the communication with the STM board. */
    boost::thread* m_io_serviceThread;
    /** @brief The thread for the receiving part of the communication with the STM board. */
    boost::thread* m_responseHandlerThread;
    /** @brief The SerialPortHandler that sends messages to the STM board. */
    serialPortHandler m_serialPort;
};

#endif
