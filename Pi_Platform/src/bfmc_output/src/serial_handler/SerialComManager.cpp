/**
 * @file SerialComManager.cpp
 * @author Bosch Engineering GmbH
 * @brief This file holds the logic for sending commands to the STM board vie Serial.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#include "serial_handler/SerialComManager.hpp"

SerialComManager::SerialComManager(BaseResponseHandler& f_responseHandler) :
    SerialComManager(19200, "/dev/ttyACM0", f_responseHandler) {
}

SerialComManager::SerialComManager(unsigned int f_baudrate, const std::string f_dev, BaseResponseHandler& f_responseHandler) :
    m_responseHandler(f_responseHandler), m_io_service(), m_io_serviceThread(NULL), m_responseHandlerThread(NULL), m_serialPort(m_io_service, f_baudrate, f_dev, m_responseHandler) {
    m_io_serviceThread      = new boost::thread(boost::bind(&boost::asio::io_service::run, &m_io_service));
    m_responseHandlerThread = new boost::thread(boost::bind(&BaseResponseHandler::_run, &m_responseHandler));
}

SerialComManager::~SerialComManager() {
    delete m_io_serviceThread;
}

void SerialComManager::closeAll() {
    m_serialPort.close();
    m_responseHandler.deactive();
    m_responseHandlerThread->join();
    m_io_serviceThread->join();
}

void SerialComManager::sendSpeed(float f_vel) {
    // Measure time
    m_serialPort.tStart = ros::Time::now();

    std::string l_msg = message::speed(f_vel);
    m_serialPort.write(l_msg);
}

void SerialComManager::sendSteer(float f_ster) {
    // Measure time
    m_serialPort.tStart = ros::Time::now();

    std::string l_msg = message::steer(f_ster);
    m_serialPort.write(l_msg);
}

void SerialComManager::sendBrake(float f_angle) {
    std::string l_msg = message::brake(f_angle);
    m_serialPort.write(l_msg);
}

void SerialComManager::sendPidState(bool f_activate) {
    std::string l_msg = message::pida(f_activate);
    m_serialPort.write(l_msg);
}

void SerialComManager::sendEncoderPublisher(bool f_activate) {
    std::string l_msg = message::enpb(f_activate);
    m_serialPort.write(l_msg);
}

void SerialComManager::sendPidParam(float f_kp, float f_ki, float f_kd, float f_tf) {
    std::string l_msg = message::pids(f_kp, f_ki, f_kd, f_tf);
    m_serialPort.write(l_msg);
}
