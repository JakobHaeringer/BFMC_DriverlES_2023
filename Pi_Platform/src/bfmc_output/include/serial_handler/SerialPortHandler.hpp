/**
 * @file SerialPortHandler.hpp
 * @author Bosch Engineering GmbH
 * @brief The header file holds the class for sending commands to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#ifndef SERIAL_PORT_HANDLER_HPP
#define SERIAL_PORT_HANDLER_HPP

#include "serial_handler/ResponseHandler.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace std;

/** @brief Different ways a serial port may be flushed. */
enum flush_type {
    flush_receive = TCIFLUSH,
    flush_send    = TCOFLUSH,
    flush_both    = TCIOFLUSH
};

/**
 * @brief This class handles the sending of the messages to the STM board via serial and it forwards the received messages to the ResponseHandler.
 *
 * The lower level functionality that is responsible for handling the messages is implemented in this serialPortHandler. It is responsible for
 * reading/writing messages from/to the buffer. It also establishes/closes the connection with the STM board. The serialPortHandler is used by
 * the higher level class serialCommunicationHandler.
 */
class serialPortHandler {
public:
    /**
     * @brief Construct a new serial Port Handler object that opens a serial port and performs a flush for both receive and transmit buffers.
     *
     * @param io_service Object that provides core I/O functionality.
     * @param baud Baud rate.
     * @param device Device name (listed in /dev folder on RPi).
     * @param responseHandler Object that provides response message processing.
     */
    serialPortHandler(boost::asio::io_service& io_service, unsigned int baud, const string& device, BaseResponseHandler& responseHandler);

    /**
     * @brief Sends data over UART, calls the do_write function via the io service in the other thread.
     *
     * @param smg  Message to be sent.
     */
    void write(std::string);

    /** @brief call the do_close function via the io service in the other thread*/
    void close();

    /** @brief Return the state of the socket.
     *
     * @param return True if the socket is still active
     */
    bool active();

    // Measure time
    ros::Time tStart;
    ros::Time tEnd;

private:
    /** @brief Maximum length of bytes that is read in one step. */
    static const int max_read_length = 512;
    /** @brief Start an asynchronous read and call read_complete when it completes or fails. */
    void read_start(void);
    /** @brief The asynchronous read operation has now completed or failed and returns an error. */
    void read_complete(const boost::system::error_code& error, size_t bytes_transferred);
    /** @brief Callback to handle write call from outside this class. */
    void do_writeString(std::string);
    /** @brief Start an asynchronous write and call write_complete when it completes or fails. */
    void write_start(void);
    /** @brief The asynchronous read operation has now completed or failed and returned an error. */
    void write_complete(const boost::system::error_code& error);
    /** @brief Something has gone wrong, so close the socket & make this object inactive. */
    void do_close(const boost::system::error_code& error);
    /**
     * @brief Flush a serial port's buffers.
     *
     * @param serial_port Port to flush.
     * @param what Determines the buffers to flush.
     * @param error Set to indicate what error occurred, if any.
     */
    void flush_serial_port(boost::asio::serial_port& serial_port, flush_type what, boost::system::error_code& error);

private:
    /** @brief Object response handler processes the response received. */
    BaseResponseHandler& m_responseHandler;
    /** @brief Object avoids the parallel writing. */
    boost::mutex m_writeMtx;
    /** @brief Remains true while this object is still operating.*/
    bool active_;
    /** @brief The main IO service that runs this connection.*/
    boost::asio::io_service& io_service_;
    /** @brief The serial port this instance is connected to. */
    boost::asio::serial_port serialPort;
    /** @brief Buffered write data. */
    deque<char> write_msgs_;

public:
    /** @brief Data read from the socket. */
    char read_msg_[max_read_length];
};

#endif
