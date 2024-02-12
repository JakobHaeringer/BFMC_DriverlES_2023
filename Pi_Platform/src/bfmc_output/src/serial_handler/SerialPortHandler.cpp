/**
 * @file SerialPortHandler.cpp
 * @author Bosch Engineering GmbH
 * @brief The header file holds the class for sending commands to the STM board.
 * @version 1.0
 * @date 2023-07-17
 *
 * @copyright  Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.
 */
#include "serial_handler/SerialPortHandler.hpp"

using namespace std;

serialPortHandler::serialPortHandler(boost::asio::io_service& io_service, unsigned int baud, const string& device, BaseResponseHandler& responseHandler) :
    m_responseHandler(responseHandler),
    active_(true),
    io_service_(io_service),
    serialPort(io_service, device) {
    if (!serialPort.is_open()) {
        cerr << "Failed to open serial port\n";
        return;
    }
    else {
        cout << "Port opened!" << std::endl;
    }

    boost::asio::serial_port_base::baud_rate baud_option(baud);
    serialPort.set_option(baud_option); // set the baud rate after the port has been opened

    boost::system::error_code error;
    flush_serial_port(serialPort, flush_both, error);
    std::cout << "flush: " << error.message() << std::endl;

    read_start();
}

void serialPortHandler::write(std::string f_msg) {
    io_service_.post(boost::bind(&serialPortHandler::do_writeString, this, f_msg));
}

void serialPortHandler::close() {
    io_service_.post(boost::bind(&serialPortHandler::do_close, this, boost::system::error_code()));
}

bool serialPortHandler::active() {
    return active_;
}

void serialPortHandler::read_start(void) {
    serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
                               boost::bind(&serialPortHandler::read_complete,
                                           this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

void serialPortHandler::read_complete(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error) {

        m_responseHandler(read_msg_, bytes_transferred);
        // ROS_INFO-DEBUG SerialPortHandler
        // cout.write(read_msg_, bytes_transferred); // echo to standard output
        read_start(); // start waiting for another asynchronous read again
    }
    else
        do_close(error);
}

void serialPortHandler::do_writeString(std::string f_msg) {

    boost::lock_guard<boost::mutex>* guard             = new boost::lock_guard<boost::mutex>(m_writeMtx);
    bool                             write_in_progress = write_msgs_.empty();
    for (std::string::iterator it = f_msg.begin(); it != f_msg.end(); ++it) {
        write_msgs_.push_back(*it);
    }
    delete guard;
    if (write_in_progress) {
        write_start();
    }
}

void serialPortHandler::write_start(void) {
    boost::asio::async_write(serialPort,
                             boost::asio::buffer(&write_msgs_.front(), 1),
                             boost::bind(&serialPortHandler::write_complete,
                                         this,
                                         boost::asio::placeholders::error));
}

void serialPortHandler::write_complete(const boost::system::error_code& error) {
    // Convert deque to string
    // std::string messageString(write_msgs_.begin(), write_msgs_.end());
    // Print message using ROS_INFO
    // ROS_INFO("Write messages %s", messageString.c_str());

    if (!error) {                 // write completed, so send next write data
        write_msgs_.pop_front();  // remove the completed data
        if (!write_msgs_.empty()) // if there is anthing left to be written
            write_start();        // then start sending the next item in the buffer
        else {
            // Measure time if write completed!
            tEnd = ros::Time::now();
            // double elapsedTime = (tEnd - tStart).toSec() * 1000; // conversion to ms
            // ROS_INFO("SerialHandler: %f", elapsedTime);
        }
    }
    else
        do_close(error);
}

void serialPortHandler::do_close(const boost::system::error_code& error) {
    if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel()
        return;                                         // ignore it because the connection cancelled the timer
    if (error)
        cerr << "Error: " << error.message() << endl; // show the error message
    serialPort.close();
    active_ = false;
}

void serialPortHandler::flush_serial_port(
    boost::asio::serial_port&  serial_port,
    flush_type                 what,
    boost::system::error_code& error) {
    if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what)) {
        error = boost::system::error_code();
    }
    else {
        error = boost::system::error_code(
            errno,
            boost::asio::error::get_system_category());
    }
}
