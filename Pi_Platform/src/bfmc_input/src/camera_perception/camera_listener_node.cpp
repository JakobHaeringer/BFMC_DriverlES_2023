/**
 * @file camera_perception_listener_node.cpp
 * @author Jakob Haeringer
 * @brief This file implements the logic from the Raspberry Pi to receive and decode messages from the Nvidia Jetson board vie UDP.
 * @version 1.0
 * @date 2023-07-17
 */
#include "bfmc_interface/LaneDetection.h"
#include "bfmc_interface/SignDetection.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>

/** @brief IPv4 Address of the Raspberry Pi itself. */
#define IPADDRESS "192.168.2.2"
/** @brief UDP Port that the Raspberry Pi opens to receive messages. */
#define UDP_PORT 13251
/** @brief The identifier in the UDP message which signals that the message contains data from the lane detection. */
#define HEADER_LANE_DETECTION "1"
/** @brief The identifier in the UDP message which signals that the message contains data from the object detection. */
#define HEADER_SIGN_DETECTION "2"
/** @brief The delimiter that splits up the contents of a single UDP message. */
#define MESSAGE_DELIMITER ";"

using boost::asio::ip::address;
using boost::asio::ip::udp;

/**
 * @brief This class implements the Raspberry Pi UDP listener that receives messages and publishes them on the ROS network running on the Raspberry Pi.
 *
 * The Client starts a new system thread, which opens up a UDP port. On this port the Raspberry then receives the
 * messages sent by the Nvidia Jetson Board. The messages are a custom string, that contains an identifier so the
 * Raspberry Pi can tell how the data must be interpreted, as well as a body that holds the important information from
 * the equivalent process. The UDP messages break up the messages and then sent stream the information via ROS topic
 * to the rest of the Raspberry Pi platform. Here the lane detection and the object detection data are sent on a single
 * topic each.
 */
class Client {
    /** @brief The handler of the UDP socket on operating system level. */
    boost::asio::io_service io_service;
    /** @brief The UDP socket which receives the messages. */
    udp::socket socket{io_service};
    /** @brief The queue of the UDP socket. */
    boost::array<char, 1024> recv_buffer;
    /** @brief The combination of IPv4 address and port number that define the UDP endpoint. */
    udp::endpoint remote_endpoint;

    /** @brief The ROS node handler that manages the communication on the node. */
    ros::NodeHandle nh;
    /** @brief The publisher which broadcasts the data from the lane detection. */
    ros::Publisher laneDetectPub;
    /** @brief The publisher which broadcasts the data from the object detection. */
    ros::Publisher signDetectPub;

    /** @brief The message object that is used as a template for the ROS lane detection message. */
    bfmc_interface::LaneDetection laneDetectMsg;
    /** @brief The message object that is used as a template for the ROS object detection message. */
    bfmc_interface::SignDetection signDetectMsg;

    // Variables that are used to measure the time for decoding a message. Used for debugging purposes.
    // ros::Time tStart;
    // ros::Time tEnd;

public:
    /** @brief Construct a new Client object and open up the ROS publisher topics. */
    Client() {
        laneDetectPub = nh.advertise<bfmc_interface::LaneDetection>("/input/camera/lane_detection", 1);
        signDetectPub = nh.advertise<bfmc_interface::SignDetection>("/input/camera/sign_detection", 5);
    }

    /** @brief Open a UDP socket and bind it to the specified IPv4 address and port number. */
    void Receiver() {
        socket.open(udp::v4());
        socket.bind(udp::endpoint(address::from_string(IPADDRESS), UDP_PORT));
        wait();
        io_service.run();
    }

    /** @brief This will send the UDP socket process to sleep until received data are written into the queue. */
    void wait() {
        socket.async_receive_from(boost::asio::buffer(recv_buffer),
                                  remote_endpoint,
                                  boost::bind(&Client::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    /** @brief The received message gets checked for errors and then sent to the decoder.
     *
     * @param error Contains an error code if something went wrong during the receiving of the UDP message.
     * @param bytes_transferred The size of the received message in bytes.
     */
    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        // tStart = ros::Time::now();
        if (error) {
            // std::cout << "Receive failed: " << error.message() << "\n";
            return;
        }
        decodeMessage(error, bytes_transferred);
        // tEnd = ros::Time::now();
        // double elapsedTime = (tEnd - tStart).toSec() * 1000; // conversion to ms
        // ROS_INFO("Client,%f", elapsedTime);
        wait();
    }

    /** @brief The decoder which classifies the content of the message and splits up the content accordingly.
     *
     * @param error Contains an error code if something went wrong during the receiving of the UDP message.
     * @param bytes_transferred The size of the received message in bytes.
     */
    void decodeMessage(const boost::system::error_code& error, size_t bytes_transferred) {
        std::string message = std::string(recv_buffer.begin(), recv_buffer.begin() + bytes_transferred);

        // Get header and erase it from message
        std::string header = message.substr(0, 1);
        message.erase(0, 2);

        if (header == HEADER_LANE_DETECTION) {
            std::vector<std::string> splitted_message = split(message, MESSAGE_DELIMITER);
            laneDetectMsg.header.stamp                = ros::Time::now();
            laneDetectMsg.midDistance                 = std::stof(splitted_message[0]);
            laneDetectMsg.curveCoefficient            = std::stof(splitted_message[1]);
            laneDetectPub.publish(laneDetectMsg);
            // ROS_INFO("Sent dist: %f \t Sent curveCoeff: %f", laneDetectMsg.midDistance, laneDetectMsg.curveCoefficient);
        }
        else if (header == HEADER_SIGN_DETECTION) {
            std::vector<std::string> splitted_message = split(message, MESSAGE_DELIMITER);
            signDetectMsg.header.stamp                = ros::Time::now();
            signDetectMsg.sign_class                  = splitted_message[0];
            signDetectMsg.x_bounding_box              = std::stoi(splitted_message[1]);
            signDetectMsg.y_bounding_box              = std::stoi(splitted_message[2]);
            signDetectMsg.width_bounding_box          = std::stoi(splitted_message[3]);
            signDetectMsg.height_bounding_box         = std::stoi(splitted_message[4]);
            signDetectMsg.confidence                  = std::stof(splitted_message[5]);
            signDetectMsg.distance                    = std::stof(splitted_message[6]);
            signDetectMsg.area                        = std::stoi(splitted_message[7]);
            signDetectPub.publish(signDetectMsg);
        }
        else {
            ROS_INFO("Header unknown");
        }
    }

    /** @brief Splitting up the received message string at the specified delimiter.
     *
     * @param s The string that holds the message.
     * @param delimiter The delimiter that separates the messages content.
     * @return A vector that holds the split message where each element contains one string.
     */
    std::vector<std::string> split(std::string s, std::string delimiter) {
        size_t                   pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string              token;
        std::vector<std::string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
            token     = s.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }
};

int main(int argc, char* argv[]) {
#ifdef ITMOVES_SIMULATOR
    return 0;
#else
    ros::init(argc, argv, "camera_perception_listener");
    Client client;
    // Start thread for the UDP client
    std::thread r([&] { client.Receiver(); });
    r.join();

    return 0;
#endif
}
