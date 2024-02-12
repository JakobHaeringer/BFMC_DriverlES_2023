#!/usr/bin/env python3
##
# @file traffic_light_node.py
# @author Jakob Haeringer, Bosch Engineering GmbH
# @brief This file implements the logic for processing the data received by the environmental server concerning the traffic lights.
# @version 1.0
# @date 2023-08-17
# @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.

import socket
import json
import rospy
from bfmc_interface.msg import TrafficLight


class TrafficLightNode():
    """Class that handles the messages received by the environmental server concerning the traffic lights. 
     This class was implemented by Bosch itself. Since there were some errors, we adapted the class slightly and also changed the ROS topic names."""

    def __init__(self):
        """Constructor that initializes all the ROS publisher topics.
        @param self The IMUNode object itself. """
        rospy.init_node('traffic_light', anonymous=False)

        self.trafficLightEastPublisher = rospy.Publisher(
            "/input/environment/traffic_light/east", TrafficLight, queue_size=1)
        self.trafficLightSouthPublisher = rospy.Publisher(
            "/input/environment/traffic_light/south", TrafficLight, queue_size=1)
        self.trafficLightWestPublisher = rospy.Publisher(
            "/input/environment/traffic_light/west", TrafficLight, queue_size=1)
        self.trafficLightStartPublisher = rospy.Publisher(
            "/input/environment/traffic_light/start", TrafficLight, queue_size=1)
        self.trafficLightMsg = TrafficLight()

    # ================================ RUN ========================================
    def run(self):
        """Method for running listener algorithm.
        @param self The IMUNode object itself. """
        rospy.loginfo("starting traffic_light")
        self._init_socket()
        self._getting()

    # ================================ INIT SOCKET ========================================
    def _init_socket(self):
        """Communication parameters, create and bind socket.
        @param self The IMUNode object itself. """
        self.PORT = 50007
        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # (internet, UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(('', self.PORT))
        self.sock.settimeout(1)

    # ================================ GETTING ========================================
    def _getting(self):
        """Listen for incoming broadcast messages.
        @param self The IMUNode object itself. """
        while not rospy.is_shutdown():
            # Wait for data
            try:
                data, addr = self.sock.recvfrom(
                    4096)  # buffer size is 1024 bytes
                dat = data.decode('utf-8')
                dat = json.loads(dat)
                ID = int(dat['id'])
                self.trafficLightMsg.header.stamp = rospy.Time.now()
                self.trafficLightMsg.state = int(dat['state'])

                if (ID == 2):
                    self.trafficLightEastPublisher.publish(
                        self.trafficLightMsg)
                elif (ID == 4):
                    self.trafficLightSouthPublisher.publish(
                        self.trafficLightMsg)
                elif (ID == 1):
                    self.trafficLightWestPublisher.publish(
                        self.trafficLightMsg)
                elif (ID == 3):
                    self.trafficLightStartPublisher.publish(
                        self.trafficLightMsg)

            except Exception as e:
                if str(e) != "timed out":
                    print("Receiving data failed with error: " + str(e))


if __name__ == "__main__":
    traffic_light_node = TrafficLightNode()
    traffic_light_node.run()
