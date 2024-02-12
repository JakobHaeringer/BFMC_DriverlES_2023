#!/usr/bin/env python3
##
# @file imu_node.py
# @author Jakob Haeringer, Constantin Blessing, Bosch Engineering GmbH
# @brief This file implements the logic to process the data from the IMU of the vehicle or the simulated IMU of the Unity Simulator.
# @version 1.0
# @date 2023-08-17
# @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers, All rights reserved.

from bfmc_interface.msg import IMU
import rospy
import time
import os.path
import sys
import numpy

sys.path.append('.')

ITMOVES_SIMULATOR = rospy.get_param("ITMOVES_SIMULATOR", False)

if ITMOVES_SIMULATOR:
    from bfmc_interface.msg import SimulatorIMU
else:
    import RTIMU


class IMUNode():
    """ Class that handles and filters the messages received by the Bosch IMU.
    This class was pre-implemented by Bosch but has been adapted by us.
    """

    gravity = 9.80665

    def __init__(self):
        """Constructor that initializes the ROS topic, as well as the RTIMULib library.
        @param self The IMUNode itself. """
        rospy.init_node('imu', anonymous=False)
        self._publisher = rospy.Publisher("/input/imu", IMU, queue_size=1)

        if ITMOVES_SIMULATOR:
            return

        settings_file = "RTIMULib"

        print(f"Using settings file '{settings_file}.ini'.")

        if not os.path.exists(f"{settings_file}.ini"):
            print(
                f"Settings file '{settings_file}.ini' does not exist and will be created.")

        self._imu_settings = RTIMU.Settings(settings_file)

    def run(self):
        """Initializes the IMU and then polls for the data produced by the IMU.
        @param self The IMUNode object itself. """
        rospy.loginfo("Starting IMU node.")

        if ITMOVES_SIMULATOR:
            def callback(msg):
                # Limit pitch to be between -90° and +90°.
                if msg.orientation.x >= numpy.pi:
                    msg.orientation.x = max(
                        msg.orientation.x - 2.0 * numpy.pi, -0.5 * numpy.pi)
                else:
                    msg.orientation.x = min(
                        msg.orientation.x, 0.5 * numpy.pi)

                self._publish_message(
                    msg.orientation.z,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.acceleration.z,
                    msg.acceleration.x,
                    msg.acceleration.y)

            rospy.Subscriber("/simulator/imu",
                             SimulatorIMU, callback)
            rospy.spin()
        else:
            self._init_imu()
            self._poll_imu()

    def _init_imu(self):
        """Loads the predefined settings into the RTIMULib which is preprocessing the IMU data with a Kalman Filter.
        @param self The IMUNode object itself. """
        self._imu = RTIMU.RTIMU(self._imu_settings)

        if not self._imu.IMUInit():
            sys.exit(1)

        self._imu.setSlerpPower(0.02)
        self._imu.setGyroEnable(True)
        self._imu.setAccelEnable(True)
        self._imu.setCompassEnable(True)

        # The poll interval should be 7ms.
        self._poll_interval = self._imu.IMUGetPollInterval()

    def _poll_imu(self):
        """As long as the ROS node is active, the function polls for the IMU data and publishes them through a helper fuction if the queue holds some.
        @param self The IMUNode object itself. """
        while not rospy.is_shutdown():
            time.sleep(self._poll_interval / 1000.0)

            if not self._imu.IMURead():
                continue

            imu_data = self._imu.getIMUData()
            pose, acceleration = imu_data["fusionPose"], imu_data["accel"]
            self._publish_message(
                pose[0], pose[1], pose[2], acceleration[0], acceleration[1], acceleration[2])

    def _publish_message(self, roll, pitch, yaw, acceleration_x, acceleration_y, acceleration_z):
        """The helper function that publishes the data through a ROS topic and converts the acceleration data from g forces to m/s².
        @param self The IMUNode object itself.
        @param roll The value of the rotation around the x-axis.
        @param pitch The value of the rotation around the y-axis.
        @param yaw The value of the rotation around the z-axis.
        @param acceleration_x The value of the acceleration in the direction of the x-axis.
        @param acceleration_y The value of the acceleration in the direction of the y-axis.
        @param acceleration_z The value of the acceleration in the direction of the z-axis."""
        imu_msg = IMU()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.roll = roll
        imu_msg.pitch = pitch
        imu_msg.yaw = yaw

        if not ITMOVES_SIMULATOR:
            acceleration_x *= IMUNode.gravity
            acceleration_y *= IMUNode.gravity
            acceleration_z *= IMUNode.gravity

        imu_msg.acceleration_x = acceleration_x
        imu_msg.acceleration_y = acceleration_y
        imu_msg.acceleration_z = acceleration_z
        self._publisher.publish(imu_msg)


if __name__ == "__main__":
    imu_node = IMUNode()
    imu_node.run()
