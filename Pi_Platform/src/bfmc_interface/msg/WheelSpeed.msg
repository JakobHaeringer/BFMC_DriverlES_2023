##
# @file WheelSpeed.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for data sent by the wheelspeed sensors.
# @param header             Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param wheelspeedLeft     [m/s] Current speed of the left wheel.
# @param wheelspeedRight    [m/s] Current speed of the right wheel.
# @param speed              [m/s] Current speed of the vehicle.
# @param totalDistance      [m] Overall driven distance of the vehicle.
# @version 1.0
# @date 2023-07-13
Header header
float32 wheelspeedLeft
float32 wheelspeedRight
float32 speed
float32 totalDistance