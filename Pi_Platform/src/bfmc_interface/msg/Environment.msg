##
# @file Environment.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used to send detected obstacles to the environmental server.
# @param header        Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param obstacle_id   ID of the detected obstacle - see LiveTrafficSystemID.
# @param x             X coordinate of the detected obstacle.
# @param y             Y coordinate of the detected obstacle.
# @version 1.0
# @date 2023-07-13
Header header
uint8 obstacle_id
float32 x
float32 y