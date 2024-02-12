##
# @file TrafficLight.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the current traffic light state.
# @param header     Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param state      Current state of the traffic light. Possible states are:
#                       - RED       = 0
#                       - YELLOW    = 1
#                       - GREEN     = 2
#                       - UNDEFINED = 3
# @version 1.0
# @date 2024-01-12
Header header
uint8 state