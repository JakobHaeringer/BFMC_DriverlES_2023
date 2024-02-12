##
# @file NucleoCommand.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used to send steering/speed commands to the serialHandlerNODE.
# @param header        Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param command_type  Type of the command.
# @param value         Value of the specified command.
# @version 1.0
# @date 2023-07-13
Header header
uint8 command_type
float32 value