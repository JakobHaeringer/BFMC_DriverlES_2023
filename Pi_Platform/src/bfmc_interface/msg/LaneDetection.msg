##
# @file LaneDetection.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for sending the lanedetection values.
# @param header            Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param midDistance       [m] Distance to the center of the lane.
# @param curveCoefficient  Curvature coefficient of the lane.
# @version 1.0
# @date 2023-07-13
Header header
float32 midDistance
float32 curveCoefficient
