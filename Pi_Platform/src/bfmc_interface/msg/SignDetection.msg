##
# @file SignDetection.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the sign and object detection.
# @param header                Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param sign_class            Detected class name.
# @param x_bounding_box        [Pixel] X coordinate of the middle point of the bounding box.
# @param y_bounding_box        [Pixel] Y coordinate of the middle point of the bounding box.
# @param width_bounding_box    [Pixel] Width of the of the bounding box.
# @param height_bounding_box   [Pixel] Height of the of the bounding box.
# @param confidence            [%] Confidence of the neural network for the detected object.
# @param distance              [m] Distance to the detected object.
# @param area                  Area in the image of the detected object (Left, Middle, Right).
# @version 1.0
# @date 2023-07-13
Header header
string sign_class
uint16 x_bounding_box
uint16 y_bounding_box
uint16 width_bounding_box
uint16 height_bounding_box
float32 confidence
float32 distance
uint8 area
