##
# @file IMU.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the IMU data.
# @param header            Contains the standard ROS header (std_msgs/Header) with (sequence ID, timestamp and frame_id).
# @param roll              [°]    Roll  angle                   (right      = negative, left     = positive)
# @param pitch             [°]    Pitch angle                   (up         = negative, down     = positive)
# @param yaw               [°]    Yaw   angle                   (right      = positive, left     = negative)
# @param acceleration_x    [m/s²] Longitudinal  Acceleration    (forward    = positive, backward = negative)
# @param acceleration_y    [m/s²] Lateral       Acceleration    (right      = positive, left     = negative)                        
# @param acceleration_z    [m/s²] Vertical      Acceleration    (up         = positive, down     = negative)
# @version 1.0
# @date 2023-07-13
Header header
float32 roll
float32 pitch
float32 yaw
float32 acceleration_x
float32 acceleration_y
float32 acceleration_z