# This message contains an IMU measurement.

Header header

# The orientation in [rad].
# x = pitch (up         = negative, down     = positive)
# y = yaw   (right      = positive, left     = negative)
# z = roll  (right      = negative, left     = positive)
geometry_msgs/Vector3 orientation

# Acceleration in [m/s].
# x = lateral         (right      = positive, left     = negative)
# y = vertical        (up         = positive, down     = negative)
# z = longitudinal    (forward    = positive, backward = negative)
geometry_msgs/Vector3 acceleration