# This message is used to send actuator commands to the simulated vehicle.

Header header

# The identifier of the actuator. Possible values are...
#     - motor
#     - steering
string id

# The values to send to the actuator. Depending on the identifier, the possible values are:
#     - motor:    torque in [N/m]
#     - steering: steering angle in [deg]
float32[] values