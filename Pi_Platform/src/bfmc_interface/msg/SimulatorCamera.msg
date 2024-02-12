# This message encompasses a snapshot of the Unity simulator's ego camera.

# The RGB color image.
sensor_msgs/CompressedImage color
# A 16-bit depth image, whereas each pixel encodes the distance in millimeters.
# Therefore, the maximum depth encoded is 2^16 millimeters.
sensor_msgs/Image depth