# ROS2 layout of the ROS1 Livox message retained by rosbags-convert.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
