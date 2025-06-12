Currently the project relys on the following sensors and topics :

- Lidar: Topic Name: /lidar_points Message Type: Type: sensor_msgs/msg/PointCloud2
- UWB: Topic Name: /uwb/data Message Type: uwb_location/msg/UWB
- Camera: Topic Name: /image_right_raw/h264_half Message Type: foxglove_msgs/msg/CompressedVideo
- Odom: /rt/odom Message Type: nav_msgs/msg/Odometry
- State Machine: /rt/lowstate Message Type: lowlevel_msg/msg/LowState 
- Low Command: /rl_lowcmd Message Type: lowlevel_msg/msg/LowCmd
- Joy: /joy Message Type: sensor_msgs/msg/Joy
