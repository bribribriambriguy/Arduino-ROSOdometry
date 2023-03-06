# Arduino-ROSOdometry
Arduino-ROS differential drive base controller (Noetic)

Repository includes Arduino node that publishes /left_wheel (Int16) encoder ticks and /right_wheel (Int16) encoder 
ticks, also subscribes to /cmd_vel (geometry_msgs/Twist) and converts into two motor speeds for differential drive
robot. Python node subscribes to /left_wheel and /right_wheel and publishes /odom (nav_msgs/Odometry) and broadcasts
odom -> base_link transform. 
