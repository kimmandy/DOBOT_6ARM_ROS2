# DOBOT_6ARM_ROS2

6축 로봇암(DOBOT)을 제어하기 위한 코드입니다.

how to use.

frist connect with dobot wifi

open the terminal(cmd) try this code.

<code> ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py </code>
-> to connect with robot. has a log like this? it goood

$ ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot "{}"
-> next open the anther cmd open and enter this code. and it say's like
