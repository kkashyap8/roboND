#!/bin/sh
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
xterm -e " source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 2
xterm -e " source devel/setup.bash; rosrun topic_tools relay cmd_vel_mux/input/teleop /cmd_vel" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch my_robot mapping.launch"
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &


