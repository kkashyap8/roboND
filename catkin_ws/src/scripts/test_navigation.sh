#!/bin/sh
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"
