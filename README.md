# Udacity Robotics Nanodegree
**Introduction** : This projct "Home Service Robot" is developed and simulated as fully autonomous robot, which when recieves pick up zone and drop off zone's (x,y) coordinate via command line,
starts localizing and navigating itself on a given map using Adaptive Montecarlo localization(one of Probalistic localization approach) and navigating to goal by planning a global plan based on Grid-based Dijkstra's algorithm and avoiding obstacle and course correction by local planner which used Dynamic Window Approach (DWA).

**Prerequisite**
- Ubuntu 16.04
* ROS Kinetic
+ Gazebo 7.16

**ROS Package used**
- [SLAM Gmapping](https://github.com/ros-perception/slam_gmapping)
- [turtlebot_teleop](https://github.com/turtlebot/turtlebot)
- [turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions)
- [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator)
- [AMCL](http://wiki.ros.org/amcl?distro=noetic)
- [move_base](http://wiki.ros.org/move_base?distro=noetic)

**Directory Structure**

    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── my_robot                          # Custom Robot files
    │   ├── config                # Nav planner & Costmap yaml files
    │   ├── launch                # Launch files for SLAM, amcl,world and robot
    |   ├── maps                  # Holds Map files
    |   ├── meshes                # sensor Mesh file
    |   ├── urdf                  # Robot's urdf & gazebo parameters 
    |   ├── worlds                # Gazebo simulation environment
    |   ├── ...
    ├── scripts                   # shell scripts files
    │   ├── test_slam.sh          # My robot 
    |   ├── test_navigation.sh
    |   ├── pick_object.sh
    |   ├── add_marker.sh         # same beahvior as home_service.sh, can be used interchangebly
    |   ├── home_service.sh
    │   ├── test_slam_turtlebot.sh          # turtlebot kobuki in custom world 
    |   ├── test_navigation_turtlebot.sh
    |   ├── pick_object_turtlebot.sh
    |   ├── add_marker_turtlebot.sh         # same beahvior as home_service.sh, can be used interchangebly
    |   ├── home_service_turtlebot.sh
    ├──rvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──

  **Launch Instructions**

  `~catkin_ws $ ./src/scripts/home_service_turtlebot.sh`
  `~catkin_ws $ ./src/scripts/home_service.sh`

** Videos **

[pick object](https://www.youtube.com/watch?v=PjNk3vHYKDY)
[home service robot turtlebot](https://www.youtube.com/watch?v=pYREEd_RO1M)


  **Notes**
  This project has two set of robots in same environment. First set is having custom robot used throughtout this nanodegree (my_robot) and second one is turtlebot kobuki.
  

  
