# Udacity Robotics Nanodegree
**Introduction** :  "Home Service Robot" is developed and simulated as fully autonomous robot, which when receives pick up zone and drop off zone's (x,y) coordinate via command line,
starts localizing and navigating itself on a given map using Adaptive Montecarlo localization(one of Probalistic localization approach) and navigating to goal by planning a global plan based on Grid-based Dijkstra's algorithm and avoiding obstacle and course correction by local planner.

**Submission Content**

Package and Launch file to be considered:
- my_robot # package contains world file, urdf, configs and maps
- world.launch # launching world file
- mapping.launch # launching RTAB-map based mapping
- amcl.launch # launching amcl

**Shell scrpits to be used for Project Evaluation**
~catkin_ws:
- `./src/scripts/test_slam.sh`
- `./src/scripts/test_navigation.sh`
- `./src/scripts/pick_objects.sh` (ex: (x,y): Pick Up Zone: 8.0 3.5 / Drop Off zone: 1.0 0.0)
- `./src/scripts/add_marker.sh` 
- `./src/scripts/home_service.sh`

**Prerequisite**
- Ubuntu 16.04
* ROS Kinetic
+ Gazebo 7.16

**ROS Package used**
- [SLAM Gmapping](https://github.com/ros-perception/slam_gmapping) : This package contains Grid based fast SLAM alogoritm for mapping, Robot's pose is estimated using particle filter, while map is created as Grided occupancy map. This package needs Laser scans and Odometry of robot to create a 2D occupancy map, which should be created and used with help of map_server node in navigation tasks. map_server node generates two file map meta data and map as image.`Test_slam.sh` should be used for mapping & use `rosrun map_server map_saver -f mymap` to create map and it's metadata file.
  
- [AMCL](http://wiki.ros.org/amcl?distro=noetic) : AMCL package is used for localizing robot in world using particle filters, which is a very powerful probablistic localization approach. It's beauty lies in its adaptive nature of particle size and resampling method.It is relatively light weight on memory and gives contol on sample size selection. AMCL needs laser scan and robot's odometry to update particle weight and refines the robot's pose and send pose infomation to move_base for robot's maneuver in static or dynamic environment to avoid obstacle.Once the map is available this package can be used to fairly estimate robot's pose in world. To test amcl, use `Test_navigation.sh`.
There are various other techniques available for localization, few of them are EKF, Bayes filter based Markov decision ,Histogram filters etc. 
  
- [move_base](http://wiki.ros.org/move_base?distro=noetic) : This package is heart of this project, as it enables planning a global path and executing local short terms goals by using costmap, which again is a gridded occupancy map.It also enable corrective behaviors like rotation, back up and navigating around obstacle as well as sampling path and selecting best scoring path for navigation. Its provide actuation to robot's base based on odometery, laser scan, Tf informtaions and obviously map. For Global path planning , we are using navfn and for local planning, in general `base_local_planner` or `dwa_local_planner` can be used. For thios project , we used Dynamic windowing approach "dwa_local_planner", which is better in general.
  
- [turtlebot_teleop](https://github.com/turtlebot/turtlebot) : This package is used to sending goal command to robot either via keyboard or via controller. `test_navigation.sh` use this package.
  
- [turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions) : This package contains configuration setting for Rviz to launch panels. It enables robot's pose visualization along with Occupancy grid map, global cost map, local coast map, laser scans, point cloud, trjectories (both Global & local), Particle's distribution from AMCl and much more.
  
- [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator) : This package contains virtual models of different turtlebots in urdf format and launch files, mesh files, configuration files and more to simulate navigation in gazebo environment.


**Directory Structure**

    ├── # Official ROS packages
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
    |
    ├── # custom my_robot package & it's content                       
    |
    ├── my_robot                  # Custom Robot files
    │   ├── config                # Nav planner & Costmap yaml files
    │   ├── launch                # Launch files for SLAM, amcl,world and robot, should be used for project evalution
    |   ├── maps                  # Holds Map files
    |   ├── meshes                # sensor Mesh file
    |   ├── urdf                  # Robot's urdf & gazebo parameters 
    |   ├── worlds                # Gazebo simulation environment
    |   ├── ...
    ├── scripts                   # shell scripts files
    |   ├── # For my_robot package,to be used for project evaluation
    │   ├── test_slam.sh          # My robot 
    |   ├── test_navigation.sh
    |   ├── pick_object.sh
    |   ├── add_marker.sh         # same beahvior as home_service.sh, can be used interchangebly
    |   ├── home_service.sh
    |   ├── # For turtlebot kobuki, not to be used for project evaluation
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

  `~catkin_ws $ ./src/scripts/home_service.sh` (Should be used for Project evaluation)

  `~catkin_ws $ ./src/scripts/home_service_turtlebot.sh`
  

**Videos**

[pick object](https://www.youtube.com/watch?v=PjNk3vHYKDY)

[home_service robot my_robot](https://www.youtube.com/watch?v=kR3tQOS2jDM)

[home service robot turtlebot](https://www.youtube.com/watch?v=pYREEd_RO1M)

**Project brief**

Lets dive in to details.

- Using Gazebo, Design your simulation environment. Special care should be taken to ensure enough distinct features in your world, as they will be helpful during mapping and localization.
  
  <img width="274" alt="my_world" src="https://github.com/kkashyap8/roboND/assets/32445398/1d7238ff-436c-436e-93fa-69a3e8db2b3d">

- use Gazebo or urdf or any other tools(which support robot design and can be exproted as xml,urdf or sdf format) to design your robot , its moving and static linkage along with sensors(RGBD Camera & LIDAR sensor) and actuators(differential wheel controller for odometry).
  
  <img width="413" alt="robot model" src="https://github.com/kkashyap8/roboND/assets/32445398/19799893-15d9-42a8-9e2a-3af92abd2db3">

- start creating your package in catkin environment with turtlebot teleop ros pacakage and Rviz to start testing you robots behavior and fine tune its properties for sensors and actuators.
  
- Once satisfied with results, use SLAM_Gmapping package(laser based SLAM) or RTAB map(real time appearance based mapping technique, used SURF) to create MAP to be used further in naviagtion stack. I used both of them, but for final project kept map created by RTAB map using SURF to create Graph SLAM with visual feature matching with robot's Pose with loop clousre enabled. If your environment has similar visual similarities, our map might need multiple iterations to create approximate map.
  
  ![RTAB map](https://github.com/kkashyap8/roboND/assets/32445398/ff1ffbcf-186f-48be-b677-24eabf05cb9a)
  
if you see, following final map in **Tested Locations for pick up and Drop off** has some distortions as well in lower half on map.

- In next phase, Start with Localization. This project uses odometry and Laser scans with one of the probablistic localization approach, Adaptive montecarlo Localization to localize robot in world using resampled particles within min and Max particle's probablastic pose distribution. There are great length of parameters , which needs to be tuned for better result of AMCL, ROS wiki's detailed explanation for AMCL is great, so must refer it.

   Initial pose of robot in map : particle's pose are covering large surrounding region 

  <img width="293" alt="initial pose" src="https://github.com/kkashyap8/roboND/assets/32445398/4aaac828-774e-49f7-8fb2-7a76a461687e">

  Pose after filter update : Particles are concentrated in very small surrounding region
 
  ![pose after filter update ](https://github.com/kkashyap8/roboND/assets/32445398/61ed37de-bc1b-4a3f-979d-fa5af2a7be63)

- Now, you are ready to test you Navigation stack to see how your robot automatically plan path, both Global and Local to reach goal,either sent by teleop package or Rviz or any custom package written by you. This phase also deals with various concepts like, cost map, local planner, global planner , recovery methods etc. Again great resources are present in ROS wiki for move_base package.

 ![navigation](https://github.com/kkashyap8/roboND/assets/32445398/cacd0093-060d-4714-80bc-b603e4b38525)

- This phase in project to write your own package, which will send goals to your robot one by one. Here I used actionlib_msgs to request move_base or in General Robot to go to a pick up zone or drop off zone, we do get response as well, if action was performed successfully or not.
  
- The last piece of this project was to create a node, which will ask Rviz to display a marker at pick up zone , when Robot is enroute to Pickup zone or when reached to drop off zone.
  
- Please watch home service robot turtlebot video at above mentioned link.

**Tested Locations for pick up and Drop off**

    X         Y 
    8.0       3.5
    3.0       5.0
    -4.0      5.0
    -6.5      1.5
    1.0       0
    
  <img width="520" alt="Map_positions" src="https://github.com/kkashyap8/roboND/assets/32445398/d890a4e6-bd12-49e8-b41a-5237f82d042a">

**Notes**

This project has two set of robots in same environment. First set is having custom robot used throughtout this nanodegree (my_robot) and second one is turtlebot kobuki.
To use turtlebot with custom package from my_robot package, Please download zipped folder as we need to make some changes in turtlebot packages(checked in as submodule, please excuse my little knowledge with git).
planning to improve this package in future to explore more on ROS. 
  

  
