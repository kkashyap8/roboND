#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global joint publisher variables
ros::Publisher goal_pub, robot_state_pub;

int main(int argc, char** argv)
{
   // Initialize the pick_objects node
   ros::init(argc, argv, "pick_objects");
   ros::NodeHandle n;
   goal_pub = n.advertise<geometry_msgs::Point>("send_pose",1);
   robot_state_pub = n.advertise<std_msgs::UInt8>("robot_state",1);

   // Tell the action client that we wnat to spin a thread by default 
   MoveBaseClient ac("move_base", true);
   // Wait 5 sec for move_base action server to come up
   while (!ac.waitForServer(ros::Duration(5.0)))
   {
      ROS_INFO("Waiting for the move_base action server to come up");
   }
   
   // run unless user shuts down node
   while (ros::ok())
   {
       double pickup_loc_x, pickup_loc_y;
       double dropoff_loc_x, dropoff_loc_y;
       // User input
       std::cout << std::endl <<"Enter Pick up location, Please :";
       std::cin >> pickup_loc_x >> pickup_loc_y;
       std::cout << std::endl <<"Enter Drop off location, Please :";
       std::cin >> dropoff_loc_x >> dropoff_loc_y;
       

       // Create goal with MoveBaseGoal message & send pose 
       move_base_msgs::MoveBaseGoal goal;
       geometry_msgs::Point send_pose;
       std_msgs::UInt8 robot_state;
       // Set up the frame parameters
       goal.target_pose.header.frame_id = "map";
       goal.target_pose.header.stamp = ros::Time::now();
   
       // Pick up position
       goal.target_pose.pose.position.x = pickup_loc_x;
       goal.target_pose.pose.position.y = pickup_loc_y;
       goal.target_pose.pose.orientation.w = 1.0;

       // Publish same data on send_pose
       send_pose.x = pickup_loc_x;
       send_pose.y = pickup_loc_y;
       send_pose.z = 1.0; // encoding for pick up location

       // Send the Pick up Position and orientation to robot
       ROS_INFO("Sending Pick up Location");
       ac.sendGoal(goal);
       // Also Publish goal position 
       goal_pub.publish(send_pose);
       robot_state.data = 1;
       robot_state_pub.publish(robot_state);//enroute Pickup Location
       // Wait an infinite time for the results
       ac.waitForResult();

       // Check if the robot reached its goal
       if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       {
           ROS_INFO(" Robot is at Pick up location now");
           robot_state.data = 2;
           robot_state_pub.publish(robot_state);//Reached Pickup Location
       }
       else
       {
           ROS_INFO("Robot failed to reach Pick up location for some reason");
           robot_state.data = 0;
           robot_state_pub.publish(robot_state);//could n't reach to pick up/ drop off location
       }
       // Now, send Drop off location to Move base
       goal.target_pose.pose.position.x = dropoff_loc_x;
       goal.target_pose.pose.position.y = dropoff_loc_y;
       goal.target_pose.pose.orientation.w = 1.0;
        
       // Publish same data on send_pose
       send_pose.x = dropoff_loc_x;
       send_pose.y = dropoff_loc_y;
       send_pose.z = 0.0; // encoding for drop off location

       // Send the drop off Position and orientation to robot
       ROS_INFO("Sending Drop off Location");
       ac.sendGoal(goal);
       goal_pub.publish(send_pose);
       robot_state.data = 3;
       robot_state_pub.publish(robot_state);//enroute drop off location
       // Wait an infinite time for the results
       ac.waitForResult();

       // Check if the robot reached its goal
       if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       {
           ROS_INFO(" Robot is at drop off location now");
           robot_state.data = 4;
           robot_state_pub.publish(robot_state);//reached drop off location
       }
       else
       {
           ROS_INFO("Robot failed to reach drop off location for some reason");
           robot_state.data = 0;
           robot_state_pub.publish(robot_state);//could n't reach to pick up/ drop off location
       }
   }

   return 0; 
}

