#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>


// Variable to hold Pose (only x,y, not orientation) info
//std::vector<double> pose{0,0};
double pose_x, pose_y, type_encoding;
ros::Publisher marker_pub;
//void show_marker();
visualization_msgs::Marker marker;
double state_encoding;

void show_marker()
{
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ObjectMarker";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose_x;
    marker.pose.position.y = pose_y;
    marker.pose.position.z = 0.125;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.0f;
    marker.lifetime = ros::Duration();
    marker.header.stamp = ros::Time::now();
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = type_encoding;
    if (state_encoding == 1 || state_encoding == 4)
    {
       marker.color.a = 1.0f;
    }
    else if (state_encoding == 2 || state_encoding == 3)
    {
       marker.color.a = 0.0f;
    } 
    else // 0 means some problem
    {
      marker.color.a = 0.0f;
    }   
    marker_pub.publish(marker);
}

// Callback to goal Pose of Robot 
void goal_pose_callback(const geometry_msgs::Point::ConstPtr& goalPoseMsg)
{
    pose_x = goalPoseMsg->x;
    pose_y = goalPoseMsg->y;
    type_encoding = goalPoseMsg->z;
    ROS_INFO("Goal is at %.2f %.2f : ",pose_x, pose_y);
    show_marker();
}
// Callback for robot state during maneuver plan
void robot_state_callback(const std_msgs::UInt8::ConstPtr& stateMsg)
{
    state_encoding = stateMsg->data;
    show_marker();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"ObjectMarker");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("Visualization_marker", 1);
    ros::Subscriber goalPose_sub = n.subscribe<geometry_msgs::Point>("send_pose",1,goal_pose_callback);
    ros::Subscriber robotState_sub = n.subscribe<std_msgs::UInt8>("robot_state",1,robot_state_callback);
    ros::spin();
    return 0;
}
