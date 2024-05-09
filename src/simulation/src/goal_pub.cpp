#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h> 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");

    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_topic", 10);

    // Create a goal for move_base
    move_base_msgs::MoveBaseGoal goal;

    // Set the frame ID of the goal pose (e.g., "map" or "odom")
    goal.target_pose.header.frame_id = "world";

    // Set the timestamp of the goal pose (usually set to ros::Time::now())
    goal.target_pose.header.stamp = ros::Time::now();

    // Set the goal position (x, y, z)
    goal.target_pose.pose.position.x = 2.5;      // 2.5
    goal.target_pose.pose.position.y = 9.5;      // 9,5
    goal.target_pose.pose.position.z = 0.0;

    // Set the goal orientation (quaternion)
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Create a PoseStamped message and set the goal pose
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header = goal.target_pose.header;
    goal_msg.pose = goal.target_pose.pose;

    // Set the publishing rate (e.g., 1 Hz)
    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        // Publish the goal on the custom_goal_topic
        goal_pub.publish(goal_msg);
        // ROS_INFO("Goal message is published!");
        // Sleep to control the publishing rate
        rate.sleep();
    }

    return 0;
}
