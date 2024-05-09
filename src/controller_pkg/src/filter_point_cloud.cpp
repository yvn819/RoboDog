#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <string>

octomap::OcTree tree(0.1);
ros::Subscriber point_cloud_sub;
ros::Publisher filtered_cloud_pub, octomap_pub;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    filtered_cloud_pub.publish(cloud_msg);

    // Convert pcl::PointCloud<pcl::PointXYZ> to octomap::Pointcloud
    octomap::Pointcloud octo_cloud;
    for (const auto& point : cloud->points) {
        octo_cloud.push_back(point.x, point.y, point.z);
    }

    //convert sensor_msgs::PointCloud2::ConstPtr to octomap::Pointcloud
    // for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i) {
    //   float x = 0.0, y = 0.0, z = 0.0;

    //   // Get the data offset for x, y, and z fields
    //   int x_offset = cloud_msg->fields[0].offset;
    //   int y_offset = cloud_msg->fields[1].offset;
    //   int z_offset = cloud_msg->fields[2].offset;

    //   // Copy the data from the cloud_msg to octomap::Pointcloud
    //   memcpy(&x, &cloud_msg->data[i * cloud_msg->point_step + x_offset], sizeof(float));
    //   memcpy(&y, &cloud_msg->data[i * cloud_msg->point_step + y_offset], sizeof(float));
    //   memcpy(&z, &cloud_msg->data[i * cloud_msg->point_step + z_offset], sizeof(float));

    //   octo_cloud.push_back(x, y, z);
    // }

    // Clear the octomap
    tree.clear();

    // Update the octomap with the filtered point cloud
    tree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0)); // Use the correct origin of the point cloud

    // Publish the updated octomap
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::binaryMapToMsg(tree, octomap_msg);
    octomap_pub.publish(octomap_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_clould_point");
    ros::NodeHandle nh;

    

    // Create the publisher and subscriber
    // ... (If you have other publishers or subscribers, create them here) ...

    // Subscribe to the point cloud topic and attach the callback function
    point_cloud_sub = nh.subscribe("/pointcloud", 1, pointCloudCallback);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
    ros::spin();

    return 0;
}


