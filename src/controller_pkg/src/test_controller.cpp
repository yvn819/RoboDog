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


#define PI M_PI

#define CLOCKWISE_TURN 8
#define COUNTERCLOCKWISE_TURN -8
#define UPSTAIR 2
#define SLOPE 3
#define STOP -1


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  private:
    ros::NodeHandle nh;

    ros::Publisher commands;
    ros::Publisher octomap_pub;
    ros::Timer timer;                             

    // Eigen::Vector3d 三维向量
    // Controller internals (you will have to set them below)
    // Current state 
    Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
    Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
    Eigen::Matrix3d R;     // current orientation of the UAV
    Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

    // Desired state
    Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
    Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
    Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
    double yawd;           // desired yaw angle

    double hz;             // frequency of the main control loop

    ros::Publisher filtered_cloud_pub;
    ros::Subscriber point_cloud_sub ,current_state;
    octomap::OcTree tree;

    

    // void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);


  public:
    controllerNode():hz(1000.0), tree(0.05){

      point_cloud_sub = nh.subscribe("/pointcloud", 1, &controllerNode::pointCloudCallback, this);
    //   current_goal = nh.subscribe("/current_goal", 1, &controllerNode::getDesiredState, this);
    //   current_state = nh.subscribe("current_state_est", 1, &controllerNode::getCurrentState, this);

      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
      filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);

      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

    }


    void controlLoop(const ros::TimerEvent& t){

      mav_msgs::Actuators msg;
      msg.angular_velocities.resize(5);


      msg.angular_velocities[0] = 0; // Phase between front and back legs (in degree) up/downstairs?
      msg.angular_velocities[1] = 135; // Phase between front left + back right legs and front right and left back legs
                                      // 0,90 clockwise 90,180 conterclockwise turn
      msg.angular_velocities[2] = 0; // Amplitude change of all legs
      msg.angular_velocities[3] = 0; // Amplitude change of back legs (added to angular_velocities[2])
      msg.angular_velocities[4] = 7; // Frequency of legs velocity

      commands.publish(msg);

    }


    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
      // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
      // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromROSMsg(*cloud_msg, *cloud);

      filtered_cloud_pub.publish(cloud_msg);

      // Convert pcl::PointCloud<pcl::PointXYZ> to octomap::Pointcloud
      octomap::Pointcloud octo_cloud;
      // for (const auto& point : cloud->points) {
      //   octo_cloud.push_back(point.x, point.y, point.z);
      // }

      //convert sensor_msgs::PointCloud2::ConstPtr to octomap::Pointcloud
      for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i) {
        float x = 0.0, y = 0.0, z = 0.0;

        // Get the data offset for x, y, and z fields
        int x_offset = cloud_msg->fields[0].offset;
        int y_offset = cloud_msg->fields[1].offset;
        int z_offset = cloud_msg->fields[2].offset;

        // Copy the data from the cloud_msg to octomap::Pointcloud
        memcpy(&x, &cloud_msg->data[i * cloud_msg->point_step + x_offset], sizeof(float));
        memcpy(&y, &cloud_msg->data[i * cloud_msg->point_step + y_offset], sizeof(float));
        memcpy(&z, &cloud_msg->data[i * cloud_msg->point_step + z_offset], sizeof(float));

        octo_cloud.push_back(x, y, z);
      }

      // Clear the octomap
      tree.clear();

      // Update the octomap with the filtered point cloud
      tree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0)); // Use the correct origin of the point cloud

      // Publish the updated octomap
      octomap_msgs::Octomap octomap_msg;
      octomap_msgs::binaryMapToMsg(tree, octomap_msg);
      octomap_msg.header.frame_id = "world";
      octomap_pub.publish(octomap_msg);
    }




};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
