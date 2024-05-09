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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <string>

#define PI M_PI
#define START_X  0
#define START_Y 0

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

    Eigen::Vector2d x_plane; 
    Eigen::Vector2d xd_plane;
    Eigen::Vector2d v_plane;
    Eigen::Vector2d quasi_goal;


    double yawd;           // desired yaw angle

    double hz; 
                // frequency of the main control loop


    ros::Publisher filtered_cloud_pub, current_goal_pub;
    ros::Subscriber point_cloud_sub, current_state, current_goal;
    octomap::OcTree tree;

    geometry_msgs::Point current_goal_msg;

    int state = 2;

    bool receive_goal = true;
    // bool at_start_point = false;

    // void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);


  public:
    controllerNode():hz(50), tree(0.05){

      current_goal = nh.subscribe("/desired_state", 1, &controllerNode::getDesiredState, this);
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::getCurrentState, this);
      // current_state = nh.subscribe("/true_twist", 1, &controllerNode::getCurrentTwist, this);

      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      // octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
      
      current_goal_pub = nh.advertise<geometry_msgs::Point>("/current_goal",1);

      // timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
      timer = nh.createTimer(ros::Rate(0.5), &controllerNode::isAtDesiredState, this);

    }

    void getCurrentState(const nav_msgs::Odometry& current_state){
      x << current_state.pose.pose.position.x, current_state.pose.pose.position.y, current_state.pose.pose.position.z;       
      x_plane << current_state.pose.pose.position.x, current_state.pose.pose.position.y;
      if(current_state.twist.twist.linear.x >= 0 || current_state.twist.twist.linear.y >= 0){
        v << current_state.twist.twist.linear.x, current_state.twist.twist.linear.y, current_state.twist.twist.linear.z;
        v_plane << current_state.twist.twist.linear.x, current_state.twist.twist.linear.y;
      }
      // round v
      // v(0) = (v(0) >= -0.05 && v(0) <= 0.05) ? 0.0 : v(0);
      // v(1) = (v(1) >= -0.05 && v(1) <= 0.05) ? 0.0 : v(1);
    }

    // void getCurrentTwist(const geometry_msgs::TwistStamped& current_twist){
    //   if(current_twist.twist.linear.x >= 0 || current_twist.twist.linear.y >= 0){
    //     v << current_twist.twist.linear.x, current_twist.twist.linear.y, current_twist.twist.linear.z;
    //     v_plane << current_twist.twist.linear.x, current_twist.twist.linear.y;
    //   }      
    // }

    void getDesiredState(const geometry_msgs::Point& next_desired_point){

      if (receive_goal){
        if (next_desired_point.x >= x(0) || next_desired_point.y >= x(1)){
          xd << next_desired_point.x, next_desired_point.y, next_desired_point.z;
          xd_plane << next_desired_point.x, next_desired_point.y;
          
          current_goal_msg.x = xd(0);
          current_goal_msg.y = xd(1);
          current_goal_msg.z = xd(2);
          receive_goal = false;          
        }
        else{
          controlLoop(1);
        }

      }
      // ROS_INFO_STREAM("current goal: " << current_goal_msg.x << " " << current_goal_msg.y);
      current_goal_pub.publish(current_goal_msg);
      // ROS_INFO_STREAM("xd: " << xd);
      
    }

    void isAtDesiredState(const ros::TimerEvent& t){
      // try local goal if path's performance not good
      Eigen::Vector2d hyper_goal = checkAndAssign(x_plane);
      if (hyper_goal.x() != 0 && hyper_goal.y() !=0){
        xd_plane << hyper_goal.x(), hyper_goal.y();
        ROS_INFO("goal set");
      }

      mav_msgs::Actuators msg;  
      msg.angular_velocities.resize(5);
      // Compute the desired heading direction (angle between xd and x)
      Eigen::Vector3d direction = xd - x;
      Eigen::Vector2d direction_plane = xd_plane - x_plane;
      double distance = direction_plane.norm();
      double rotation_angle = turnAngle(xd_plane, x_plane);

      ROS_INFO_STREAM("xd: " << xd_plane);
      ROS_INFO_STREAM("x: " << x);
      ROS_INFO_STREAM("v: " << v);

      ROS_INFO_STREAM("direction: " << direction);
      ROS_INFO_STREAM("distance: " << distance);
      ROS_INFO_STREAM("rotation_angle: " << rotation_angle);

      // Determine the control plan
  
      // reach the desired state

      if (xd(0) == 0 && xd(1) == 0){
        if(distance >= 0.15){
          state = 4;
          ROS_INFO("try to start");
          controlLoop(state);
        }
        else{
          state = 1;
          receive_goal = true;
          controlLoop(state);
        }
      }
      else if (distance < 0.2){
        receive_goal = true;
        state = 1;  // "Stop"
        ROS_INFO("Stop");
        controlLoop(state);

      }

      else if (state == 2 || state ==3){
        state = 6;  // turn -> rest
        ROS_INFO("rest");
        controlLoop(state);   
      }

      else if (state ==6){

        state = 5;
        ROS_INFO("move a bit");
        controlLoop(state);
          
      }

      else if ((rotation_angle < M_PI && rotation_angle > M_PI/4) || (rotation_angle < -M_PI/4 && rotation_angle > -M_PI)){
        if (rotation_angle < -M_PI/4 && rotation_angle > -M_PI){
          state = 3;  

          ROS_INFO("Ready to rotate counterclockwise");
          controlLoop(state);

          // controlLoop(4); //move a bit 

        }
        else if (rotation_angle < M_PI && rotation_angle > M_PI/6){
          state = 2;  
          ROS_INFO("Ready to rotate clockwise");
          controlLoop(state);
          
          // controlLoop(4); // move a bit 

        }
        
      }

      // if in target direction but not arrive, go forward
      else{
        state = 4;  //"GoForward"
        ROS_INFO("Ready to go forward");
        controlLoop(state);
      }
           
    }


    double turnAngle(Eigen::Vector2d point, Eigen::Vector2d cur_pos){
      Eigen::Vector2d direction = point - cur_pos;

      double desired_heading = std::atan2(direction.y(), direction.x());

      // Compute the current heading direction (angle of v)
      double current_heading = std::atan2(v.y(), v.x());

      // Compute the required rotation angle
      double rotation_angle = current_heading - desired_heading;

      // Ensure that the rotation angle is within the range [-pi, pi]
      if (rotation_angle > M_PI) {
          rotation_angle -= 2*M_PI;
      } else if (rotation_angle < -M_PI) {
          rotation_angle += 2*M_PI;
      }
      return rotation_angle;

    }


    double checkAngle(Eigen::Vector2d point, Eigen::Vector2d cur_pos){
      Eigen::Vector2d direction = point - cur_pos;

      double desired_heading = std::atan2(direction.y(), direction.x());

      // Compute the current heading direction (angle of v)
      double current_heading = std::atan2(v.y(), v.x());

      // Compute the required rotation angle
      double rotation_angle = current_heading - desired_heading;

      // Ensure that the rotation angle is within the range [-pi, pi]
      // if (rotation_angle > M_PI/2) {
      //     rotation_angle -= M_PI;
      // } else if (rotation_angle < -M_PI/2) {
      //     rotation_angle += M_PI;
      // }
      return rotation_angle;

    }


    Eigen::Vector2d checkAndAssign(const Eigen::Vector2d& x) {
      double x0 = x.x(); // Access the first element of x
      double y0 = x.y(); // Access the second element of x
      Eigen::Vector2d xd;
      xd << 0, 0;
      // first turn
      if (x0 >= -0.5 && x0 <= 0.5 && y0 >=0.8 && y0 <=2) {
        // Assign the target point coordinates to xd
        xd << 1, 1.5;

        // receive_goal = true;
      }
      else if (x0 >= 1 && x0 <= 2 && y0 >=1 && y0 <=2){
        xd << 2.5, 1.5;
      }
      else if (x0 >= 2.5 && x0 <= 3.5 && y0 >=1 && y0 <=2){
        xd << 4, 0.5;
      }
      else if (x0 >= 3.5 && x0 <= 4.5 && y0 >=0 && y0 <=1){
        xd << 5.5, 1;
      }
      else if (x0 >= 5 && x0 <= 6 && y0 >=0 && y0 <=1){
        xd << 5.5, 6;
      }      
      return xd;
      // Check if x1 is in the interval [1, 2]
      // if (x1 >= 1.0 && x1 <= 2.0) {
      //     // Assign the target point coordinates to xd
      //     xd(1) = /* Put the desired value for x1 in the interval [1, 2] */;
      // }
    }

    void controlLoop(int state){

      mav_msgs::Actuators msg;
      msg.angular_velocities.resize(5);
      switch(state){
        case 1:   // Slow down
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 90; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 2; 
          commands.publish(msg);
          break;

        case 2:    //Clockwise
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 30; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 10;  
          commands.publish(msg);
          for (int i = 0; i < 20; ++i) {
            ros::Rate rate_command(10);
            commands.publish(msg);
            rate_command.sleep();
          }
          break;

        case 3:     // CounterClockwise
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 150; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 10; 
          commands.publish(msg);
          for (int i = 0; i < 30; ++i) {
            ros::Rate rate_command(10);
            commands.publish(msg);
            rate_command.sleep();
          }
          break;  

        case 4:    // GoForward for a while
          // jumping mode
          // msg.angular_velocities[0] = 90; 
          // msg.angular_velocities[1] = 0; 
          // msg.angular_velocities[2] = 10; 
          // msg.angular_velocities[3] = -5; 
          // msg.angular_velocities[4] = 7; 
          // walking mode
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 90; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 7;   
          for (int i = 0; i < 30; ++i) {
            ros::Rate rate_command(10);
            commands.publish(msg);
            rate_command.sleep();
          }
          break; 

        case 5:    // GoForward
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 90; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 7;
          commands.publish(msg);
          break;     

        case 6:   // rest
          msg.angular_velocities[0] = 0; 
          msg.angular_velocities[1] = 90; 
          msg.angular_velocities[2] = 0; 
          msg.angular_velocities[3] = 0; 
          msg.angular_velocities[4] = 0; 
          commands.publish(msg);
          break;
      }
      

      // msg.angular_velocities[0] = 0; // Phase between front and back legs (in degree) up/downstairs?
      // msg.angular_velocities[1] = 90; // Phase between front left + back right legs and front right and left back legs
      //                                 // 0,90 clockwise 90,180 conterclockwise turn
      // msg.angular_velocities[2] = 0; // Amplitude change of all legs
      // msg.angular_velocities[3] = 0; // Amplitude change of back legs (added to angular_velocities[2])
      // msg.angular_velocities[4] = 7; // Frequency of legs velocity

    }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
