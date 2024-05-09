#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>


#define PI M_PI
#define START_X 0
#define START_Y 1

class pathPlanningNode{
  private:
    ros::NodeHandle nh;
    ros::Subscriber current_state, path_sub;
    ros::Publisher desired_state_pub; 
    ros::Timer timer; 

    Eigen::Vector2d x;   
    Eigen::Vector3d v;  

    double hz;
    double distance;
    std::vector<Eigen::Vector2d> path;
    bool at_start_point = false;
  
  
  public:
    pathPlanningNode():hz(50.0){

      current_state = nh.subscribe("current_state_est", 1, &pathPlanningNode::getCurrentState, this);
      // path_sub = nh.subscribe("/move_base_Robodog/TrajectoryPlannerROS/global_plan", 100, &pathPlanningNode::pathCallback, this);
      path_sub = nh.subscribe("/move_base_Robodog/NavfnROS/plan", 100, &pathPlanningNode::pathCallback, this);
      
      // timer = nh.createTimer(ros::Duration(1/hz), &pathPlanningNode::planningLoop, this);

      desired_state_pub = nh.advertise<geometry_msgs::Point>("/desired_state",1);

    }
    void getCurrentState(const nav_msgs::Odometry& current_state){

      // x << current_state.pose.pose.position.x, current_state.pose.pose.position.y, current_state.pose.pose.position.z;       
      x << current_state.pose.pose.position.x, current_state.pose.pose.position.y;      
      v << current_state.twist.twist.linear.x, current_state.twist.twist.linear.y, current_state.twist.twist.linear.z;

    }

    //get path from global planner
    
    void pathCallback(const nav_msgs::Path& path_info) {
      Eigen::Vector2d start;
      start << START_X, START_Y;
      // If not at the starting point yet.
      if (!at_start_point){
        ROS_INFO_ONCE("ready to start");
        isAtStartPoint(start, x);
        geometry_msgs::Point desired_state_msg;
        desired_state_msg.x = START_X;
        desired_state_msg.y = START_Y;
        desired_state_msg.z = 0;
        desired_state_pub.publish(desired_state_msg);
      }

      // if already pass the starting point
      else{
        for (const auto& pose : path_info.poses) {

          Eigen::Vector2d point;
          // path.clear();
          // point << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
          if (pose.pose.position.x >= -1 && pose.pose.position.x <=7 && pose.pose.position.y >=0 && pose.pose.position.y <= 9.5){
            point << pose.pose.position.x, pose.pose.position.y;
          }
          // distance = (point.head<2>() - x.head<2>()).norm();
          distance = (point - x).norm();
          double rotation_angle = turnAngle(point, x);

          // ROS_INFO_STREAM("Distance: " << distance);
          // ROS_INFO_STREAM("Rotation Angle: " << rotation_angle);


          // The target point is in front of the current position and at a distance of 0.5 - 1
          if (rotation_angle >= -M_PI/2 && rotation_angle <= M_PI/2 && distance >= 0.5 && distance <= 1.0) {
            // ros::Rate rate_command(10);
            geometry_msgs::Point desired_state_msg;
            desired_state_msg.x = point(0);
            desired_state_msg.y = point(1);
            desired_state_msg.z = pose.pose.position.z;
            desired_state_pub.publish(desired_state_msg);
            ROS_INFO("rotation angle:%f", rotation_angle);
            ROS_INFO("desired state: %f, %f" , point(0), point(1));
            break;
          }
            // path.push_back(point);
      }
      }

    }

    void isAtStartPoint(Eigen::Vector2d start_point, Eigen::Vector2d cur_pos){
      double dis = (start_point - cur_pos).norm();
      if (dis <= 0.2){
        at_start_point = true;
      }
    }

    double turnAngle(Eigen::Vector2d point, Eigen::Vector2d cur_pos){
      Eigen::Vector2d direction = point - cur_pos;

      double desired_heading = std::atan2(direction.y(), direction.x());

      // Compute the current heading direction (angle of v)
      double current_heading = std::atan2(v.y(), v.x());

      // Compute the required rotation angle
      double rotation_angle = desired_heading - current_heading;

      // Ensure that the rotation angle is within the range [-pi, pi]
      if (rotation_angle > M_PI) {
          rotation_angle -= 2 * M_PI;
      } else if (rotation_angle < -M_PI) {
          rotation_angle += 2 * M_PI;
      }
      return rotation_angle;

    }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "pathplanning_node");
  pathPlanningNode n;
  ros::spin();
}
