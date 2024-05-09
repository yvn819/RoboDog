#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

class MarkerPublisher {
public:
    MarkerPublisher(): hz(50.0){
        markerPub = nh.advertise<visualization_msgs::Marker>("marker", 1);
        current_goal = nh.subscribe("current_goal", 1, &MarkerPublisher::getCurrentGoal, this);

        // Initialize the marker
        marker.header.frame_id = "world"; // Replace "map" with your desired frame
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1; // Change the scale as needed
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0; // Set marker color to red
        marker.color.a = 1.0; // Set marker alpha to fully opaque
        marker.lifetime = ros::Duration(1/hz);
    }

    void getCurrentGoal(const geometry_msgs::Point& current_goal){
        marker.pose.position.x = current_goal.x;
        marker.pose.position.y = current_goal.y;
        marker.pose.position.z = current_goal.z;

        markerPub.publish(marker);
    }

    void publishMarker(const Eigen::Vector3d& position) {
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        markerPub.publish(marker);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher markerPub;
    ros::Subscriber current_goal;
    visualization_msgs::Marker marker;
    double hz; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_publisher_node");
    MarkerPublisher markerPublisher;
    ros::spinOnce();


    return 0;
}
