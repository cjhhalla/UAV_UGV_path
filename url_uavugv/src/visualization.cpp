#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <numeric>
#include <string>
#include <utility>
#include <tf/tf.h>



std::string model_name = "jackal";
std::vector<geometry_msgs::PoseStamped> poses;
bool use_gps, is_sim;
std::string robot_id, cmd_vel;
bool init_ = true;
double init_x = 0;
double init_y = 0;
double init_z = 0;

visualization_msgs::Marker points;

void initializeMarker() {
    points.header.stamp = ros::Time();
    points.header.frame_id = "world";  
    points.action = visualization_msgs::Marker::ADD;
    points.id = 10;
    points.type = visualization_msgs::Marker::CUBE;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.scale.z = 0.5;
    points.color.r = 0.0;
    points.color.b = 1.0;
    points.color.g = 0.0;
    points.color.a = 1.0;
    points.pose.orientation.w = 1.0;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped pose = *msg;
    // ROS_INFO("POSESTAMP SUB");
    if (init_){
        init_x = pose.pose.position.x;
        init_y = pose.pose.position.y;
        init_z = pose.pose.position.z;
        init_ = false;
    }
    else if (!init_){
        pose.pose.position.x = pose.pose.position.x - init_x;
        pose.pose.position.y = pose.pose.position.y - init_y;
        poses.push_back(pose);
    }
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "world";  
    transformStamped.child_frame_id = robot_id;  

    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;
    transformStamped.transform.rotation = pose.pose.orientation;

    br.sendTransform(transformStamped);
}

void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    points.pose.position.x = msg->pose.position.x - init_x;
    points.pose.position.y = msg->pose.position.y - init_y;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh("~");
    nh.param("use_gps", use_gps, false);
    nh.param("robot", robot_id, std::string(""));
    nh.param("cmd_vel",cmd_vel, std::string(""));
    nh.param("is_sim",is_sim, false);
    ROS_INFO("visualization_cmd_vel: %s", cmd_vel.c_str());
    initializeMarker();

    ros::Subscriber pose_sub = nh.subscribe("/"+robot_id + "/posestamped", 10, poseCallback);
    ros::Subscriber waypoint_sub = nh.subscribe("/"+robot_id + "/waypoints", 10, waypointCallback);
    ros::Publisher waypoint_pub = nh.advertise<visualization_msgs::Marker>("/"+robot_id + "/waypoints_marker", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/"+robot_id + "/path", 10);
    ros::Rate rate(10);

    while (ros::ok()) {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";

        path.poses = poses;
        waypoint_pub.publish(points);
        path_pub.publish(path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
