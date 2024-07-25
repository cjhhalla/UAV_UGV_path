#include "fail_safe.h"
#include <numeric> 
#include <algorithm> 
#include <cmath>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker marker;

Failsafe::Failsafe(ros::NodeHandle& nh) {
    nh.param("use_gps", use_gps, false);
    nh.param("robot", robot_id, std::string(""));
    nh.param("cmd_vel",cmd_vel, std::string("/cmd_vel"));
    nh.param("is_sim",is_sim, false);
    // ROS_INFO("use_gps: %s", use_gps ? "true" : "false");
    // ROS_INFO("cmd_vel: %s", cmd_vel.c_str());
    // ROS_INFO("robot_id: %s", robot_id.c_str());
    // ROS_INFO("is_sim: %s", is_sim ? "true" : "false");
    vis_pub = nh.advertise<visualization_msgs::Marker>(robot_id + "visualization_marker", 10 );
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(robot_id + cmd_vel, 10);
    flag_sub = nh.subscribe(robot_id + "/is_safe", 10, &Failsafe::flagCallback, this);
    cmd_vel_sub = nh.subscribe(robot_id + cmd_vel + "/temp", 10, &Failsafe::cmdvelCallback, this);
    pose_sub = nh.subscribe(robot_id + "/posestamped", 10, &Failsafe::poseCallback, this);
    fail_safe = true;
    init_ = true;
}

void Failsafe::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose = *msg;
    if (init_){
        init_x = pose.pose.position.x;
        init_y = pose.pose.position.y;
        init_z = pose.pose.position.z;
        init_ = false;
    }
    else if (!init_){
        pose.pose.position.x = pose.pose.position.x - init_x;
        pose.pose.position.y = pose.pose.position.y - init_y;
        pose.pose.position.z = pose.pose.position.z - init_z;
    }
}

void Failsafe::flagCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == true){
        fail_safe = true;
    }
    else if (msg->data == false){
        fail_safe = false;
    }
}

void Failsafe::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_tem = *msg;
}

void Failsafe::main(){
    if (!fail_safe){
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_vel_pub.publish(cmd);
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "world";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.pose.position.x;
        marker.pose.position.y = pose.pose.position.y;
        marker.pose.position.z = pose.pose.position.z;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
        // ROS_INFO("%s FAIL SAFE MODE DETECTED!", robot_id.c_str());
    }
    else {
        cmd_vel_pub.publish(cmd_tem);
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
        marker.action = visualization_msgs::Marker::DELETE;
        marker.id = 0; 
        vis_pub.publish(marker);
        // ROS_INFO("%s FAIL SAFE NOW WORKING!", robot_id.c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fail_safe_detector");
    ros::NodeHandle nh("~");
    Failsafe fail(nh);
    ros::Rate rate(25);
    while (ros::ok()) {
        fail.main();
        rate.sleep();
        ros::spinOnce();
    }
}
