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

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose = *msg;
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
}

// void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     nav_msgs::Odometry odom = *msg;
//     geometry_msgs::PoseStamped pose;
//     pose.header = odom.header;
//     pose.pose.position.x = odom.pose.pose.position.x;
//     pose.pose.position.y = odom.pose.pose.position.y;
//     pose.pose.position.z = odom.pose.pose.position.z;
//     pose.pose.orientation.x = odom.pose.pose.orientation.x; 
//     pose.pose.orientation.y = odom.pose.pose.orientation.y; 
//     pose.pose.orientation.z = odom.pose.pose.orientation.z; 
//     pose.pose.orientation.w = odom.pose.pose.orientation.w; 
//     poses.push_back(pose);
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh("~");
    nh.param("use_gps", use_gps, false);
    nh.param("robot", robot_id, std::string(""));
    nh.param("cmd_vel",cmd_vel, std::string("/cmd_vel"));
    nh.param("is_sim",is_sim, false);

    ros::Subscriber pose_sub = nh.subscribe(robot_id + "/posestamped", 10, poseCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(robot_id + "path", 10);
    ros::Rate rate(10);

    while (ros::ok()) {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";

        path.poses = poses;

        path_pub.publish(path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
