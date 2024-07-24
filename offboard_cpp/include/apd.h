#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <vector>

class ApdControl
{
public:
    ApdControl();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void get_waypoint();
    void publish_waypoint();
    void arming();


private:
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;

    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber init_local_pos_sub;

    ros::Publisher local_st_pub;
    ros::Publisher local_vel_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient land_client;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    mavros_msgs::CommandTOL land_cmd;

    std::vector<geometry_msgs::PoseStamped> waypoint_pose;
    std::vector<double> x_pos;
    std::vector<double> y_pos;
    std::vector<double> z_pos;

    bool verbal_flag;
    bool init_local_pose_check;

    int waypoint_count;
    int num_waypoint;

    ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    
};