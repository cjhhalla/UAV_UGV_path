#ifndef FAIL_SAFE_H
#define FAIL_SAFE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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

class Failsafe {
public:
    Failsafe(ros::NodeHandle& nh);
    void flagCallback(const std_msgs::Bool::ConstPtr& msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void main();
    // void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // void poseCallback_sim(const gazebo_msgs::ModelStates::ConstPtr& msg);
    // double moveX(const geometry_msgs::PoseStamped& goal);
    // void purePursuitControl(const geometry_msgs::PoseStamped& goal, double look_ahead_distance);
    // std::pair<double, double> calculateLookAheadPoint(const std::pair<double, double>& current_position, const std::pair<double, double>& goal_position, double look_ahead_distance);
    // void stop();
    // std::pair<double, double> rotateGoal(double goal_x, double goal_y);

    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_sub, flag_sub, position_sub, cmd_vel_sub;
    double kp, kd, prev_error, length, max_speed, look_ahead;
    int laps_completed;
    bool use_gps, fail_safe, initial_pose, is_sim, init_;
    double roll, pitch, yaw, init_yaw, init_x, init_y, init_z, L;
    std::string robot_id, cmd_vel, model_name;
    std::vector<double> yaw_vec;
    std::vector<double> x_pos_vec;
    std::vector<double> y_pos_vec;
    geometry_msgs::Twist cmd_tem;
    geometry_msgs::PoseStamped pose;
    ros::Publisher vis_pub;
};

#endif 
