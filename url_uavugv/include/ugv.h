#ifndef UGV_H
#define UGV_H

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

class UGV {
public:
    UGV(ros::NodeHandle* nh);
    // void flagCallback(const std_msgs::Bool::ConstPtr& msg);
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseCallback_sim(const gazebo_msgs::ModelStates::ConstPtr& msg);
    double moveX(const geometry_msgs::PoseStamped& goal);
    void purePursuitControl(const geometry_msgs::PoseStamped& goal, double look_ahead_distance);
    std::pair<double, double> calculateLookAheadPoint(const std::pair<double, double>& current_position, const std::pair<double, double>& goal_position, double look_ahead_distance);
    void stop();
    std::pair<double, double> rotateGoal(double goal_x, double goal_y);
    void loadParameters();

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub, pose_pub;
    ros::Subscriber pose_sub, position_sub;
    nav_msgs::Odometry pose;
    double kp, kd, prev_error, length, max_speed, look_ahead;
    int laps_completed;
    bool use_gps, fail_safe, initial_pose, is_sim, cw_ccw;
    double roll, pitch, yaw, init_yaw, init_x, init_y, L;
    std::string robot_id, cmd_vel, model_name,model_name_;
    std::vector<double> yaw_vec;
    std::vector<double> x_pos_vec;
    std::vector<double> y_pos_vec;
    double center_x, center_y, center1_x, center2_x, radius;

};

#endif // JACKAL_H
