#include "ugv.h"
#include <numeric> 
#include <algorithm> 
#include <cmath>
#include <string>

UGV::UGV(ros::NodeHandle& nh) {
    nh.param("length", length, 3.0);
    nh.param("center_x", center_x, 0.0);
    nh.param("center_y", center_y, 0.0);
    nh.param("center1_x", center1_x, 0.0);
    nh.param("center2_x", center2_x, 0.0);
    nh.param("radius", radius, 3.0);    
    nh.param("max_speed", max_speed, 2.0);
    nh.param("laps_completed", laps_completed, 1);
    nh.param("use_gps", use_gps, false);
    nh.param("robot", robot_id, std::string(""));
    nh.param("look_ahead", look_ahead, 1.2);
    nh.param("cmd_vel",cmd_vel, std::string("/cmd_vel"));
    nh.param("is_sim",is_sim, false);
    ROS_INFO("robot_id: %s", robot_id.c_str());
    ROS_INFO("length: %f", length);
    ROS_INFO("max_speed: %f", max_speed);
    ROS_INFO("laps_completed: %d", laps_completed);
    ROS_INFO("use_gps: %s", use_gps ? "true" : "false");
    ROS_INFO("look_ahead_distance: %f", look_ahead);
    ROS_INFO("cmd_vel: %s", cmd_vel.c_str());

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(robot_id + cmd_vel + "/temp", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(robot_id + "/posestamped", 10);
    if(is_sim){
        pose_sub = nh.subscribe("/gazebo/model_states", 10, &UGV::poseCallback_sim, this);
    }
    else if (!use_gps) {
        pose_sub = nh.subscribe(robot_id + "/mavros/local_position/odom", 10, &UGV::poseCallback, this);
    } else {
        position_sub = nh.subscribe(robot_id + "/mavros/global_position/local", 10, &UGV::poseCallback, this);
    }

    kp = 1.0;
    kd = 0.1;
    prev_error = 0.0;
    L = 0.5;
    yaw = 0.0;
    initial_pose = false;
    init_yaw = 0.0;
    init_x = 0.0;
    init_y = 0.0;
    fail_safe = true;
    model_name = "jackal";
}

// void UGV::flagCallback(const std_msgs::Bool::ConstPtr& msg) {
//     fail_safe = msg->data;
// }

void UGV::poseCallback_sim(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    auto it = std::find(msg->name.begin(), msg->name.end(), model_name);
    if (it != msg->name.end()) {
        int index = std::distance(msg->name.begin(), it);
        const geometry_msgs::Pose& pose_ = msg->pose[index];
        pose.pose.pose.position = pose_.position;
        tf::Quaternion quat(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        if (!initial_pose) {
            yaw_vec.push_back(yaw);
            x_pos_vec.push_back(pose.pose.pose.position.x);
            y_pos_vec.push_back(pose.pose.pose.position.y);
            if (yaw_vec.size() > 10 && x_pos_vec.size() > 10 && y_pos_vec.size() > 10) {
                init_yaw = std::accumulate(yaw_vec.begin(), yaw_vec.end(), 0.0) / yaw_vec.size();
                init_x = std::accumulate(x_pos_vec.begin(), x_pos_vec.end(), 0.0) / x_pos_vec.size();
                init_y = std::accumulate(y_pos_vec.begin(), y_pos_vec.end(), 0.0) / y_pos_vec.size();
                ROS_INFO("initial yaw: %f", init_yaw);
                ROS_INFO("initial x_pos: %f", init_x);
                ROS_INFO("initial y_pos: %f", init_y);
                initial_pose = true;
            }
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = pose_;
        pose_pub.publish(pose_stamped);
    } else {
        ROS_WARN("Model %s not found in /gazebo/model_states", model_name.c_str());
    }
}

void UGV::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    pose = *msg;
    auto orientation_q = pose.pose.pose.orientation;
    tf::Quaternion quat(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (!initial_pose) {
        yaw_vec.push_back(yaw);
        x_pos_vec.push_back(pose.pose.pose.position.x);
        y_pos_vec.push_back(pose.pose.pose.position.y);
        if (yaw_vec.size() > 25 && x_pos_vec.size() > 25 && y_pos_vec.size() > 25) {
            init_yaw = std::accumulate(yaw_vec.begin(), yaw_vec.end(), 0.0) / yaw_vec.size();
            init_x = std::accumulate(x_pos_vec.begin(), x_pos_vec.end(), 0.0) / x_pos_vec.size();
            init_y = std::accumulate(y_pos_vec.begin(), y_pos_vec.end(), 0.0) / y_pos_vec.size();
            ROS_INFO("initial yaw: %f", init_yaw);
            ROS_INFO("initial x_pos: %f", init_x);
            ROS_INFO("initial y_pos: %f", init_y);
            initial_pose = true;
        }
    }
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = pose.pose.pose;
    pose_pub.publish(pose);
}

double UGV::moveX(const geometry_msgs::PoseStamped& goal) {
    auto goal_ = goal.pose.position;
    double error_x = goal_.x - pose.pose.pose.position.x;
    double error_y = goal_.y - pose.pose.pose.position.y;
    double error = sqrt(pow(error_x, 2) + pow(error_y, 2));
    double dt = 0.01;
    double speed = kp * error + kd * (error - prev_error) / dt;
    prev_error = error;
    return std::clamp(speed, -max_speed, max_speed);
}

void UGV::purePursuitControl(const geometry_msgs::PoseStamped& goal, double look_ahead_distance) {
    auto goal_ = goal.pose.position;
    std::pair<double, double> current_position(pose.pose.pose.position.x, pose.pose.pose.position.y);
    std::pair<double, double> look_ahead_point = calculateLookAheadPoint(current_position, {goal_.x, goal_.y}, look_ahead_distance);

    if (look_ahead_point.first == -1) {
        ROS_WARN("No lookahead point found!");
        return;
    }

    double alpha = atan2(look_ahead_point.second - pose.pose.pose.position.y, look_ahead_point.first - pose.pose.pose.position.x) - yaw;
    double curvature = 2 * L * sin(alpha) / look_ahead_distance;
    double speed = moveX(goal);

    geometry_msgs::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = curvature;
    // ROS_INFO("CMD linear x: %f", cmd.linear.x);
    // ROS_INFO("CMD angular z: %f", cmd.angular.z);
    
    cmd_vel_pub.publish(cmd);
}

std::pair<double, double> UGV::calculateLookAheadPoint(const std::pair<double, double>& current_position, const std::pair<double, double>& goal_position, double look_ahead_distance) {
    std::pair<double, double> path_vector(goal_position.first - current_position.first, goal_position.second - current_position.second);
    double path_length = sqrt(pow(path_vector.first, 2) + pow(path_vector.second, 2));
    if (path_length < look_ahead_distance) {
        return {-1, -1};
    }
    std::pair<double, double> look_ahead_vector((look_ahead_distance / path_length) * path_vector.first, (look_ahead_distance / path_length) * path_vector.second);
    return {current_position.first + look_ahead_vector.first, current_position.second + look_ahead_vector.second};
}

void UGV::stop() {
    cmd_vel_pub.publish(geometry_msgs::Twist());
    ros::Duration(1.0).sleep();
}

std::pair<double, double> UGV::rotateGoal(double goal_x, double goal_y) {
    double goalx = goal_x;
    double goaly = goal_y;
    // if (init_yaw > 2*M_PI) init_yaw -= 2.0 * M_PI;
    // else if (init_yaw < 0) init_yaw += 2.0 * M_PI;
    // ROS_INFO("yaw: %f", init_yaw * 180 / M_PI);
    double cos_yaw = cos(init_yaw);
    double sin_yaw = sin(init_yaw);
    double rotated_x = goalx * cos_yaw - goaly * sin_yaw + init_x;
    double rotated_y = goalx * sin_yaw + goaly * cos_yaw + init_y;
    return {rotated_x, rotated_y};
}
