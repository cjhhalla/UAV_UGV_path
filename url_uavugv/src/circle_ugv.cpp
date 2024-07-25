#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "ugv.h"

double dist(const geometry_msgs::PoseStamped& goal, const nav_msgs::Odometry& odom_pose) {
    auto now = odom_pose.pose.pose.position;
    auto goal_ = goal.pose.position;
    return sqrt(pow(now.x - goal_.x, 2) + pow(now.y - goal_.y, 2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh("~");
    UGV UGV(nh);
    geometry_msgs::PoseStamped goal;
    while (!UGV.initial_pose && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("INITIALIZING!");
    }
    goal.pose.position.x = UGV.init_x;
    goal.pose.position.y = UGV.init_y;
    double center_x = UGV.center_x;
    double center_y = UGV.center_y;
    double radius = UGV.radius;
    int num_waypoint = 36;
    std::vector<std::pair<double, double>> wp;
    for (int i=0; i<num_waypoint; i++){
        std::pair<double, double> wp_ = {center_x + radius * std::cos(2*M_PI*i/num_waypoint),
                                    center_y + radius * std::sin(2*M_PI*i/num_waypoint)};
        wp.push_back(wp_);
    }
    
    std::vector<std::pair<double, double>> waypoints;
    for (auto& point : wp) {
        waypoints.push_back(UGV.rotateGoal(point.first, point.second));
        ROS_INFO("WAYPOINT X: %f", UGV.rotateGoal(point.first, point.second).first);
        ROS_INFO("WAYPOINT Y: %f", UGV.rotateGoal(point.first, point.second).second);
    }

    int set_goal = 0;
    double look_ahead_distance = UGV.look_ahead;
    int laps_completed = 0;

    try {
        ros::Rate rate(25);
        while (ros::ok()) {
            if (dist(goal, UGV.pose) <= look_ahead_distance) {
                ROS_INFO("Current Goal X: %f", goal.pose.position.x);
                ROS_INFO("Current Goal Y: %f", goal.pose.position.y);
                goal.pose.position.x = waypoints[set_goal].first;
                goal.pose.position.y = waypoints[set_goal].second;
                set_goal = (set_goal + 1) % waypoints.size();
                ROS_INFO("NEXT WAYPOINT");
                if (set_goal == 0) {
                    laps_completed++;
                }
                if (laps_completed >= UGV.laps_completed) {
                    goal.pose.position.x = UGV.init_x;
                    goal.pose.position.y = UGV.init_y;
                    for (int i = 0; i < 150; ++i) {
                        UGV.purePursuitControl(goal, look_ahead_distance);
                        rate.sleep();
                    }
                    if (dist(goal, UGV.pose) <= look_ahead_distance) {
                        ROS_INFO("Completed laps. Stopping.");
                        return 0;
                        break;
                    }
                }
            }
            UGV.purePursuitControl(goal, look_ahead_distance);
            rate.sleep();
            ros::spinOnce();
        }
    } catch (ros::Exception& e) {
        return 0;
    }

    return 0;
}
