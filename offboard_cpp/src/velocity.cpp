#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandTOL.h>
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_feedback");
  ros::NodeHandle nh;

  double target_z;
  nh.param("target_z",target_z,2.0);

  ros::Subscriber cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose",10,cur_pose_cb);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
    ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ROS_INFO("Initializing...");
  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  double last_angle = 0;
  int count =0;
  double current_angle = 0;

  ROS_INFO("Connected.");

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = target_z;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  
  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ros::Time time_start = ros::Time::now();  
  while(ros::ok()){
    if( current_state.mode != "OFFBOARD" &&
	(ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) &&
	  offb_set_mode.response.mode_sent){
	ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
	  (ros::Time::now() - last_request > ros::Duration(5.0))){
	if( arming_client.call(arm_cmd) &&
	    arm_cmd.response.success){
	  ROS_INFO("Vehicle armed");
	}
	last_request = ros::Time::now();
      }
    }
    if(std::abs(current_pose.pose.position.z-target_z)>0.4)
    {
        local_pos_pub.publish(pose);
    }
    else{
    pose.pose.position.x = 2*sin(2.0*M_PI*0.05*(ros::Time::now()-time_start).toSec());
    pose.pose.position.y = 2*cos(2.0*M_PI*0.05*(ros::Time::now()-time_start).toSec());
    vel.linear.x = 2*2.0*M_PI*0.05*cos(2.0*M_PI*0.05*(ros::Time::now()-time_start).toSec());
    vel.linear.y = -2.0*2*M_PI*0.05*sin(2.0*M_PI*0.05*(ros::Time::now()-time_start).toSec());
    local_pos_pub.publish(pose);
    local_vel_pub.publish(vel);
    }

    current_angle = std::atan2(pose.pose.position.y,pose.pose.position.x);
    if (current_angle){
        if(std::abs(last_angle-current_angle)>M_PI){
            count++;
            ROS_INFO("Completed laps: %d", count);
        }
    }
    last_angle = current_angle;

    if (count>=2){
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        for(int i = 200; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        mavros_msgs::CommandTOL land_cmd;
        land_cmd.request.altitude = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.yaw = 0;
        if (land_client.call(land_cmd) && land_cmd.response.success) {
            ROS_INFO("Landing initiated");
            break; 
        }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}