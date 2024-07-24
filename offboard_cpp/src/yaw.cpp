#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
double yaw_,roll_,pitch_;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
  tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);
  // ROS_INFO("Yaw: %f", yaw_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node_feedback");
  ros::NodeHandle nh;

  double target_z;
  nh.param("target_z",target_z,2.0);

  ros::Subscriber cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose",10,cur_pose_cb);
  ros::Publisher local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
    ("mavros/setpoint_raw/local", 10);
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
  double radius = 1.5; // 원의 반지름
  double angular_velocity = 0.2; // 각속도 (radians per second)
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
  mavros_msgs::PositionTarget target;
  target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
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
      if (current_pose.pose.position.z < target_z) {
          pose.pose.position.z += 0.1; // 점진적 증가
      } else {
          pose.pose.position.z -= 0.1; // 점진적 감소
      }
      local_pos_pub.publish(pose);
    }
    else{
    double time = ros::Time::now().toSec();
    double x = radius * cos(angular_velocity * time);
    double y = radius * sin(angular_velocity * time);
    double yaw = atan2(y, x) + M_PI; // 원의 중심을 바라보게 조정

        // 현재 위치 및 yaw 각도 설정
    target.position.x = x;
    target.position.y = y;
    target.position.z = target_z; // 고정 높이
    if (yaw_-yaw <0.01){
      target.yaw = yaw;
    } 

    // 데이터 퍼블리시
    local_raw_pub.publish(target);
    }
    current_angle = std::atan2(target.position.y,target.position.x);
    if (current_angle){
        if(std::abs(last_angle-current_angle)>M_PI){
            count++;
            ROS_INFO("Completed laps: %d", count);
        }
    }
    last_angle = current_angle;
    if (count>=5){
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = target_z;
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