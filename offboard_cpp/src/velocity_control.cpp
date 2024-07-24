#include "apd.h"

ApdControl::ApdControl():
verbal_flag(false),init_local_pose_check(true),waypoint_count(0),priv_nh("~")
{
    priv_nh.getParam("verbal_flag",verbal_flag);

    state_sub = nh.subscribe<mavros_msgs::State>
                ("/mavros/state",10,&ApdControl::state_cb,this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose",10,&ApdControl::cur_pose_cb,this);
    init_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose",10,&ApdControl::init_pose_cb,this);
    
    local_st_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::Twist>
                    ("mavros/setpoint_velocity/cmd_vel_unstamped",10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    ros::Rate rate(20.0);
    ROS_INFO("Initializing...");
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected");

    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    arm_cmd.request.value = true;
    
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    arming();
}

void ApdControl::arming()
{
    geometry_msgs::PoseStamped pose_init;
    pose_init.pose.position.x = 0.0;
    pose_init.pose.position.y = 0.0;
    pose_init.pose.position.z = 1.5;
    ros::Rate rate(20.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        local_st_pub.publish(pose_init);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request_local = ros::Time::now();

    ros::Time time_start = ros::Time::now();

    while(ros::ok() && !current_state.armed)
    {
    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_local > ros::Duration(5.0)))
    {
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("OFFBOARD enabled");
        }
        last_request_local = ros::Time::now();
    }
    else
    {
        if(!current_state.armed && (ros::Time::now()-last_request_local > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request_local = ros::Time::now();
        }
    
    }
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Arming and while loop end... Before");
    }
    ROS_INFO("Arming and while loop end... After");
}

void ApdControl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void ApdControl::cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void ApdControl::init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

}

void ApdControl::get_waypoint()
{

}

void ApdControl::publish_waypoint()
{
    ros::Rate rate(20.0);
    if(!init_local_pose_check)
    {
        local_st_pub.publish(waypoint_pose[waypoint_count]);
        if(verbal_flag)
        {
            double dist = std::sqrt(
                                (current_pose.pose.position.x-waypoint_pose[waypoint_count].pose.position.x)* 
                (current_pose.pose.position.x-waypoint_pose[waypoint_count].pose.position.x) + 
                (current_pose.pose.position.y-waypoint_pose[waypoint_count].pose.position.y)* 
                (current_pose.pose.position.y-waypoint_pose[waypoint_count].pose.position.y) + 
                (current_pose.pose.position.z-waypoint_pose[waypoint_count].pose.position.z)* 
                (current_pose.pose.position.z-waypoint_pose[waypoint_count].pose.position.z)
                );
            ROS_INFO("distance: %.2f",dist); 
        }
        if (std::abs(current_pose.pose.position.x-waypoint_pose[waypoint_count].pose.position.x) < 0.5 &&
            std::abs(current_pose.pose.position.y-waypoint_pose[waypoint_count].pose.position.y) < 0.5 &&
            std::abs(current_pose.pose.position.z-waypoint_pose[waypoint_count].pose.position.z) < 0.5)
            {
                waypoint_count +=1;
                if(waypoint_count >=num_waypoint)
                {   
                    waypoint_count = waypoint_count-1;
                    ROS_INFO("land enabled");
                    while (!(land_client.call(land_cmd)&& land_cmd.response.success))
                    {
                        ROS_INFO("FINISH & LAND");
                        ros::spinOnce();
                        rate.sleep();
                    }
                    ROS_INFO("shutting down ROS");
                    ros::shutdown();
                }
            }
    }
}
