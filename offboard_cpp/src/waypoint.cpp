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

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    get_waypoint();

    ros::Rate rate(20.0);
    ROS_INFO("Initializing...");
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!!!");
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
    ros::Time last_request = ros::Time::now();
    while(ros::ok() && !current_state.armed)
    {
    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("OFFBOARD enabled");
        }
        last_request = ros::Time::now();
    }
    else
    {
        if(!current_state.armed && (ros::Time::now()-last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
    for(int i = 200; ros::ok() && i > 0; --i){
        local_st_pub.publish(pose_init);
        ros::spinOnce();
        rate.sleep();
    }
    ros::spinOnce();
    rate.sleep();
    }
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
    if(init_local_pose_check)
    {
        for(int i=0; i<num_waypoint; i++)
        {
            geometry_msgs::PoseStamped temp_target_pose;
            temp_target_pose.pose.position.x = msg->pose.position.x+x_pos[i];
            temp_target_pose.pose.position.y = msg->pose.position.y+y_pos[i];
            temp_target_pose.pose.position.z = msg->pose.position.z+z_pos[i];
            waypoint_pose.push_back(temp_target_pose);
        }
        init_local_pose_check = false;
    }
    publish_waypoint();
    ros::Rate rate(20.0);
    rate.sleep();
}

void ApdControl::get_waypoint()
{
    if (ros::param::get("offboard_example_node/num_waypoint", num_waypoint)) {}
    else 
    {
        ROS_WARN("Didn't find num_waypoint");
    }
    if (ros::param::get("offboard_example_node/x_pos", x_pos)) {}
    else 
    {
        ROS_WARN("Didn't find x_pos");
    }
    if (ros::param::get("offboard_example_node/y_pos", y_pos)) {}
    else 
    {
        ROS_WARN("Didn't find y_pos");
    }
    if (ros::param::get("offboard_example_node/z_pos", z_pos)) {}
    else 
    {
        ROS_WARN("Didn't find z_pos");
    }
    if (x_pos.size() != num_waypoint) {
        ROS_WARN("Wrong x_pos values.");
    }
    if (y_pos.size() != num_waypoint) {
        ROS_WARN("Wrong y_pos values.");
    }
    if (z_pos.size() != num_waypoint) {
        ROS_WARN("Wrong z_pos values.");
    }
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


