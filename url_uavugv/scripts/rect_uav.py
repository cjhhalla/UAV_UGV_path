#!/usr/bin/env python
import time
from math import sqrt, pow
import math
import rospy
import roslib
import mavros

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from mavros_msgs.srv import SetMode, CommandBool

import sys
import signal

class px4():
    def __init__(self):
        rospy.init_node('Pixhawk', anonymous=True)
        #param
        self.length = rospy.get_param('~length', 3)
        self.target_altitude = rospy.get_param('~target_altitude', 3)
        self.max_speed = rospy.get_param('~max_speed', 2)
        self.laps_completed = rospy.get_param('~laps_completed',1)
        self.use_gps = rospy.get_param('~use_gps',False)
        self.robot_id = rospy.get_param('~robot','')

        rospy.loginfo("robot_id: {}".format(self.robot_id))
        rospy.loginfo("length: {}".format(self.length))
        rospy.loginfo("target_altitude: {}".format(self.target_altitude))
        rospy.loginfo("max_speed: {}".format(self.max_speed))
        rospy.loginfo("laps_completed: {}".format(self.laps_completed))
        rospy.loginfo("use_gps: {}".format(self.use_gps))

        if not self.use_gps:
            self.local_pos_pub = rospy.Publisher(self.robot_id + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.position_sub = rospy.Subscriber(self.robot_id + '/mavros/local_position/odom', Odometry, self.position_callback)
        else:
            self.local_pos_pub = rospy.Publisher(self.robot_id + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.position_sub = rospy.Subscriber(self.robot_id + '/mavros/global_position/local', Odometry, self.position_callback)

        self.local_vel_pub = rospy.Publisher(self.robot_id + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber(self.robot_id + '/mavros/state', State, self.state_callback)
        self.arming = rospy.ServiceProxy(self.robot_id + '/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy(self.robot_id + '/mavros/set_mode', SetMode)
        self.landing = rospy.ServiceProxy(self.robot_id + '/mavros/set_mode', SetMode)
        self.current_state = State()
        self.pose = Odometry()
        self.vel = TwistStamped()
        self.vel_ = self.vel.twist.linear
        self.ang_ = self.vel.twist.angular
        self.rate = rospy.Rate(25)

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.pose = msg

    def set_goal(self, goal):
        goal_ = goal.pose.position
        self.local_pos_pub.publish(goal)

    def vel_goal(self, goal):
        goal_pos = goal.pose.position
        self.vel.twist.linear.x = self.clip_velocity(0.5 * (goal_pos.x - self.pose.pose.pose.position.x))
        self.vel.twist.linear.y = self.clip_velocity(0.5 * (goal_pos.y - self.pose.pose.pose.position.y))
        self.vel.twist.linear.z = self.clip_velocity(0.5 * (goal_pos.z - self.pose.pose.pose.position.z))
        self.vel.header.stamp = rospy.Time.now()
        self.local_vel_pub.publish(self.vel)

    def clip_velocity(self, velocity, max_speed= 1):
        return max(min(velocity, self.max_speed), -self.max_speed)

def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    goal_ = goal.pose.position
    return sqrt(pow(now.x-goal_.x,2) + pow(now.y-goal_.y,2) + pow(now.z-goal_.z,2))

def tic(): 
    global starttime
    starttime = time.time()

def toc():
    nowtime = time.time()
    return nowtime - starttime

px4 = px4()

goal = PoseStamped()
goal_ = goal.pose.position

goal_.x = 0
goal_.y = 0
goal_.z = px4.target_altitude

time.sleep(1)

off_check = 0
arm_check = 0

set_goal = 0

if __name__ == '__main__':
    vel_goal_flag = 0
    laps_completed = 0
    target_z = px4.target_altitude
    length = px4.length
    try:
        for i in range(100):
            px4.set_goal(goal)

        while not px4.current_state.connected: 
            px4.rate.sleep()

        tic()
        start_time = time.time()
        while not rospy.is_shutdown():
            if px4.current_state.mode != "OFFBOARD" and (toc() > 2):
                px4.offboarding(base_mode=0, custom_mode="OFFBOARD")
                off_check = 0
                tic()
            elif not px4.current_state.armed and (toc() > 2):
                px4.arming(True)
                arm_check = 0
                tic()

            if px4.current_state.mode == "OFFBOARD" and off_check == 0:
                rospy.loginfo("offboard enabled : %r" % px4.current_state.mode)
                off_check = 1
            elif px4.current_state.armed and arm_check == 0:
                rospy.loginfo("Vehicle armed : %s" % px4.current_state.armed)
                arm_check = 1

            if dist(goal, px4.pose) < 0.3:
                if set_goal == 0:
                    goal_.x = length
                    goal_.y = 0
                    goal_.z = target_z
                    set_goal = 1
                elif set_goal == 1:
                    goal_.x = length
                    goal_.y = length
                    goal_.z = target_z
                    set_goal = 2
                elif set_goal == 2:
                    goal_.x = 0
                    goal_.y = length
                    goal_.z = target_z
                    set_goal = 3
                elif set_goal == 3:
                    goal_.x = 0
                    goal_.y = 0
                    goal_.z = target_z
                    set_goal = 0
                    laps_completed += 1

            if laps_completed >= px4.laps_completed:
                goal_.x = 0
                goal_.y = 0
                goal_.z = target_z
                if dist(goal, px4.pose) < 0.3:
                    px4.landing(base_mode=0, custom_mode="AUTO.LAND")
                    rospy.loginfo("Landing!")
                    break

            if vel_goal_flag == 0:
                px4.vel_goal(goal) 
            px4.rate.sleep()

    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
        sys.exit(0)

