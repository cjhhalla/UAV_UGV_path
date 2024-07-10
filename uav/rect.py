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
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) # topic msg Publish
        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.position_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.landing = rospy.ServiceProxy('/mavros/set_mode', SetMode)
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

    def clip_velocity(self, velocity, max_speed= 0.5):
        return max(min(velocity, max_speed), -max_speed)

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
goal_.z = 3

time.sleep(1)

off_check = 0
arm_check = 0

set_goal = 0

if __name__ == '__main__':
    vel_goal_flag = 0
    laps_completed = 0
    target_z = 4
    length = 4
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

            if laps_completed >= 2:
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
