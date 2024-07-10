#!/usr/bin/env python

import time
from math import sqrt, pow
import math
import rospy
import roslib
import mavros

from geometry_msgs.msg  import PoseStamped, TwistStamped
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
        	self.state_sub = rospy.Subscriber('/mavros/state',State, self.state_callback)
        	self.position_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_callback)
        	self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        	self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

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
		goal_ = goal.pose.position
		self.vel_.x = 0.5*(goal_.x - self.pose.pose.pose.position.x)
		self.vel_.y = 0.5*(goal_.y - self.pose.pose.pose.position.y)
		self.vel_.z = 0.5*(goal_.z - self.pose.pose.pose.position.z)

		if self.vel_.x >  2:
			self.vel_.x =  1.5
		if self.vel_.x < -2:
			self.vel_.x = -1.5
		if self.vel_.y >  2:
			self.vel_.y =  1.5
		if self.vel_.y < -2:
			self.vel_.y = -1.5
		if self.vel_.z >  2:
			self.vel_.z =  1.5
		if self.vel_.z < -2:
			self.vel_.z = -1.5
		self.vel.header.stamp = rospy.Time.now()
		self.local_vel_pub.publish(self.vel)

	def circle_goal(self, radius, angular,start_time):
		dt = time.time() - start_time
		self.vel_.x = -radius * angular * math.sin(angular * dt)
		self.vel_.y =  radius * angular * math.cos(angular * dt)
		self.vel_.z = 0
		#ang_.x = 0
		#ang_.y = 0
		#ang_.z = angular
		self.vel.header.stamp = rospy.Time.now()
		self.local_vel_pub.publish(self.vel)
		px4.rate.sleep()


def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    goal_ = goal.pose.position
    return sqrt(pow(now.x-goal_.x,2) + pow(now.y-goal_.y,2) + pow(now.z-goal_.z,2))


def tic(): 
    global starttime
    starttime=time.time()

def toc():
    nowtime=time.time()
    #print("toc: %f"%(nowtime-starttime))
    return nowtime-starttime

px4 = px4()

goal = PoseStamped()
goal_ = goal.pose.position

goal_.x=0
goal_.y=0
goal_.z=4

time.sleep(1)

off_check=0
arm_check=0

set_goal=0

if __name__ == '__main__':
 radius = 3
 angular = 1
 vel_goal_flag = 0
 while 1:
    try:
        # send a few setpoints before starting
        for i in range(0,100):
            px4.set_goal(goal)

        while not px4.current_state.connected: # wait for FCU connection
            px4.rate.sleep()
         
        tic()
	start_time = time.time()
        while not rospy.is_shutdown():
            if px4.current_state.mode != "OFFBOARD" and (toc() > 2) :
                px4.offboarding(base_mode=0, custom_mode="OFFBOARD")
                off_check=0
                tic()
            elif not px4.current_state.armed and (toc() > 2) :
                px4.arming(True)
                arm_check=0
                tic()
            
            if px4.current_state.mode == "OFFBOARD" and off_check==0:
                rospy.loginfo("offboard enabled : %r" %px4.current_state.mode)
                off_check=1
            elif px4.current_state.armed and arm_check==0:
                rospy.loginfo("Vehicle armed : %s" %px4.current_state.armed)
                arm_check=1

            if dist(goal, px4.pose) < 0.3:
		vel_goal_flag = 1                
		if set_goal == 0:
                    goal_.x=4
                    goal_.y=0
                    goal_.z=4
                    set_goal = 1
                elif set_goal == 1:
                    goal_.x=4
                    goal_.y=4
                    goal_.z=4
                    set_goal = 2
                elif set_goal == 2:
                    goal_.x=0
                    goal_.y=4
                    goal_.z=4
                    set_goal = 3
                elif set_goal == 3:
                    goal_.x=0
                    goal_.y=0
                    goal_.z=4
                    set_goal = 0

            
	    #px4.set_goal(goal) #by position command
            if vel_goal_flag == 0:
		px4.vel_goal(goal) #by velocity command
            if vel_goal_flag == 1:
		px4.circle_goal(radius, angular, start_time)
            px4.rate.sleep()

    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
        sys.exit(0)


