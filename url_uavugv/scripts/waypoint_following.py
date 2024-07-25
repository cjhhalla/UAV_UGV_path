#!/usr/bin/env python
import time
from math import sqrt, pow, atan2
import math
import rospy
import roslib
import mavros
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget

import sys
import signal

import tf.transformations as transformations

class px4():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        #param
        self.length = rospy.get_param('~length', 3)
        self.target_altitude = rospy.get_param('~target_altitude', 3)
        self.max_speed = rospy.get_param('~max_speed', 2)
        self.laps_completed = rospy.get_param('~laps_completed',1)
        self.use_gps = rospy.get_param('~use_gps',False)
        self.robot_id = rospy.get_param('~robot','')
        self.num_waypoint = rospy.get_param('num_waypoint',None)
        self.waypoint_x = rospy.get_param('x_pos',None)
        self.waypoint_y = rospy.get_param('y_pos',None)
        self.waypoint_z = rospy.get_param('z_pos',None)
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
        self.flag_sub = rospy.Subscriber(self.robot_id + '/is_safe', Bool, self.flag_callback)
        self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.current_state = State()
        self.pose = Odometry()
        self.vel = TwistStamped()
        self.vel_ = self.vel.twist.linear
        self.ang_ = self.vel.twist.angular
        self.rate = rospy.Rate(25)
        self.yaw = 0
        self.initial_yaw = False
        self.yaw_list = []
        self.init_yaw = 0
        self.fail_safe = True

        if not self.check_params():
            rospy.signal_shutdown("Parameter loading failed")
            return

    def check_params(self):
        if self.num_waypoint is None:
            rospy.logerr("Parameter 'num_waypoint' not set")
            return False
        if self.waypoint_x is None:
            rospy.logerr("Parameter 'waypoint x' not set")
            return False
        if self.waypoint_y is None:
            rospy.logerr("Parameter 'waypoint y' not set")
            return False
        if self.waypoint_z is None:
            rospy.logerr("Parameter 'waypoint z' not set")
            return False
        if not len(self.waypoint_x) == self.num_waypoint:
            rospy.logerr("Not match number of 'x_pos' with 'num_waypoint")
        if not len(self.waypoint_y) == self.num_waypoint:
            rospy.logerr("Not match number of 'y_pos' with 'num_waypoint")
        if not len(self.waypoint_z) == self.num_waypoint:
            rospy.logerr("Not match number of 'z_pos' with 'num_waypoint")
        return True

    def flag_callback(self, msg):
        if msg.data == True:
            self.fail_safe = True
        elif msg.data == False:
            self.fail_safe = False

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.pose = msg
        orientation_q = self.pose.pose.pose.orientation
        euler = transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.yaw = euler[2]
        if not self.initial_yaw:
            self.yaw_list.append(self.yaw)
            if len(self.yaw_list) > 100 :
                self.init_yaw = np.mean(self.yaw_list)
                rospy.loginfo("initial yaw: {}".format(self.init_yaw))
                self.initial_yaw = True

    def set_goal(self, goal):
        goal_ = goal.pose.position
        self.local_pos_pub.publish(goal)

    def vel_goal(self, goal, set_goal):
        goal_pos = goal.pose.position
        self.vel.twist.linear.x = self.clip_velocity(0.5 * (goal_pos.x - self.pose.pose.pose.position.x))
        self.vel.twist.linear.y = self.clip_velocity(0.5 * (goal_pos.y - self.pose.pose.pose.position.y))
        self.vel.twist.linear.z = self.clip_velocity(0.5 * (goal_pos.z - self.pose.pose.pose.position.z))
        self.vel.header.stamp = rospy.Time.now()
        delta_x = goal_pos.x - self.pose.pose.pose.position.x
        delta_y = goal_pos.y - self.pose.pose.pose.position.y

        if not set_goal == 0: 
            target_yaw = atan2(delta_y,delta_x)
            if (target_yaw - self.yaw) < -math.pi:
                target_yaw = target_yaw + math.pi * 2
            if (target_yaw - self.yaw) > math.pi:
                target_yaw = target_yaw - math.pi * 2
            self.vel.twist.angular.z = (0.5* (target_yaw - self.yaw))

        self.local_vel_pub.publish(self.vel)

    def clip_velocity(self, velocity, max_speed= 1):
        return max(min(velocity, self.max_speed), -self.max_speed)

    def rotate_goal(self, goal_x, goal_y, goal_z):
        cos_yaw = np.cos(self.init_yaw)
        sin_yaw = np.sin(self.init_yaw)
        rotated_x = goal_x * cos_yaw - goal_y * sin_yaw
        rotated_y = goal_x * sin_yaw + goal_y * cos_yaw
        rotated_z = goal_z
        return rotated_x, rotated_y, rotated_z 

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

time.sleep(3)

off_check = 0
arm_check = 0

set_goal = 0
wp = []
for i in range(px4.num_waypoint):
    wp_ = (px4.waypoint_x[i], px4.waypoint_y[i], px4.waypoint_z[i])
    wp.append(wp_)
wp = [px4.rotate_goal(x,y,z) for x,y,z in wp]
rospy.loginfo("way point: {}".format(wp))

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

################## FAIL SAFE #############################
            if not px4.fail_safe:
                goal_.x = px4.pose.pose.pose.position.x
                goal_.y = px4.pose.pose.pose.position.y
                goal_.z = target_z
                px4.vel_goal(goal,set_goal)
                continue
################## FAIL SAFE #############################

            if dist(goal, px4.pose) < 0.3:
                goal_.x, goal_.y, goal_.z = wp[set_goal]
                set_goal +=1
                
                if set_goal >= px4.num_waypoint:
                    set_goal -=1
                    goal_.x = 0
                    goal_.y = 0
                    goal_.z = target_z
                    if dist(goal, px4.pose) < 0.5:
                        px4.landing(base_mode=0, custom_mode="AUTO.LAND")
                        rospy.loginfo("Landing!")
                        break
            px4.vel_goal(goal,set_goal) 
            px4.rate.sleep()

    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
        sys.exit(0)

