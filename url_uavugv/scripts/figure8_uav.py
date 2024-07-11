#!/usr/bin/env python
import time
from math import sqrt, pow
import math
import rospy
import roslib
import mavros
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    return sqrt(pow(now.x-goal, 2) + pow(now.y-goal, 2) + pow(now.z-goal, 2))

class robot:
    def __init__(self, FLIGHT_ALTITUDE, RATE, RADIUS, CYCLE_S, REF_FRAME, USE_GPS):
        rospy.init_node('robot_controller', anonymous=True)
        
        self.alt_param = rospy.get_param('~target_altitude',3.0)
        self.rate_param = rospy.get_param('~rate',50)
        self.radius_param = rospy.get_param('~radius',10)
        self.cycle_param = rospy.get_param('~cycle',100)
        self.use_gps = rospy.get_param('~use_gps',False)
        self.robot_id = rospy.get_param('~robot','')

        rospy.loginfo("robot_id: {}".format(self.robot_id))
        rospy.loginfo("radius: {}".format(self.radius_param))
        rospy.loginfo("target_altitude: {}".format(self.target_altitude))
        rospy.loginfo("rate: {}".format(self.rate_param))
        rospy.loginfo("cycle: {}".format(self.cycle_param))
        rospy.loginfo("use_gps: {}".format(self.use_gps))

        if not self.use_gps:
            self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.position_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_callback)
        else:
            self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.position_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)

        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback) 
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.landing = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


        self.FLIGHT_ALTITUDE = self.alt_param           # Flight altitude
        self.RATE = self.rate_param                                # Loop rate hz
        self.RADIUS = self.radius_param                            # Radius of figure 8 in meters
        self.CYCLE_S = self.cycle_param                          # Time to complete one figure 8 cycle in seconds
        self.STEPS = int(self.CYCLE_S * self.RATE)
        self.FRAME = REF_FRAME                          # Reference frame
        self.current_state = State()
        self.pose = Odometry()
        self.vel = TwistStamped()
        self.vel_ = self.vel.twist.linear
        self.rate = rospy.Rate(25)

        self.target_z = 4
        self.off_check = 0
        self.arm_check = 0

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.pose = msg
        self.pose_ = self.pose.pose.pose.position

    def vel_goal(self, goal):
        goal_ = goal.pose.position

        self.vel_.x = 0.5 * (goal_.x - self.pose_.x)
        self.vel_.y = 0.5 * (goal_.y - self.pose_.y)
        self.vel_.z = 0.5 * (goal_.z - self.pose_.z)

        self.vel.header.stamp = rospy.Time.now()
        self.local_vel_pub.publish(self.vel)

    def set_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self.local_pos_pub.publish(goal)

    def takeoff_wait(self):
        goal = PoseStamped()
        goal.pose.position.x = 0
        goal.pose.position.y = 0
        goal.pose.position.z = 4

        while not self.current_state.connected:
            self.rate.sleep()

        start_time = time.time()
        while self.arm_check == 0:
            if self.current_state.mode != "OFFBOARD" and time.time() - start_time > 2:
                self.offboarding(base_mode=0, custom_mode="OFFBOARD")
                self.off_check = 0
                start_time = time.time()
            elif not self.current_state.armed and time.time() - start_time > 2:
                self.arming(True)
                self.arm_check = 0
                start_time = time.time()

            if self.current_state.mode == "OFFBOARD" and self.off_check == 0:
                rospy.loginfo("offboard enabled : %r" % self.current_state.armed)
                self.off_check = 1
            elif self.current_state.armed and self.arm_check == 0:
                rospy.loginfo("Vehicle armed : %s" % self.current_state.armed)
                self.arm_check = 1

            self.vel_goal(goal)
            self.rate.sleep()
        for i in range(250):
            self.vel_goal(goal)
            self.rate.sleep()

    def main(self):
        i = 0                        # Set the counter
        dt = 1.0 / self.RATE         # Set the sample time step
        dadt = math.pi * 2 / self.CYCLE_S # First derivative of angle with respect to time
        r = self.RADIUS              # Set the radius of the figure-8
        path = []	

        rospy.sleep(2)
        self.takeoff_wait()
        rospy.loginfo('start figure8')

        PI = math.pi
        start_stamp = rospy.get_rostime()
        t = np.arange(0, self.STEPS, 1)
	
        posx = [1] * len(t)
        posy = [1] * len(t)
        posz = [1] * len(t)
        velx = [1] * len(t)
        vely = [1] * len(t)
        velz = [1] * len(t)
        afx = [1] * len(t)
        afy = [1] * len(t)
        afz = [1] * len(t)
        yawc = [1] * len(t)
        pitchc = [1] * len(t)
        rollc = [1] * len(t)
        yaw_ratec = [1] * len(t)

        for i in range(0, self.STEPS):
            a = (-math.pi / 2) + i * (math.pi * 2 / self.STEPS)
            c = math.cos(a)
            c2a = math.cos(2.0 * a)
            c4a = math.cos(4.0 * a)
            c2am3 = c2a - 3.0
            c2am3_cubed = c2am3 * c2am3 * c2am3
            s = math.sin(a)
            cc = c * c
            ss = s * s
            sspo = (s * s) + 1.0 # sin squared plus one
            ssmo = (s * s) - 1.0 # sin squared minus one
            sspos = sspo * sspo

            posx[i] = -(r * c * s) / sspo
            posy[i] = (r * c) / sspo
            posz[i] = self.FLIGHT_ALTITUDE

            velx[i] = dadt * r * (ss * ss + ss + (ssmo * cc)) / sspos
            vely[i] = -dadt * r * s * (ss + 2.0 * cc + 1.0) / sspos
            velz[i] = 0.0

            afx[i] = -dadt * dadt * 8.0 * r * s * c * ((3.0 * c2a) + 7.0) / c2am3_cubed
            afy[i] = dadt * dadt * r * c * ((44.0 * c2a) + c4a - 21.0) / c2am3_cubed
            afz[i] = 0.0

            yawc[i] = math.atan2(vely[i], velx[i])
            pitchc[i] = math.asin(afx[i] / 9.81)
            rollc[i] = math.atan2(afy[i], afz[i])
	
        for i in range(0, self.STEPS):
            next_yaw = yawc[(i + 1) % self.STEPS] 
            curr_yaw = yawc[i]
            if (next_yaw - curr_yaw) < -math.pi:
                next_yaw = next_yaw + math.pi * 2
            if (next_yaw - curr_yaw) > math.pi:
                next_yaw = next_yaw - math.pi * 2
            yaw_ratec[i] = (next_yaw - curr_yaw) / dt

        target = PositionTarget()
        rr = rospy.Rate(self.RATE)
        k = 0
        laps_completed = 0

        while not rospy.is_shutdown():
            if k >= len(posx):
                k = 0
                laps_completed += 1
            if laps_completed >= 2:
                goal = PoseStamped()
                goal_ = goal.pose.position
                goal_.x = 0
                goal_.y = 0
                goal_.z = self.FLIGHT_ALTITUDE
                for _ in range (100):
                   self.vel_goal(goal)									
                self.landing(base_mode=0, custom_mode="AUTO.LAND")
                rospy.loginfo("Landing!")
                break
            target.header.frame_id = self.FRAME  # Define the frame that will be used
            target.coordinate_frame = 1 # MAV_FRAME_LOCAL_NED = 1
            target.type_mask = 0  # Use everything!
            # PositionTarget::IGNORE_VX +
            # PositionTarget::IGNORE_VY +
            # PositionTarget::IGNORE_VZ +
            # PositionTarget::IGNORE_AFX +
            # PositionTarget::IGNORE_AFY +
            # PositionTarget::IGNORE_AFZ +
            # PositionTarget::IGNORE_YAW;

            # Gather position for publishing
            target.position.x = posx[k]
            target.position.y = posy[k]
            target.position.z = posz[k]
			
            # Gather velocity for publishing
            target.velocity.x = velx[k]
            target.velocity.y = vely[k]
            target.velocity.z = velz[k]
			
            # Gather acceleration for publishing
            target.acceleration_or_force.x = afx[k]
            target.acceleration_or_force.y = afy[k]
            target.acceleration_or_force.z = afz[k]
			
            # Gather yaw for publishing
            target.yaw = yawc[k]
			
            # Gather yaw rate for publishing
            target.yaw_rate = yaw_ratec[k]
			
            # Publish to the setpoint topic
            self.publisher.publish(target)
			
            k = k + 1
            rr.sleep()


if __name__ == '__main__':
    try:
        q = robot(FLIGHT_ALTITUDE=0, RATE=0, RADIUS=0, CYCLE_S=0, REF_FRAME='map', USE_GPS=True)
        q.main()
    except rospy.ROSInterruptException:
        pass

