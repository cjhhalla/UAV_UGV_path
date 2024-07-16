#!/usr/bin/env python
import time
from math import sqrt, pow
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget

import sys
import signal
import tf

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    return sqrt(pow(now.x - goal.x, 2) + pow(now.y - goal.y, 2) + pow(now.z - goal.z, 2))

class robot:
    def __init__(self, FLIGHT_ALTITUDE, RATE, RADIUS, CYCLE_S, REF_FRAME, USE_GPS):
        rospy.init_node('robot_controller', anonymous=True)
        
        self.alt_param = rospy.get_param('~target_altitude', 3.0)
        self.rate_param = rospy.get_param('~rate', 50)
        self.radius_param = rospy.get_param('~radius', 10)
        self.cycle_param = rospy.get_param('~cycle', 100)
        self.use_gps = rospy.get_param('~use_gps', False)
        self.robot_id = rospy.get_param('~robot', '')
        self.yaw_control = rospy.get_param('~yaw_control', True)
        self.laps_completed = rospy.get_param('~laps_completed',1)


        rospy.loginfo("robot_id: {}".format(self.robot_id))
        rospy.loginfo("radius: {}".format(self.radius_param))
        rospy.loginfo("target_altitude: {}".format(self.alt_param))
        rospy.loginfo("rate: {}".format(self.rate_param))
        rospy.loginfo("cycle: {}".format(self.cycle_param))
        rospy.loginfo("use_gps: {}".format(self.use_gps))
        rospy.loginfo("yaw_control: {}".format(self.yaw_control))
        rospy.loginfo("laps_completed: {}".format(self.laps_completed))

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

        self.FLIGHT_ALTITUDE = self.alt_param
        self.RATE = self.rate_param
        self.RADIUS = self.radius_param
        self.CYCLE_S = self.cycle_param
        self.STEPS = int(self.CYCLE_S * self.RATE)
        self.FRAME = REF_FRAME
        self.current_state = State()
        self.pose = Odometry()
        self.vel = TwistStamped()
        self.vel_ = self.vel.twist.linear
        self.rate = rospy.Rate(25)

        self.target_z = 4
        self.off_check = 0
        self.arm_check = 0
        self.init_yaw = 0
        self.initial_yaw = False
        self.yaw_list = []

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.pose = msg
        self.pose_ = self.pose.pose.pose.position
        orientation_q = self.pose.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.yaw = euler[2]
        if not self.initial_yaw:
            self.yaw_list.append(self.yaw)
            if len(self.yaw_list) > 100:
                self.init_yaw = np.mean(self.yaw_list)
                rospy.loginfo("initial yaw: {}".format(self.init_yaw))
                self.initial_yaw = True

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

    def rotate_goal(self, x, y, yaw):
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        x_new = cos_yaw * x - sin_yaw * y
        y_new = sin_yaw * x + cos_yaw * y
        return x_new, y_new

    def main(self):
        i = 0
        dt = 1.0 / self.RATE
        dadt = math.pi * 2 / self.CYCLE_S
        r = self.RADIUS
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
            sspo = (s * s) + 1.0
            ssmo = (s * s) - 1.0
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
            if laps_completed >= self.laps_completed:
                goal = PoseStamped()
                goal_ = goal.pose.position
                goal_.x = 0
                goal_.y = 0
                goal_.z = self.FLIGHT_ALTITUDE
                for _ in range(100):
                    self.vel_goal(goal)
                self.landing(base_mode=0, custom_mode="AUTO.LAND")
                rospy.loginfo("Landing!")
                break

            rotated_x, rotated_y = self.rotate_goal(posx[k], posy[k], self.init_yaw)
            target.header.frame_id = self.FRAME
            target.coordinate_frame = 1
            target.type_mask = 0

            target.position.x = rotated_x
            target.position.y = rotated_y
            target.position.z = posz[k]
			
            target.velocity.x = velx[k]
            target.velocity.y = vely[k]
            target.velocity.z = velz[k]
			
            target.acceleration_or_force.x = afx[k]
            target.acceleration_or_force.y = afy[k]
            target.acceleration_or_force.z = afz[k]
			
            if self.yaw_control:
                target.yaw = yawc[k]
                target.yaw_rate = yaw_ratec[k]
            else:
                target.yaw = 0
                target.yaw_rate = 0
            
            self.publisher.publish(target)
			
            k = k + 1
            rr.sleep()

if __name__ == '__main__':
    try:
        q = robot(FLIGHT_ALTITUDE=0, RATE=0, RADIUS=0, CYCLE_S=0, REF_FRAME='map', USE_GPS=True)
        q.main()
    except rospy.ROSInterruptException:
        pass
