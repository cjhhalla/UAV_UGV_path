#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos, pi
import sys
import signal
import numpy as np
import tf.transformations as transformations
from gazebo_msgs.msg import ModelStates

def signal_handler(signal, frame): 
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    goal_ = goal.pose.position
    return sqrt((now.x - goal_.x) ** 2 + (now.y - goal_.y) ** 2)

class Jackal:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.radius = rospy.get_param('~radius', 3)
        self.max_speed = rospy.get_param('~max_speed', 2)
        self.laps_completed = rospy.get_param('~laps_completed',1)
        self.use_gps = rospy.get_param('~use_gps',False)
        self.robot_id = rospy.get_param('~robot','')
        self.look_ahead = rospy.get_param('~look_ahead',1.2)
        self.center_x1 = rospy.get_param('~center_x1',0)
        self.center_x2 = rospy.get_param('~center_x2',0)

        rospy.loginfo("radius: {}".format(self.radius))
        rospy.loginfo("max_speed: {}".format(self.max_speed))
        rospy.loginfo("laps_completed: {}".format(self.laps_completed))
        rospy.loginfo("use_gps: {}".format(self.use_gps))
        rospy.loginfo("look_ahead_distance: {}".format(self.look_ahead))
        rospy.loginfo("robot id: {}".format(self.robot_id))
        rospy.loginfo("center_x1: {}".format(self.center_x1))
        rospy.loginfo("center_x2: {}".format(self.center_x2))

        self.cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.pose_callback)

        # if not self.use_gps:
        #     self.pose_sub = rospy.Subscriber(self.robot_id + '/mavros/local_position/odom', Odometry, self.pose_callback)
        # else:
        #     self.position_sub = rospy.Subscriber(self.robot_id + '/mavros/global_position/local', Odometry, self.pose_callback)

        self.pose = Odometry()
        self.rate = rospy.Rate(100)

        self.kp = 1
        self.kd = 0.1

        self.prev_error = 0
        self.yaw = 0
        self.initial_yaw = False
        self.yaw_list = []
        self.init_yaw = 0
        self.model_name = 'jackal'

    # def pose_callback(self, msg):
    #     index = msg.name.index(self.model_name)
    #     pose_ = msg.pose[index]
    #     self.pose.pose.pose.position = pose_.position
    #     orientation_q = pose_.orientation
    #     euler = transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    #     self.yaw = euler[2]

    def pose_callback(self, msg):
        #index = msg.name.index(self.model_name)
        #pose_ = msg.pose[index]
        self.pose   = msg
        orientation_q = self.pose.pose.pose.orientation
        euler = transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.yaw = euler[2]
        if not self.initial_yaw:
            self.yaw_list.append(self.yaw)
            if len(self.yaw_list) > 100 :
                self.init_yaw = np.mean(self.yaw_list)
                rospy.loginfo("initial yaw: {}".format(self.init_yaw))
                self.initial_yaw = True

    def move_x(self, goal):
        goal_ = goal.pose.position
        error_x = goal_.x - self.pose.pose.pose.position.x
        error_y = goal_.y - self.pose.pose.pose.position.y
        error = sqrt(error_x ** 2 + error_y ** 2)
        dt = 0.01
        speed = self.kp * error + self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        return np.clip(speed, -self.max_speed, self.max_speed)

    def pure_pursuit_control(self, goal, look_ahead_distance):
        goal_ = goal.pose.position
        current_position = (self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)
        look_ahead_point = self.calculate_look_ahead_point(current_position, (goal_.x, goal_.y), look_ahead_distance)

        if look_ahead_point is None:
            rospy.logwarn("No lookahead point found!")
            return

        alpha = atan2(look_ahead_point[1] - self.pose.pose.pose.position.y, look_ahead_point[0] - self.pose.pose.pose.position.x) - self.yaw
        curvature = 2 * sin(alpha) / look_ahead_distance
        speed = self.move_x(goal)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = curvature * 0.5
        self.cmd_vel_pub.publish(cmd)

    def calculate_look_ahead_point(self, current_position, goal_position, look_ahead_distance):
        path_vector = np.array(goal_position) - np.array(current_position)
        path_length = np.linalg.norm(path_vector)
        if path_length < look_ahead_distance:
            return None
        look_ahead_vector = (look_ahead_distance / path_length) * path_vector
        return np.array(current_position) + look_ahead_vector

    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def rotate_goal(self, goal_x, goal_y):
        cos_yaw = np.cos(self.init_yaw)
        sin_yaw = np.sin(self.init_yaw)
        rotated_x = goal_x * cos_yaw - goal_y * sin_yaw
        rotated_y = goal_x * sin_yaw + goal_y * cos_yaw
        return rotated_x, rotated_y

if __name__ == '__main__':
    jackal = Jackal()
    goal = PoseStamped()
    rospy.sleep(0.5)


    radius = jackal.radius
    num_waypoints = 36  
    center1_x , center1_y = jackal.rotate_goal(jackal.center_x1,0)
    center2_x , center2_y = jackal.rotate_goal(jackal.center_x2,0)
    center1 = (center1_x, center1_y)
    center2 = (center2_x, center2_y)

    waypoints_circle1 = [
        (
            center1[0] + radius * cos(pi +2 * pi * i / num_waypoints),
            center1[1] + radius * sin(pi+ 2 * pi * i / num_waypoints)
        )
        for i in range(num_waypoints)
    ]

    wp1 = [jackal.rotate_goal(x,y) for x,y in waypoints_circle1]

    waypoints_circle2 = [
        (
            center2[0] + radius * cos(-2 * pi * i / num_waypoints),
            center2[1] + radius * sin(-2 * pi * i / num_waypoints)
        )
        for i in range(num_waypoints)
    ]

    wp2 = [jackal.rotate_goal(x,y) for x,y in waypoints_circle2]

    waypoints = wp1 + wp2

    set_goal = 0
    look_ahead_distance = jackal.look_ahead
    laps_completed = 0

    try:
        while not rospy.is_shutdown():
            if dist(goal, jackal.pose) < look_ahead_distance:
                goal.pose.position.x, goal.pose.position.y = waypoints[set_goal]
                set_goal = (set_goal + 1) % len(waypoints)
                if set_goal == 0:
                    laps_completed += 1  
                if laps_completed >= 2:
		    goal.pose.position.x=0
		    goal.pose.position.y=0
                    for _ in range (2000):
			jackal.pure_pursuit_control(goal, look_ahead_distance)
			jackal.rate.sleep()
			if dist(goal, jackal.pose) < look_ahead_distance:
                    		rospy.loginfo("Completed 2 laps. Stopping.")
                    		break
		    break

            jackal.pure_pursuit_control(goal, look_ahead_distance)
            jackal.rate.sleep()
    except rospy.ROSInterruptException:
        sys.exit(0)

