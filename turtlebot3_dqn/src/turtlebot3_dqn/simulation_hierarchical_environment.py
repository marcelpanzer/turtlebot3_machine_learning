#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from simulation_stage_2_respawnGoal import Respawn
from simulation_stage_dyanmic_respawnGoal import Respawn

# from respawnGoal import Respawn


class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.crashes=0
        self.goals=0
        self.data = rospy.wait_for_message('scan', LaserScan, timeout=5)


    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)
	# dqn 1
    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True
            self.goals += 1
            print("crashes:", self.crashes, "goals reached:", self.goals)

        return scan_range + [heading, current_distance], self.goals
	#dqn 2
    def getState_2(self, scan):
        scan_range = []
        min_range = 0.13
        done = False
	# filtering for short range planner
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(1.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            elif scan.ranges[i] >= 1.5:
                scan_range.append(1.5)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        if min_range > min(scan_range) > 0:
            done = True
            self.crashes += 1
            print("crashes:", self.crashes, "goals reached:", self.goals)

        return scan_range + [obstacle_min_range, obstacle_angle], done, self.crashes

    def setReward(self, state, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        # reward Funktion aus Vorlage einsetzen
        angle_reward = 3 - math.fabs(heading*6/math.pi)
        distance_rate = 2 ** (self.goal_distance/ current_distance)

        reward = angle_reward * distance_rate - 3

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 500
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def setReward_2(self, state, done, action_2, action):
        yaw_reward = []
        obstacle_min_range = state[-2]
        obstacle_angle = state[-1]

        if obstacle_min_range > 0.5:
            ob_reward = 5
        else:
            ob_reward = 1 + (math.fabs(obstacle_min_range)-0.5)*20
	
	#punishing deviation from angle
	if action == action_2:
	    reward_action = 3
	else:
            reward_action = 2 - 2* (math.fabs(action - action_2))

        if math.fabs(obstacle_angle) <= math.pi/8:
            reward_obstacle_angle = -3 + 3 * ((math.fabs(obstacle_angle))/(math.pi/8))
        else:
            reward_obstacle_angle = 1

        reward = ob_reward + reward_obstacle_angle + reward_action

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        return reward


    def step(self, action):
        self.data = None
        while self.data is None:
            try:
                self.data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, goals = self.getState(self.data)
        reward = self.setReward(state, action)

        return np.asarray(state), reward, goals


    def step_2(self, action_2, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/4 - action_2/2) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        self.data = None
        while self.data is None:
            try:
                self.data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state_2, done, crashes = self.getState_2(self.data)
        reward_2 = self.setReward_2(state_2, done, action_2, action)

        return np.asarray(state_2), reward_2, done, crashes

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        self.data = None
        while self.data is None:
            try:
                self.data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, goals = self.getState(self.data)
        state_2, done, crashes = self.getState_2(self.data)

        return np.asarray(state), np.asarray(state_2)
