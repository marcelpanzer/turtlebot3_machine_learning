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
import time
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from simulation_stage_2_respawnGoal import Respawn
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Pose

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


    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        # print self.position
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

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done

    def setReward(self, state, done, action):
        yaw_reward = []
        obstacle_min_range = state[-2]
        current_distance = state[-3]
        heading = state[-4]

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)

        if obstacle_min_range < 0.5:
            ob_reward = -5
        else:
            ob_reward = 0

        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 500
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward


    def step(self, action):
        start_time = time.time()
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        time_publish_start = time.time()
        self.pub_cmd_vel.publish(vel_cmd)
        time_publish_end = time.time()
        data = None

        time_data_start = time.time()
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        time_data_end = time.time()

        time_get_state_start = time.time()
        state, done = self.getState(data)
        time_get_state_end = time.time()


        time_set_reward_start = time.time()
        reward = self.setReward(state, done, action)
        time_set_reward_end = time.time()

        time_publish = time_publish_end - time_publish_start
        data_time = time_data_end - time_data_start
        getstate_time = time_get_state_end - time_get_state_start
        time_setreward = time_set_reward_end - time_set_reward_start
        time_step = time.time() - start_time

        return np.asarray(state), reward, done, time_publish, data_time, getstate_time, time_setreward, time_step

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)
