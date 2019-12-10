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
from geometry_msgs.msg import Twist, Point, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from simulation_respawn_real import Respawn
# from nodes.turtlebot3_real_transmission_2 import Sender
# from gazebo_msgs.msg import ModelStates, ModelState

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.start_x = 0
        self.start_y = 0
        self.start_orientation = PoseWithCovarianceStamped()
        self.heading = 0
        self.count = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.position_x, self.position_y = 0, 0
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch = True)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.respawn_goal = Respawn()
        self.action_memory = []
        self.time_start = time.time()
        self.orientation, self.yaw_init = 0, 0
        self.goal_x_map, self.goal_y_map = 0, 0

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        self.position_x, self.position_y = self.position.x, self.position.y
        orientation = odom.pose.pose.orientation
        self.orientation = orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        # print "odom yaw: ", yaw

        goal_angle = math.atan2(self.goal_y - self.position.y , self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        scan_range2 = []
        # print scan.ranges
        heading = self.heading
        min_range = 0.3
        done = False

        # no filter
        # for i in range(len(scan.ranges)):
            # if scan.ranges[i] == float('Inf'):
            #     scan_range.append(3.5)
            # # zero Problem
            # # elif np.isnan(scan.ranges[i]):
            # #     scan_range.append(0)
            # elif scan.ranges[i] <= 0.07:
            #     scan_range.append(3.5)
            # else:
            #     scan_range.append(scan.ranges[i])

        # Filter
        i = 0
        while i <= len(scan.ranges)-1:
            # print "length", len(scan_range)
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
                i += 1
            elif scan.ranges[i] == 0:
                k = 1
                t = 0
                if i == 0:
                    while scan.ranges[k]==0:
                        k += 1
                    while t <= k:
                        scan_range.append(scan.ranges[k])
                        t += 1
                    i = k + 1
                else:
                    k = i
                    m = i
                    a = scan.ranges[i-1]
                    while scan.ranges[k]==0:
                        if k == 359:
                            while m <= k:
                                scan_range.append(a)
                                m += 1

                            for i in range(len(scan_range)):
                                if scan_range[i] < 0.12:
                                    scan_range2.append(0.12)
                                else:
                                    scan_range2.append(scan_range[i])

                            current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
                            # if current_distance < 0.2:
                            if current_distance < 0.15:
                                vel_cmd = Twist()
                                self.get_goalbox = True
                            obstacle_min_range = round(min(scan_range), 2)
                            obstacle_angle = np.argmin(scan_range)
                            if min_range > min(scan_range) > 0:
                                done = True

                            return scan_range2 + [heading, current_distance, obstacle_min_range, obstacle_angle], done
                        k += 1

                    b = scan.ranges[k]

                    while m < k:
                        scan_range.append(max(a, b))
                        m += 1
                    i = k
            else:
                scan_range.append(scan.ranges[i])
                i += 1

        i=0
        for i in range(len(scan_range)):
            if scan_range[i] < 0.12:
                scan_range2.append(0.12)
            else:
                scan_range2.append(scan_range[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        if min_range > min(scan_range) > 0:
            done = True
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)

        # if current_distance < 0.2:
        if current_distance < 0.15:
            vel_cmd = Twist()
            self.get_goalbox = True

        return scan_range2 + [heading, current_distance, obstacle_min_range, obstacle_angle], done

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
            rospy.loginfo("Near Collision!!")
            reward = -200
            # driving backwards last 25 actions ~5 seconds
            t = 0
            l = len(self.action_memory)
            vel_cmd = Twist()
            # while t <= 10:
            #     if len(self.action_memory) > 20:
            #         max_angular_vel = -1.5
            #         action = self.action_memory[l-t-1]
            #         ang_vel = ((-self.action_size + 1)/2 - action) * max_angular_vel * 0.5
            #         vel_cmd.linear.x = -0.15
            #         # vel_cmd.angular.z = ang_vel
            #         vel_cmd.angular.z = 0
            #         time_start = time.time()
            #         a=0
            #         self.pub_cmd_vel.publish(vel_cmd)
            #         t += 1
            #     else:
            #         t = 10
            # stand still after collision
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            time_start = time.time()
            a=0
            while a < 1:
                self.pub_cmd_vel.publish(vel_cmd)
                a = time.time() - time_start

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            print "start_position: ", self.start_x,"/ ", self.start_y
            print "odom_position:", self.position.x,"/ " ,self.position.y
            print "goal_position: ", self.goal_x,"/ ", self.goal_y
            print "action: ", action
            print "_______________________________________________________________"
            reward = 500
            self.get_goalbox = False
            done = True
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            start = 0
            start_1 = time.time()
            while start - 5 < 0:
                self.pub_cmd_vel.publish(vel_cmd)
                start = time.time() - start_1
            # self.pub_cmd_vel.publish(vel_cmd)
            # self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            # self.goal_distance = self.getGoalDistace()

        return reward, done

    def speed(self, state):
        # Calculate the data new with a filter
        scan_range = []
        speed = 0.15
        speed_goal = 0

        for i in range(len(state)):
            if state[i] < 0.30:
                scan_range.append(3.5)
            else:
                scan_range.append(state[i])
        scan_range = state

        obstacle_min_range = round(min(scan_range), 2)
        goal_distance = scan_range[361]
        # print obstacle_min_range

        if obstacle_min_range >= 1:
            speed = 0.15
        elif obstacle_min_range < 1 and obstacle_min_range >= 0.3:
            speed = 0.15 + ((obstacle_min_range-1)/7)
        speed_goal = speed
        if goal_distance < 0.5:
            speed_goal = 0.15 + (goal_distance - 0.)/8
        speed = min([speed, speed_goal])
        return speed

    def step(self, action):
        time1 = time.time()
        data = None

        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        state, done = self.getState(data)
        reward, done = self.setReward(state, done, action)
        if not done:
            max_angular_vel = 1.5
            # max_angular_vel = 0.15
            ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
            vel_cmd = Twist()
            vel_cmd.linear.x = self.speed(state)
            # vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = ang_vel
            self.action_memory.append(-1*action)
            time_start = time.time()
            self.pub_cmd_vel.publish(vel_cmd)

        if self.count % 2 == 0:
            print "start_position: ", self.start_x,"/ ", self.start_y
            print "odom_position:", self.position.x,"/ " ,self.position.y
            print "goal_position: ", self.goal_x,"/ ", self.goal_y
            print "goal_distance: ", state[-3],"/ obstacle_distance: ", state[-2]
            print "Vel_linear: ",vel_cmd.linear.x , "action: ", action
            print done
            print "_____________________________________________________________"
        self.count += 1

        return np.asarray(state), reward, done

    def reset(self):
        # corrdinate receive, transformation
        yaw_neu = 0
        if self.initGoal:
            self.start_x_map, self.start_y_map, start_orientation_2 = self.respawn_goal.getstartPosition()
            self.goal_x_map, self.goal_y_map = self.respawn_goal.getPosition()
            start_orientation_list = [start_orientation_2.x, start_orientation_2.y, start_orientation_2.z, start_orientation_2.w]
            _, _, self.yaw_init = euler_from_quaternion(start_orientation_list)
            self.initGoal = False
            # self.goal_x, self.goal_y = self.goal_x_map, self.goal_y_map
        else:
            self.start_x_map, self.start_y_map = self.goal_x_map, self.goal_y_map
            self.goal_x_map, self.goal_y_map = self.respawn_goal.getPosition()
            orientation = self.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw_neu = euler_from_quaternion(orientation_list)
            print "yaw_neu:", yaw_neu
            # self.goal_x_map, self.goal_y_map = self.goal_x, self.goal_y
            print "Wait 3 sec"
            time.sleep(3)

        # in map coordinates
        # diff_x = self.goal_x - self.start_x + self.position
        # diff_y = self.goal_y - self.start_y + self.position
        diff_x = self.goal_x_map - self.start_x_map
        diff_y = self.goal_y_map - self.start_y_map
        print "diff_x: ", diff_x
        print "diff_y: ", diff_y
        print "yaw_neu: ", yaw_neu
        # yaw = yaw_neu + self.yaw_init
        # print "yaw: ",yaw
        # Transformation
        yaw = self.yaw_init

        self.goal_x = math.cos(yaw)*diff_x + math.sin(yaw)*diff_y + self.position_x
        self.goal_y = -1*math.sin(yaw)*diff_x + math.cos(yaw)*diff_y + self.position_y
        self.goal_distance = self.getGoalDistace()


        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)
