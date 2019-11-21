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
import random
import time
import os
# from gazebo_msgs.srv import SpawnModel
# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

class Respawn():
    def __init__(self):
        # self.modelPath = os.path.dirname(os.path.realpath(__file__))
        # self.modelPath = self.modelPath.replace('turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
        #                                         'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        # self.f = open(self.modelPath, 'r')
        # self.model = self.f.read()
        self.start = PoseWithCovarianceStamped()
        self.goal_position = Pose()
        # self.modelName = 'goal'
        # self.goal_position.position.x = self.init_goal_x
        # self.goal_position.position.y = self.init_goal_y
        self.last_index = 0
        # self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0
        self.init = True
        self.update_goal = 0
        self.update_initial_pose = 0
        self.init_x = 0
        self.init_y = 0

    def getstartPosition(self):
        start = rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        # start = rospy.wait_for_message('initialpose_dqn', PoseWithCovarianceStamped);
        self.start.pose.pose.position.x = start.pose.pose.position.x
        self.start.pose.pose.position.y = start.pose.pose.position.y
        self.start.pose.pose.orientation = start.pose.pose.orientation
        return self.start.pose.pose.position.x, self.start.pose.pose.position.y, self.start.pose.pose.orientation

    def getPosition(self):
        if self.init == True:
            goal = rospy.wait_for_message('move_base_simple/goal', PoseStamped)
            self.goal_position.position.x = goal.pose.position.x
            self.goal_position.position.y = goal.pose.position.y
            self.init_x = goal.pose.position.x
            self.init_y = goal.pose.position.y
            self.init = False
        else:
            init = True
            while init:
                goal = rospy.wait_for_message('move_base_simple/goal', PoseStamped)
                self.goal_position.position.x = goal.pose.position.x
                self.goal_position.position.y = goal.pose.position.y
                if goal.pose.position.x != self.init_x and goal.pose.position.y != self.init_y:
                    init = False
                    self.init_x = goal.pose.position.x
                    self.init_y = goal.pose.position.y
                    print "New goal received"

        return self.goal_position.position.x, self.goal_position.position.y
