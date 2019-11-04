#!/usr/bin/env python

import rospy
import time
from gazebo_msgs.msg import ModelState, ModelStates

class Lab():

    def lab():
        rospy.init_node('simulation_5_obstacle_1')
        layout = moving()

    def moving(self):
        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'obstacle_1':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    obstacle_1.twist = model.twist[i]

                    obstacle_1.pose.position.x = random(-20,20)/10
                    obstacle_1.twist.linear.x = random(-20,20)/10

                    self.pub_model.publish(obstacle_1)
