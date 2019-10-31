#!/usr/bin/env python2

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64
from controller import Robot, Camera, Keyboard
import os

class CustomRobotClass:

    def __init__(self):
        # Initialize Robot
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())


        # Initialize sensors
        # metacamera = robot.getCamera("MultiSense S21 meta camera")
        # leftcamera = robot.getCamera("MultiSense S21 left camera")
        # rightcamera = robot.getCamera("MultiSense S21 right camera")
        # rangeFinder = robot.getRangeFinder("MultiSense S21 meta range finder")
        self.leftpositionsensor = self.robot.getPositionSensor('back left wheel sensor')
        self.rightpositionsensor = self.robot.getPositionSensor('back right wheel sensor')


        # Enable sensors
        # metacamera.enable(timeStep)
        # leftcamera.enable(timeStep)
        # rightcamera.enable(timeStep)
        # rangeFinder.enable(timeStep)
        self.leftpositionsensor.enable(self.time_step)
        self.rightpositionsensor.enable(self.time_step)

        # Initialize wheels
        backrightwheel = self.robot.getMotor('back right wheel')
        backleftwheel = self.robot.getMotor('back left wheel')
        frontrigthwheel = self.robot.getMotor('front right wheel')
        frontleftwheel = self.robot.getMotor('front left wheel')
        self.wheels = [backrightwheel, backleftwheel, frontrigthwheel, frontleftwheel]

        self.set_velocity_control()

    def set_velocity_control(self):
        for wheel in self.wheels:
            wheel.setPosition((float('inf')))
            wheel.setVelocity(0)

    def step(self):
        self.robot.step(self.time_step)

class RosNode:
    def __init__(self):
        self.right_encoder_publisher = rospy.Publisher('rwheel', Float64, queue_size=10)
        self.left_encoder_publisher = rospy.Publisher('lwheel', Float64, queue_size=10)



def run_robot(ros_node, robot):
    while custom_robot.step() != -1 and not rospy.is_shutdown():
        ros_node.right_encoder_publisher.publish(robot.rightpositionsensor.getValue())
        ros_node.left_encoder_publisher.publish(robot.leftpositionsensor.getValue())
        for wheel in robot.wheels:
            wheel.setVelocity(5)


# def callback(data):
#     global velocity
#     global message
#     message = 'Received velocity value: ' + str(data.data)
#     velocity = data.data

# print('Subscribing to "motor" topic')
# rospy.Subscriber('motor', Float64, callback)

if __name__== "__main__":
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    custom_robot = CustomRobotClass()
    custom_robot.step()
    rospy.init_node('webots_controller', anonymous=True)
    ros_node = RosNode()
    run_robot(ros_node, custom_robot)
