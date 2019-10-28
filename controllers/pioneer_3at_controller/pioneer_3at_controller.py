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

# import rospy
# from std_msgs.msg import Float64
from controller import Robot, Camera
import os



def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data

robot = Robot()
print(robot)

timeStep = int(robot.getBasicTimeStep())
print(robot.__dict__)

metacamera = robot.getCamera("MultiSense S21 meta camera")
leftcamera = robot.getCamera("MultiSense S21 left camera")
rightcamera = robot.getCamera("MultiSense S21 right camera")
rangeFinder = robot.getRangeFinder("MultiSense S21 meta range finder")

metacamera.enable(timeStep)
leftcamera.enable(timeStep)
rightcamera.enable(timeStep)
rangeFinder.enable(timeStep)

rangeFinder.saveImage("/home/himil07/Documents/ros/worlds/test.png", 1)


print("Number of devices", robot.getNumberOfDevices())

for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    print("Device with index ", i, device.getName())


backrightwheel = robot.getMotor('back right wheel')
backleftwheel = robot.getMotor('back left wheel')
frontrigthwheel = robot.getMotor('front right wheel')
frontleftwheel = robot.getMotor('front left wheel')


sensor = robot.getPositionSensor('back left wheel sensor')  # front central proximity sensor
sensor.enable(timeStep)

sensor1 = robot.getPositionSensor('back right wheel sensor')  # front central proximity sensor
sensor1.enable(timeStep)

backrightwheel.setPosition(float('inf'))  # turn on velocity control for both motors
backleftwheel.setPosition(float('inf'))
frontrigthwheel.setPosition(float('inf'))
frontleftwheel.setPosition(float('inf'))

velocity = 0
backrightwheel.setVelocity(velocity)
backleftwheel.setVelocity(velocity)
frontrigthwheel.setVelocity(velocity)
frontleftwheel.setVelocity(velocity)

message = 1
# print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
robot.step(timeStep)
# rospy.init_node('listener', anonymous=True)
print('Subscribing to "motor" topic')
robot.step(timeStep)
# rospy.Subscriber('motor', Float64, callback)
# right_pub = rospy.Publisher('RightFrontsensor', Float64, queue_size=10)
# left_pub = rospy.Publisher('LeftFrontsensor', Float64, queue_size=10)
print('Running the control loop')
# while robot.step(timeStep) != -1 and not rospy.is_shutdown():
while robot.step(timeStep) != -1:
    # right_pub.publish(sensor.getValue())
    # left_pub.publish(sensor1.getValue())
    print('Published sensor value: ', sensor.getValue())
    print('Published sensor1 value: ', sensor1.getValue())
    if message:
        print(message)
        message = 1
        backrightwheel.setVelocity(5)
        backleftwheel.setVelocity(5)
        frontrigthwheel.setVelocity(5)
        frontleftwheel.setVelocity(5)