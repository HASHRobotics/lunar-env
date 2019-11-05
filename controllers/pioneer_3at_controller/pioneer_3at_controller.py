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
import math
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

SPEED_UNIT = 0.0628
INCR = 10
ENCODER_UNIT = 159.23
WHEEL_BASE = 0.25

#TODO: Change variables for pioneer 3AT
axis_wheel_ratio = 1
scaling_factor = 1
wheel_diameter_left = 0.222
wheel_diameter_right = 0.222
increments_per_tour = 10


class CustomRobotClass:

    def __init__(self):
        # Initialize Robot
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())


        # Initialize sensors
        metacamera = self.robot.getCamera("MultiSense S21 meta camera")
        leftcamera = self.robot.getCamera("MultiSense S21 left camera")
        rightcamera = self.robot.getCamera("MultiSense S21 right camera")
        rangeFinder = self.robot.getRangeFinder("MultiSense S21 meta range finder")
        self.leftpositionsensor = self.robot.getPositionSensor('back left wheel sensor')
        self.rightpositionsensor = self.robot.getPositionSensor('back right wheel sensor')
        self.keyboard = Keyboard()

        # Enable sensors
        metacamera.enable(self.timeStep)
        leftcamera.enable(self.timeStep)
        rightcamera.enable(self.timeStep)
        rangeFinder.enable(self.timeStep)
        self.leftpositionsensor.enable(self.time_step)
        self.rightpositionsensor.enable(self.time_step)
        self.keyboard.enable(self.time_step)

        # Initialize wheels
        backrightwheel = self.robot.getMotor('back right wheel')
        backleftwheel = self.robot.getMotor('back left wheel')
        frontrightwheel = self.robot.getMotor('front right wheel')
        frontleftwheel = self.robot.getMotor('front left wheel')
        self.wheels = [backrightwheel, backleftwheel, frontrightwheel, frontleftwheel]

        self.set_velocity_control()
        self.prevlspeed = 0
        self.prevrspeed = 0
        self.curlspeed = 0
        self.currspeed = 0

    def set_velocity_control(self):
        for wheel in self.wheels:
            wheel.setPosition((float('inf')))
            wheel.setVelocity(0)

    def step(self):
        self.robot.step(self.time_step)

    def set_speed(self, l, r):
        if self.prevlspeed !=l or self.prevrspeed != r:
            self.wheels[0].setVelocity(SPEED_UNIT*r)
            self.wheels[1].setVelocity(SPEED_UNIT*l)
            self.wheels[2].setVelocity(SPEED_UNIT*r)
            self.wheels[3].setVelocity(SPEED_UNIT*l)
            self.prevlspeed = self.curlspeed
            self.prevrspeed = self.currspeed
            self.curlspeed = l
            self.currspeed = r
            # print("Speed",l,r)

class OdometryClass:

    def __init__(self):
        # Configuration
        self.wheel_distance = 0
        self.wheel_conversion_left = 0
        self.wheel_conversion_right = 0

        # State
        self.pos_left_prev = 0
        self.pos_right_prev = 0

        # Result
        self.x = 0
        self.y = 0
        self.theta = 0

    def odometry_track_step_pose(self, pos_left, pos_right):
        delta_pos_left = pos_left - self.pos_left_prev;
        delta_pos_right = pos_right - self.pos_right_prev;

        delta_left = delta_pos_left * self.wheel_conversion_left
        delta_right = delta_pos_right * self.wheel_conversion_right
        delta_theta = (delta_right - delta_left) / self.wheel_distance
        theta2 = self.theta + delta_theta * 0.5;
        delta_x = (delta_left + delta_right) * 0.5 * math.cos(theta2)
        delta_y = (delta_left + delta_right) * 0.5 * math.sin(theta2)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        if self.theta > math.pi:
            self.theta -= 2*math.pi
        if self.theta < -math.pi:
            self.theta += 2*math.pi

        self.pos_left_prev = pos_left;
        self.pos_right_prev = pos_right;

    def odometry_start_pos(self, pos_left, pos_right):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.pos_left_prev = pos_left
        self.pos_right_prev = pos_right

        self.wheel_distance = axis_wheel_ratio*scaling_factor*(wheel_diameter_left + wheel_diameter_right)/2
        self.wheel_conversion_left = wheel_diameter_left * scaling_factor * math.pi / increments_per_tour
        self.wheel_conversion_right = wheel_diameter_right * scaling_factor * math.pi / increments_per_tour

# class OdometryGotoClass:

#     def __init__(self):
#         self.speed_min = 1
#         self.goal_x = 0
#         self.goal_y = 0
#         self.goal_theta = 0
#         self.speed_left = 0
#         self.speed_right = 0
#         self.atgoal = 1

#     def set_goal(self, x, y, theta):
#         self.goal_x = x
#         self.goal_y = y
#         self.goal_theta = theta
#         self.atgoal = 0

#     def step(self, odometry):
#         cur_position = [odometry.x, odometry.y, odometry.theta]
#         goal_position = [self.goal_x, self.goal_y, self.goal_theta]

#         dx = goal_position[0] - cur_position[0]
#         dy = goal_position[1] - cur_position[1]

#         # Gain parameters
#         k_rho = 1.2
#         k_alpha = 1.5
#         k_beta = -0.35

#         v_adapt = 1000/0.13
#         w_adapt = 2000/ (270 * math.pi/180)

#         rho_c = math.sqrt(dx*dx + dy*dy)
#         alpha_c = math.atan2(dy, dx) - cur_position[2]
#         while alpha_c > math.pi:
#             alpha_c = alpha_c - 2 * math.pi;
#         while alpha_c < -math.pi:
#             alpha_c = alpha_c + 2 * math.pi;

#         beta_c = -cur_position[2] - alpha_c
#         while alpha_c > math.pi:
#             beta_c = beta_c - 2 * math.pi;
#         while alpha_c < -math.pi:
#             beta_c = beta_c + 2 * math.pi;

#         # Control Law
#         v_c = k_rho * rho_c
#         w_c = k_alpha * alpha_c + k_beta * beta_c

#         # Robot units
#         v_e = v_c * v_adapt
#         w_e = w_c * w_adapt


#         speed_left = (v_e - w_e/2)
#         speed_right = (v_e + w_e/2)

#         if (speed_right == 0 and speed_left == 0) or rho_c < 0.002:
#             self.atgoal = 1

#         return [speed_left, speed_right]

def command_velocity_callback(data):
    global left_velocity
    global right_velocity
    robot_linear_velocity = data.linear.x
    robot_angular_velocity = data.angular.z

    left_velocity = robot_linear_velocity - 0.5*robot_angular_velocity*WHEEL_BASE
    right_velocity = robot_linear_velocity + 0.5*robot_angular_velocity*WHEEL_BASE


class RosNode:
    def __init__(self):
        self.right_encoder_publisher = rospy.Publisher('rwheel', Float64, queue_size=10)
        self.left_encoder_publisher = rospy.Publisher('lwheel', Float64, queue_size=10)
        self.odometry_publisher = rospy.Publisher('odometry', Odometry, queue_size=100)
        rospy.Subscriber('cmd_vel', Twist, command_velocity_callback)

    def publish_odometry(self, odometry):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, odometry.theta)
        current_time = rospy.Time.now()
        self.broadcast_transform(odometry)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(odometry.x, odometry.y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        self.odometry_publisher.publish(odom)


    def broadcast_transform(self, odometry):
        current_time = rospy.Time.now()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, odometry.theta)
        odom_broadcaster = tf.TransformBroadcaster()
        #TODO: WHat is the base link here ?
        odom_broadcaster.sendTransform(
            (odometry.x, odometry.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )


# def goto_position(x,y,theta, odometry, robot):
#     odometrygoto = OdometryGotoClass()
#     odometrygoto.set_goal(x, y, theta)
#     while odometrygoto.atgoal == 0:
#         pos_left = ENCODER_UNIT*robot.leftpositionsensor.getValue()
#         pos_right = ENCODER_UNIT*robot.rightpositionsensor.getValue()
#         odometry.odometry_track_step_pose(pos_left, pos_right)
#         [speed_left, speed_right] = odometrygoto.step(odometry)
#         robot.set_speed(speed_left, speed_right)


def run_robot(ros_node, robot):
    global left_velocity
    global right_velocity
    odometry = OdometryClass()
    pos_left = ENCODER_UNIT*robot.leftpositionsensor.getValue()
    pos_right = ENCODER_UNIT*robot.rightpositionsensor.getValue()
    print(robot.leftpositionsensor.getValue(), robot.rightpositionsensor.getValue())
    odometry.odometry_start_pos(pos_left, pos_right)
    while custom_robot.step() != -1 and not rospy.is_shutdown():
        print("I am here")
        left_velocity = robot.curlspeed
        right_velocity = robot.currspeed
        key = robot.keyboard.getKey()
        if key == Keyboard.UP:
            left_velocity += INCR
            right_velocity += INCR
        elif key == Keyboard.DOWN:
            left_velocity -= INCR
            right_velocity -= INCR
        elif key == Keyboard.LEFT:
            right_velocity += INCR
            left_velocity -= INCR
        elif key == Keyboard.RIGHT:
            right_velocity -= INCR
            left_velocity += INCR
        robot.set_speed(left_velocity,right_velocity)
        ros_node.right_encoder_publisher.publish(robot.rightpositionsensor.getValue())
        ros_node.left_encoder_publisher.publish(robot.leftpositionsensor.getValue())

        pos_left = ENCODER_UNIT*robot.leftpositionsensor.getValue()
        pos_right = ENCODER_UNIT*robot.rightpositionsensor.getValue()
        odometry.odometry_track_step_pose(pos_left, pos_right)

        ros_node.publish_odometry(odometry)
        # Go to Position
        # goto_position(0, 0.2, math.pi, odometry, robot);

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
