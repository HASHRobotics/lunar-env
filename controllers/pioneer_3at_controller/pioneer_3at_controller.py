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
from sensor_msgs.msg import Image, CameraInfo
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

TIME_STEP = 32

SPEED_UNIT = 9.09
INCR = 0.01
ENCODER_UNIT = 159.23
WHEEL_BASE = 0.25
MAX_SPEED = 6.4
TELEOP = 0

left_velocity = 0
right_velocity = 0

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
        print("Robot time step ", self.time_step)


        # Initialize sensors
        self.metacamera = self.robot.getCamera("MultiSense S21 meta camera")
        self.leftcamera = self.robot.getCamera("MultiSense S21 left camera")
        self.rightcamera = self.robot.getCamera("MultiSense S21 right camera")
        self.rangeFinder = self.robot.getRangeFinder("MultiSense S21 meta range finder")
        self.cameras = {
            "left": self.leftcamera,
            "right": self.rightcamera,
            "meta": self.metacamera,
            "depth": self.rangeFinder
            }
        self.leftpositionsensor = self.robot.getPositionSensor('back left wheel sensor')
        self.rightpositionsensor = self.robot.getPositionSensor('back right wheel sensor')
        self.keyboard = Keyboard()
        self.pen = self.robot.getPen('pen')
        # Enable sensors
        self.metacamera.enable(self.time_step)
        self.leftcamera.enable(self.time_step)
        self.rightcamera.enable(self.time_step)
        self.rangeFinder.enable(self.time_step)
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
        print("Inside set speed", SPEED_UNIT*l, SPEED_UNIT*r)
        if self.prevlspeed !=l or self.prevrspeed != r:
            self.wheels[0].setVelocity(SPEED_UNIT*r)
            self.wheels[1].setVelocity(SPEED_UNIT*l)
            self.wheels[2].setVelocity(SPEED_UNIT*r)
            self.wheels[3].setVelocity(SPEED_UNIT*l)
            self.prevlspeed = self.curlspeed
            self.prevrspeed = self.currspeed
            self.curlspeed = l
            self.currspeed = r

    def get_image(self, camera_name, depth_option=False):
        if(depth_option==True):
            return self.cameras[camera_name].getRangeImageArray()    
        else:
            return self.cameras[camera_name].getImageArray()

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
    print("Inside command velocity: left_velocity: {0}, right_velocity: {1}".format(left_velocity, right_velocity))

class RosNode:
    def __init__(self):
        self.right_encoder_publisher = rospy.Publisher('rwheel', Float64, queue_size=10)
        self.left_encoder_publisher = rospy.Publisher('lwheel', Float64, queue_size=10)
        self.odometry_publisher = rospy.Publisher('odometry', Odometry, queue_size=100)
        left_camera_publisher = rospy.Publisher('left_camera', Image, queue_size=50)
        left_camerainfo_publisher = rospy.Publisher('left_camera/camera_info', CameraInfo, queue_size=50)
        right_camera_publisher = rospy.Publisher('right_camera', Image, queue_size=50)
        right_camerainfo_publisher = rospy.Publisher('right_camera/camera_info', CameraInfo, queue_size=50)
        meta_camera_publisher = rospy.Publisher('meta_camera', Image, queue_size=50)
        meta_camerainfo_publisher = rospy.Publisher('meta_camera/camera_info', CameraInfo, queue_size=50)
        depth_camera_publisher = rospy.Publisher('depth_camera', Image, queue_size=50)
        self.camera_publishers = {'left': left_camera_publisher, 'right': right_camera_publisher, 'meta': meta_camera_publisher, 'depth': depth_camera_publisher}
        self.camerainfo_publishers = {'left': left_camerainfo_publisher, 'right': right_camerainfo_publisher, 'meta': meta_camerainfo_publisher}
        rospy.Subscriber('cmd_vel', Twist, command_velocity_callback)

        self.bridge = CvBridge()

    def initailize_camera_params(self, left_camera, right_camera, meta_camera, depth_camera):
        self.left_camera_info = CameraInfo()
        self.left_camera_info.width = left_camera.getWidth()
        self.left_camera_info.height = left_camera.getHeight()
        f = left_camera.getWidth()/(2*math.tan(left_camera.getFov()/2))
        self.left_camera_info.K[0] = f
        self.left_camera_info.K[4] = f
        self.left_camera_info.K[2] = left_camera.getWidth()/2
        self.left_camera_info.K[5] = left_camera.getHeight()/2
        self.left_camera_info.K[8] = 1

        self.right_camera_info = CameraInfo()
        self.right_camera_info.width = right_camera.getWidth()
        self.right_camera_info.height = right_camera.getHeight()
        f = right_camera.getWidth()/(2*math.tan(right_camera.getFov()/2))
        self.right_camera_info.K[0] = f
        self.right_camera_info.K[4] = f
        self.right_camera_info.K[2] = right_camera.getWidth()/2
        self.right_camera_info.K[5] = right_camera.getHeight()/2
        self.right_camera_info.K[8] = 1

        self.meta_camera_info = CameraInfo()
        self.meta_camera_info.width = meta_camera.getWidth()
        self.meta_camera_info.height = meta_camera.getHeight()
        f = meta_camera.getWidth()/(2*math.tan(meta_camera.getFov()/2))
        self.meta_camera_info.K[0] = f
        self.meta_camera_info.K[4] = f
        self.meta_camera_info.K[2] = meta_camera.getWidth()/2
        self.meta_camera_info.K[5] = meta_camera.getHeight()/2
        self.meta_camera_info.K[8] = 1
        self.meta_camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.meta_camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.meta_camera_info.P = [f, 0.0, meta_camera.getWidth()/2, 0.0, 0.0, f, meta_camera.getHeight()/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        

        self.camera_infos = {'left': self.left_camera_info, 'right':self.right_camera_info, 'meta':self.meta_camera_info}

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
    
    def broadcast_image(self, img, camera):
        # image = Image()
        # image.width = len(img)
        # image.height = len(img[0])
        # img = np.transpose(np.array(img, dtype=np.uint8), (1,0,2))
        # time_now = rospy.Time.now()
        # image.header.stamp = time_now
        # image.header.frame_id = camera+"_image"
        # image.data = img.tolist()
        # image.encoding = "rgb8"
        time_now = rospy.Time.now()
        if(camera == 'depth'):
            img = np.transpose(np.array(img, dtype=np.uint16))
            image = self.bridge.cv2_to_imgmsg(img, "mono16")
        else:
            img = np.transpose(np.array(img, dtype=np.uint8), (1,0,2))
            image = self.bridge.cv2_to_imgmsg(img, "rgb8")
            self.camera_infos[camera].header.stamp = time_now
            self.camera_infos[camera].header.frame_id = "base_link"
            self.camerainfo_publishers[camera].publish(self.camera_infos[camera])
        image.header.stamp = time_now
        image.header.frame_id = "base_link"
        self.camera_publishers[camera].publish(image)
        


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
    odometry.odometry_start_pos(pos_left, pos_right)

    ros_node.initailize_camera_params(robot.leftcamera, robot.rightcamera, robot.metacamera, robot.rangeFinder)

    relative_rate = 1
    while custom_robot.step() != -1 and not rospy.is_shutdown():
        if TELEOP:
            left_velocity = robot.curlspeed
            right_velocity = robot.currspeed
            key = robot.keyboard.getKey()
            if key == Keyboard.UP:
                if left_velocity < MAX_SPEED:
                    left_velocity += INCR
                if right_velocity < MAX_SPEED:
                    right_velocity += INCR
            elif key == Keyboard.DOWN:
                if left_velocity > -MAX_SPEED:
                    left_velocity -= INCR
                if right_velocity > -MAX_SPEED:
                    right_velocity -= INCR
            elif key == Keyboard.LEFT:
                if right_velocity < MAX_SPEED:
                    right_velocity += INCR
                if left_velocity > -MAX_SPEED:
                    left_velocity -= INCR
            elif key == Keyboard.RIGHT:
                if left_velocity < MAX_SPEED:
                    left_velocity += INCR
                if right_velocity > -MAX_SPEED:
                    right_velocity -= INCR
        print("left_velocity = {0} and right_velocity = {1}".format(left_velocity, right_velocity))
        robot.set_speed(left_velocity,right_velocity)
        robot.pen.write(True)
        ros_node.right_encoder_publisher.publish(robot.rightpositionsensor.getValue())
        ros_node.left_encoder_publisher.publish(robot.leftpositionsensor.getValue())

        pos_left = ENCODER_UNIT*robot.leftpositionsensor.getValue()
        pos_right = ENCODER_UNIT*robot.rightpositionsensor.getValue()
        odometry.odometry_track_step_pose(pos_left, pos_right)

        ros_node.publish_odometry(odometry)
        if(relative_rate == 0):
            # ros_node.broadcast_image(robot.get_image('left'), 'left')
            # ros_node.broadcast_image(robot.get_image('right'), 'right')
            ros_node.broadcast_image(robot.get_image('meta'), 'meta')
            ros_node.broadcast_image(robot.get_image('depth', depth_option=True), 'depth')
            relative_rate = 1
        relative_rate -= 1
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
