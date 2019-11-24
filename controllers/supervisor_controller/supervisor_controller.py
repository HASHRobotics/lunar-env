#!/usr/bin/python2
"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import rospy
import tf
import numpy as np

from controller import Supervisor
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
from math import sqrt, atan2, cos, sin
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from scipy.io import loadmat
from random import random


last_odom = None
pose = [0,0,0]
a1 = 0
a2 = 0
a3 = 0
a4 = 0


rospy.init_node('supervisor_controller', anonymous=True)
odometry_publisher = rospy.Publisher('odometry_ground_truth', Odometry, queue_size=100)
noisy_odometry_publisher = rospy.Publisher('odometry', Odometry, queue_size=100)
odometry_error_publisher = rospy.Publisher('odom_error', Float64, queue_size=100)

def publish_noisy_odometry(odom):
    global last_odom
    if (last_odom == None):
        last_odom = odom
        pose[0] = odom.pose.pose.position.x
        pose[1] = odom.pose.pose.position.y
        q = [ odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w ]

        (r, p, y) = tf.transformations.euler_from_quaternion(q)
        pose[2] = y
    else:
        dx = odom.pose.pose.position.x - last_odom.pose.pose.position.x
        dy = odom.pose.pose.position.y - last_odom.pose.pose.position.y
        trans = sqrt(dx*dx + dy*dy)

        q = [ last_odom.pose.pose.orientation.x,
            last_odom.pose.orientation.y,
            last_odom.pose.orientation.z,
            last_odom.pose.orientation.w ]
        (r,p, theta1) = tf.transformations.euler_from_quaternion(q)

        q = [ odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w ]
        (r,p, theta2) = tf.transformations.euler_from_quaternion(q)

        rot1 = atan2(dy, dx) - theta1
        rot2 = theta2-theta1-rot1

        sd_rot1 = a1*abs(rot1) + a2*trans
        sd_rot2 = a1*abs(rot2) + a2*trans
        sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))

        trans +=  np.random.normal(0,sd_trans*sd_trans)
        rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        rot2 += np.random.normal(0, sd_rot2*sd_rot2)

        pose[0] += trans*cos(theta1+rot1)
        pose[1] += trans*sin(theta1+rot1)
        pose[2] = theta1 + rot1 + rot2
        last_odom = odom

        noisy_odometry_publisher((pose[0], pose[1],0),
                                 tf.transformations.quaternion_from_euler(0,0,pose[2]),
                                 odom.header.stamp,
                                 'baselink',
                                 'noisy_odom')
        broadcast_noisy_transform((pose(0), pose(1), 0), (0,0,pose(2)))


def broadcast_noisy_transform(position, orientation):
    current_time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        position,
        orientation,
        current_time,
        "base_link",
        "noisy_odom"
    )



show_rock_distances = rospy.get_param('show_rock_distances', 0)

def publish_odometry(position, orientation, velocity, angular_velocity):
    current_time = rospy.Time.now()
    broadcast_transform(position, orientation)

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(*position), Quaternion(*orientation))
    odom.twist.twist = Twist(Vector3(*velocity), Vector3(*angular_velocity))
    odom.child_frame_id = "base_link"
    odometry_publisher.publish(odom)

    # publish_noisy_odometry(odom)

def broadcast_transform(position, orientation):
    current_time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        position,
        orientation,
        current_time,
        "base_link",
        "odom"
    )

def publish_error():
    error = random()
    odometry_error_publisher.publish(error)


# import time
TIME_STEP = 32

# create the Robot instance.
# robot = Robot()

supervisor = Supervisor()

root = supervisor.getRoot()

# self_robot = supervisor.getSelf()

children = root.getField("children")
n = children.getCount()
for i in range(n):
    name = children.getMFNode(i).getTypeName()
    if(name == "DirectionalLight"):
        light_node_index = i
    elif(name == "Pioneer3at"):
        pioneer_3_at_index = i


light_node = children.getMFNode(light_node_index)
direction_field = light_node.getField("direction")

robot_node = children.getMFNode(pioneer_3_at_index)
#spice_data = loadmat("data/moon_rel_positions_44_25.mat")
spice_data = loadmat("/home/hash/Documents/lunar-env/data/moon_rel_positions_44_25.mat")
dir_sunlight = spice_data['U_sun_point_enu']

rotation_matrix = np.array([[1,0,0],[0,0,1],[0,-1,0]])

dir_sunlight = np.matmul(rotation_matrix,dir_sunlight)

if(show_rock_distances):
    rock_pos = np.load("/home/hash/Documents/lunar-env/data/rock_info.npy")
    rock_dist_publisher = rospy.Publisher("min_rock_dist", Float32, queue_size=10)

num_loops = 0
while(supervisor.step(TIME_STEP)!=-1):
    # if( num_loops % 125 == 0):
    # print("Changing light source now")
    if num_loops < dir_sunlight.shape[1]:
        direction_ = (dir_sunlight[:,num_loops]).tolist()
        # direction_[1]=-0.5
        direction_field.setSFVec3f(direction_)
        num_loops += 1
        if(num_loops == dir_sunlight.shape[1]):
            print("Done with one day")
    position = robot_node.getPosition()
    position[1] = 0.1073 
    orientation = np.array(robot_node.getOrientation()).reshape(3,3)
    orientation = np.matmul(orientation,np.array([[0,0,1],[0,1,0], [-1,0,0]])) # y 90
    orientation = np.matmul(orientation,np.array([[1,0,0],[0,0,1], [0,-1,0]])) # x -90
    orientation = np.hstack((np.vstack((orientation, np.zeros((1,3)))), np.array([[0,0,0,1]]).T))
    orientation = tf.transformations.quaternion_from_matrix(orientation)
    velocity = robot_node.getVelocity()
    publish_odometry(position, orientation, velocity[0:3], velocity[3:6])

    if(show_rock_distances):
        current_robot_position = np.array(robot_node.getPosition()).flatten()
        robot_rock_distances = np.sqrt(np.sum((rock_pos[:,:2] - current_robot_position[[0,2]])**2, axis=1)) - (rock_pos[:,2]/2 + 0.25)
        min_robot_rock_distance = robot_rock_distances.min()
        rock_dist_publisher.publish(min_robot_rock_distance)
    publish_error()



    # print((Robot)self_robot.getTime())
    #

#
# time = Supervisor.getTime()
# print(time)
# time.sleep(5)
# print("Changing light source")
# direction_field.setSFVec3f([0,-1,0])
# time = Supervisor.getTime()
# print(time)
# time.sleep(5)
# print("Changing light source")
# direction_field.setSFVec3f([0,0,-1])
# time = Supervisor.getTime()
# print(time)



# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.
