#!/usr/bin/python2
"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import rospy
import tf

from controller import Supervisor
from scipy.io import loadmat
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


rospy.init_node('supervisor_controller', anonymous=True)
odometry_publisher = rospy.Publisher('odometry_ground_truth', Odometry, queue_size=100)

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
spice_data = loadmat("../../data/moon_rel_positions.mat")
dir_sunlight = -spice_data['U_sun_point_me']

rotation_matrix = np.array([[1,0,0],[0,0,-1],[0,1,0]])

dir_sunlight = np.matmul(rotation_matrix,dir_sunlight)

num_loops = 0
while(supervisor.step(TIME_STEP)!=-1):
    # if( num_loops % 125 == 0):
    # print("Changing light source now")
    if num_loops < dir_sunlight.shape[1]:
        direction_ = (dir_sunlight[:,num_loops]).tolist()
        direction_[1]=-0.5
        direction_field.setSFVec3f(direction_)
        num_loops += 1
        if(num_loops == dir_sunlight.shape[1]):
            print("Done with one day")
    position = robot_node.getPosition()
    orientation = np.array(robot_node.getOrientation()).reshape(3,3)
    orientation = np.hstack((np.vstack((orientation, np.zeros((1,3)))), np.array([[0,0,0,1]]).T))
    orientation = tf.transformations.quaternion_from_matrix(orientation)
    velocity = robot_node.getVelocity()
    publish_odometry(position, orientation, velocity[0:3], velocity[3:6])



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
