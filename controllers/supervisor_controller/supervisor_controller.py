"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Supervisor, Robot

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
    print(name)
    if(name == "DirectionalLight"):
        light_node_index = i
        break


light_node = children.getMFNode(light_node_index)
direction_field = light_node.getField("direction")

num_loops = 0
while(supervisor.step(TIME_STEP)!=-1):
    num_loops += 1
    # if( num_loops % 125 == 0):
    print("Changing light source")
    direction_field.setSFVec3f([-num_loops / 125.0,-1,0])
        
        
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
