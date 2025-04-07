import pybullet as p
import time
import pybullet_data
import math as m
import numpy as np

# variables for the joints
quadril_1 = 3
coxa_1 = 0
pata_1 = 1
quadril_2 = 9
coxa_2 = 10
pata_2 = 11
quadril_3 = 13
coxa_3 = 14
pata_3 = 15
quadril_4 = 5
coxa_4 = 6
pata_4 = 7
pata_1_frame = 2
pata_2_frame = 12
pata_3_frame = 8
pata_4_frame = 16
centro_frame = 4
leg_1 = [quadril_1, coxa_1, pata_1]
leg_2 = [quadril_2, coxa_2, pata_2]
leg_3 = [quadril_3, coxa_3, pata_3]
leg_4 = [quadril_4, coxa_4, pata_4]
L1 = 34 # mm
L2 = 55 # mm
L3 = 55 # mm

def control_joint_position(joint_index, target_position, config =True):
    """ accepts joint index and target position in degrees """
    if config:
        # ajustando perna 1
        if joint_index == quadril_1:
            target_position = m.pi/2 - target_position
        elif joint_index == coxa_1:
            target_position = target_position
        elif joint_index == pata_1:
            target_position = m.pi-target_position
        # ajustando perna 2
        elif joint_index == quadril_2:
            target_position = m.pi/2-target_position
        elif joint_index == coxa_2:
            target_position = -target_position
        elif joint_index == pata_2:
            target_position = -m.pi+target_position
        # ajustando perna 3
        elif joint_index == quadril_3:
            target_position = m.pi/2-target_position
        elif joint_index == coxa_3:
            target_position = -target_position
        elif joint_index == pata_3:
            target_position = -m.pi+target_position
        # ajustando perna 4
        elif joint_index == quadril_4:
            target_position = m.pi/2 -target_position
        elif joint_index == coxa_4:
            target_position = target_position
        elif joint_index == pata_4:
            target_position = m.pi -target_position
    p.setJointMotorControl2(
        bodyUniqueId=robotId,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        force=5,
    )
def inverse_kinematics(leg, x, y, z):
    """ accepts leg list and target position in cartesian coordinates """
    c = (z**2+y**2)**0.5
    theta_1 = m.pi -m.atan2(z,y) - m.asin(L1/c)
    R_x = -m.pi/2 + theta_1
    z_2 = y*m.sin(R_x) + z*m.cos(R_x)
    h = (z_2**2 + x**2)**0.5
    theta_2 = m.acos((L2**2 - L3**2 + h**2)/(2*L2*h)) - m.atan2(x,z)
    theta_3 = m.acos((L2**2 + L3**2 - h**2)/(2*L2*L3))
    control_joint_position(leg[0], theta_1,True)
    control_joint_position(leg[1], theta_2,True)
    control_joint_position(leg[2], theta_3,True)
# Connect to the physics server
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version

# Set additional search path for PyBullet data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load a plane URDF
planeId = p.loadURDF("plane.urdf")

# Load a robot
robotId = p.loadURDF("robot.urdf", [0, 0, 0.10], useFixedBase=False) # normal "free" mode
#robotId = p.loadURDF("robot.urdf", [0, -0.25, 0.25], useFixedBase=True) # debug fixed mode
# Get the number of joints
numJoints = p.getNumJoints(robotId)
print(f"Number of joints: {numJoints}")


# Print joint information
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {jointInfo[1]}")  # Print joint name


# Enable joint position control
for i in range(numJoints):
    p.setJointMotorControl2(
        bodyUniqueId=robotId,
        jointIndex=i,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0,  # Initial target position
        force=50,  # Maximum force to apply
    )
path_l1 = []
path_l2 = []
path_l3 = []
path_l4 = []
j1 = 0 # for leg 1
j2 = 0 # for leg 2
j3 = 0 # for leg 3
j4 = 0 # for leg 4
decrease_p1 = False
decrease_p2 = False
decrease_p3 = False
decrease_p4 = False
# initial leg phases
# trot gait
stence_leg1 = True
stence_leg2 = True
stence_leg3 = False
stence_leg4 = False
swing_leg1 = False
swing_leg2 = False
swing_leg3 = True
swing_leg4 = True
x_0 = 15
# Run the simulation
while True:
    # Set target positions for joints in degrees
    y_height = 75
    radius = 20
    #for leg 1
    if j1 >= radius + x_0 and decrease_p1 == False:
        decrease_p1 = True
        stence_leg1 = False
        swing_leg1 = True
    elif j1 <= -radius + x_0 and decrease_p1 == True:
        decrease_p1 = False
        stence_leg1 = True
        swing_leg1 = False
    if decrease_p1:
        j1 -= 1
    else:
        j1 += 1
    #for leg 2
    if j2 >= radius + x_0  and decrease_p2 == False:
        decrease_p2 = True
        stence_leg2 = True
        swing_leg2 = False
    elif j2 <= -radius + x_0 and decrease_p2 == True:
        decrease_p2 = False
        stence_leg2 = False
        swing_leg2 = True
    if decrease_p2:
        j2 -= 1
    else:
        j2 += 1
    #for leg 3
    if j3 >= radius + x_0 and decrease_p3 == False:        
        decrease_p3 = True
        stence_leg3 = True
        swing_leg3 = False
    elif j3 <= -radius + x_0 and decrease_p3 == True:
        decrease_p3 = False
        stence_leg3 = False
        swing_leg3 = True
    if decrease_p3:
        j3 -= 1
    else:
        j3 += 1
    #for leg 4
    if j4 >= radius + x_0 and decrease_p4 == False:
        decrease_p4 = True
        stence_leg4 = False
        swing_leg4 =True
    elif j4 <= -radius + x_0 and decrease_p4 == True:
        decrease_p4 = False
        stence_leg4 = True
        swing_leg4 = False
    if decrease_p4:
        j4 -= 1
    else:
        j4 += 1
    # semicricle path
    
    if swing_leg3:
        y3 = -(radius**2 - (j3-x_0)**2)**0.5 + y_height
        inverse_kinematics(leg_3,j3,L1, y3)
    elif stence_leg3:
        inverse_kinematics(leg_3,j3,L1,y_height)
    if swing_leg1:
        y1 = -(radius**2 - (j1-x_0)**2)**0.5 + y_height
        inverse_kinematics(leg_1,j1,L1, y1)
    elif stence_leg1:
        inverse_kinematics(leg_1,j1,L1, y_height)
    if swing_leg2:
        y2 = -(radius**2 - (j2-x_0)**2)**0.5 + y_height
        inverse_kinematics(leg_2,-j2,L1,y2)
    elif stence_leg2:
        inverse_kinematics(leg_2,-j2,L1,y_height)
    if swing_leg4:
        y4 = -(radius**2 - (j4-x_0)**2)**0.5 + y_height
        inverse_kinematics(leg_4,-j4,L1, y4)
    elif stence_leg4:
        inverse_kinematics(leg_4,-j4,L1, y_height)

    plot = 1 # plots trajectory of the legs
    if plot == 0:
        # duration of tracing in seconds
        plot_time = 2
        # leg 1 tracking
        link_state = p.getLinkState(robotId, pata_1_frame)
        link_position = link_state[0]  # World position of the link
        path_l1.append(link_position)
        if len(path_l1) > 1:
            p.addUserDebugPoints(path_l1, pointColorsRGB=[[0, 1, 0]]*len(path_l1), pointSize=5, lifeTime=plot_time)
            path_l1 = []
        # leg 2 tracking
        link_state = p.getLinkState(robotId, pata_2_frame)
        link_position = link_state[0]
        path_l2.append(link_position)
        if len(path_l2) > 1:
            p.addUserDebugPoints(path_l2, pointColorsRGB=[[0, 1, 0]]*len(path_l2), pointSize=5, lifeTime=plot_time)
            path_l2 = []
        # leg 3 tracking
        link_state = p.getLinkState(robotId, pata_3_frame)
        link_position = link_state[0]
        path_l3.append(link_position)
        if len(path_l3) > 1:
            p.addUserDebugPoints(path_l3, pointColorsRGB=[[0, 1, 0]]*len(path_l3), pointSize=5, lifeTime=plot_time)
            path_l3 = []
        # leg 4 tracking
        link_state = p.getLinkState(robotId, pata_4_frame)
        link_position = link_state[0]
        path_l4.append(link_position)
        if len(path_l4) > 1:
            p.addUserDebugPoints(path_l4, pointColorsRGB=[[0, 1, 0]]*len(path_l4), pointSize=5, lifeTime=plot_time)
            path_l4 = []
    # Step the simulation
    p.stepSimulation()
    time.sleep(1/240.0)  # Simulate in real-time

# Disconnect from the physics server
p.disconnect()
