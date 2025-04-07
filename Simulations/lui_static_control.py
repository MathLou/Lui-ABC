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
C = 150.76 # mm
L = 81.72 # mm

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
            target_position = -m.pi/2 +target_position
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

def initial_pose(x,y,z):
    # initial position
    inverse_kinematics(leg_1, x,z,y)
    inverse_kinematics(leg_2, x,z,y)
    inverse_kinematics(leg_3, x,z,y)
    inverse_kinematics(leg_4, x,z,y)
def transformations(leg_idx,x,y,z,roll,yaw, pitch, x_center, y_center):
    """ accepts leg index, current coordinates and rotation euler angles, with x and y center offsets
        returns: x,y,z transformed coordinates """
    # rotation matrix
    R_roll = np.array([[1, 0, 0, 0],[0, m.cos(roll), -m.sin(roll), 0],[0, m.sin(roll), m.cos(roll), 0],[0, 0, 0, 1]])
    R_yaw = np.array([[m.cos(yaw), -m.sin(yaw), 0, 0],[m.sin(yaw), m.cos(yaw), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    R_pitch = np.array([[m.cos(pitch), 0, m.sin(pitch), 0],[0, 1, 0, 0],[-m.sin(pitch), 0, m.cos(pitch), 0],[0, 0, 0, 1]])
    Rotation = R_roll @ R_yaw @ R_pitch
    T_leg = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    # translation matrix CORRIGIR
    if leg_idx == 1:
        T_leg = np.array([[1, 0, 0, C/2],[0, 1, 0, L/2],[0, 0, 1, 0],[0, 0, 0, 1]])
    elif leg_idx == 2:
        T_leg = np.array([[1, 0, 0, C/2],[0, -1, 0, -L/2],[0, 0, 1, 0],[0, 0, 0, 1]])
    elif leg_idx == 3:
        T_leg = np.array([[-1, 0, 0, -C/2],[0, 1, 0, L/2],[0, 0, 1, 0],[0, 0, 0, 1]])
    elif leg_idx == 4:
        T_leg = np.array([[-1, 0, 0, -C/2],[0, -1, 0, -L/2],[0, 0, 1, 0],[0, 0, 0, 1]])
    T_center = np.array([[1, 0, 0, x_center],[0, 1, 0, y_center],[0, 0, 1, 0],[0, 0, 0, 1]])
    # final transformation matrix
    coords = np.array([x,y,z,1])
    transformed_coords = Rotation @ T_center @ T_leg @ coords.T
    # print("HERE {} {} {}".format(transformed_coords[0] - C/2, -transformed_coords[1] + L/2, transformed_coords[2]))
    return transformed_coords[0], transformed_coords[1], transformed_coords[2]

# Connect to the physics server
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version

# Set additional search path for PyBullet data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load a plane URDF
planeId = p.loadURDF("plane.urdf")

# Load plane object
movPlane = p.loadURDF("movable_plane.urdf", [0, 0, 0.20],[0,0,0,1], useFixedBase=True)

# Load a robot
robotId = p.loadURDF("robot.urdf", [-0.025, -0.225, 0.30], useFixedBase=False) # normal "free" mode
# robotId = p.loadURDF("robot.urdf", [0, -0.25, 0.25], useFixedBase=True) # debug fixed mode
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
# initial position
y_0 = 75
x_0 = 0
z_0 = L1
roll = m.radians(0)
yaw = m.radians(0)
plane_roll = 10
plane_pitch = 0
pitch = m.radians(0)
x_center = 0
y_center = 0
decrease = 0
decrease2 = 0
decrease3 = 0
time_mcu = time.time()
# control constants
time_pid = time.time()
roll_frame = 0
pitch_frame = 0
yaw_frame = 0
roll_offset = 90
yaw_offset = 90
error_roll_sum = 0
error_roll_prev = 0
error_pitch_sum = 0
error_pitch_prev = 0
# Run the simulation
initial_pose(x_0, y_0, z_0) # initialize in the initial position
rollId = p.addUserDebugParameter("roll_slider", -15, 15, 0)
pitchId = p.addUserDebugParameter("pitch_slider",-15,15,0)
leitura_mcu = [0,0,0]
while True:
    plane_roll = p.readUserDebugParameter(rollId)
    plane_pitch = p.readUserDebugParameter(pitchId)
    centro_state = p.getLinkState(robotId, centro_frame)
    # ler conforme um certo periodo de 50 ms
    #simulando sensor mcu6050
    if time.time() - time_mcu >= 0.05:
        time_mcu = time.time()
        # leitura do sensor
        pitch_frame, roll_frame, yaw_frame = p.getEulerFromQuaternion(centro_state[1])
        #making random signal from min to max (odelar com sensor real)
        np.random.seed(0)
        min = -10
        max = 10
        roll_frame = 180/m.pi*(roll_frame) + np.random.uniform(min,max)
        pitch_frame = 180/m.pi*(pitch_frame) + np.random.uniform(min,max)
        yaw_frame = 180/m.pi*(yaw_frame) + np.random.uniform(min,max)
        print('roll: {}, pitch: {}, yaw: {}'.format(roll_frame, pitch_frame, yaw_offset+ yaw_frame))
        print("--------------------")

    # PID pose control every 30ms
    if time.time() - time_pid >= 0.3:
        time_pid = time.time()
        # PID control
        # roll
        p_roll = 0.01
        i_roll = 0.0001
        d_roll = 0.0001
        # pitch
        p_pitch = 0.01
        i_pitch = 0.0001
        d_pitch = 0.0001
        error_roll = -roll_frame
        error_pitch = pitch_frame -90
        #print("error roll: {}".format(error_roll))
        u_roll = p_roll*error_roll+ i_roll*error_roll_sum+ d_roll*(error_roll - error_roll_prev)
        u_pitch = p_pitch*error_pitch+ i_pitch*error_pitch_sum+ d_pitch*(error_pitch - error_pitch_prev)
        #print('u_roll: {}'.format(u_roll))
        #print('roll input: {}'.format(u_roll))
        roll = u_roll #attach controller to leg
        pitch = u_pitch
        error_roll_prev = error_roll
        error_pitch_prev = error_pitch
        error_roll_sum += error_roll_prev
        error_pitch_sum += error_pitch_prev
        

    orientation = p.getQuaternionFromEuler([m.radians(plane_pitch),m.radians(plane_roll), 0])
    p.resetBasePositionAndOrientation(movPlane, [0, 0, 0.20], orientation)
    leg_1_coords = transformations(1, x_0, z_0,y_0, roll, yaw, pitch, x_center, y_center)
    inverse_kinematics(leg_1, leg_1_coords[0] - C/2, leg_1_coords[1]-L/2, leg_1_coords[2])
    leg_2_coords = transformations(2, x_0,z_0,y_0, roll, yaw, pitch, x_center, y_center)
    inverse_kinematics(leg_2, leg_2_coords[0] -C/2, -1*(leg_2_coords[1]+L/2), leg_2_coords[2])
    leg_3_coords = transformations(3, x_0,z_0,y_0, roll, yaw, pitch, x_center, y_center)
    inverse_kinematics(leg_3, leg_3_coords[0] +C/2, leg_3_coords[1]-L/2, leg_3_coords[2])
    leg_4_coords = transformations(4, x_0,z_0,y_0, roll, yaw, pitch, x_center, y_center)
    inverse_kinematics(leg_4, leg_4_coords[0] +C/2, -(leg_4_coords[1]+L/2), leg_4_coords[2])

    # Step the simulation
    p.stepSimulation()
    time.sleep(1/240.0)  # Simulate in real-time

# Disconnect from the physics server
p.disconnect()
