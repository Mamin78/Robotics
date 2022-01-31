from controller import Robot, Motor, Compass, GPS
import math
import numpy as np

# Constants ======================================
TIME_STEP = 64
MAX_SPEED = 6.28

L = 52
R = 20.5
# Constants ======================================

# Definitions
def get_bearing_in_degrees():
    north = compass.getValues()
    # print('noth is: ', north)
    rad = math.atan2(north[0], north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if (bearing < 0.0):
      bearing = bearing + 360.0
    return bearing

def R_mat_creator(theta):
    R = np.zeros((3, 3))
    R[0, 0] = R[1, 1] = math.cos(theta)
    R[2, 2] = 1
    R[0, 1] = math.sin(theta)
    R[1, 0] = -math.sin(theta)
    return R

def inverse_kinematic(velocity, angular_velocity, l, r, theta):
    I = np.zeros((3, 1))
    I[0, 0] = velocity[0]
    I[1, 0] = velocity[1]
    I[2, 0] = angular_velocity

    R = R_mat_creator(theta * (math.pi / 180))
    T = R.dot(I)

    tt = np.zeros((2, 1))
    tt[0, 0] = T[0, 0]
    tt[1, 0] = T[2, 0]

    coefficients = np.array([[r / 2, r / 2], [r / 2 * l, -r / 2 * l]])
    answer = np.linalg.solve(coefficients, tt)

    return answer[0][0], answer[1][0]

# File
f = open("Q2_1.csv", "w")
f.write("X,Y,Z,Degree")
f.write("\n")

# create the Robot instance.
robot = Robot()
gps = GPS('gps')
gps.enable(1)
# Compass
compass = Compass('compass')
compass.enable(1)

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
# set up the motor speeds at 10% of the MAX_SPEED.

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

i = 0 
while robot.step(TIME_STEP) != -1:
    gps_xyz = gps.getValues()

    deg = get_bearing_in_degrees()
    if i == 0:
        phi1, phi2 = inverse_kinematic([1, 1], 0, 1, 1, deg)
        i+=1

        leftMotor.setVelocity(phi1)
        rightMotor.setVelocity(phi2)
    
    f.write(str(gps_xyz[0]) + ', '+str(gps_xyz[1]) + ', '+str(gps_xyz[2]) + ', ' + str(deg))
    f.write("\n")
f.close()
