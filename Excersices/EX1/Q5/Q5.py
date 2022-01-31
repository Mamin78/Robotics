from controller import Robot, Motor, GPS, Compass
import csv
import math
import numpy as np

# Constants ======================================
TIME_STEP = 64
MAX_SPEED = 6.28

R = 20.5/1000
L = 52/1000

DESIERED_X = 0.1
DESIERED_Y = 0.1

K_RO = 3/1000
K_ALPHA = 8/1000
K_BETA = -1.5/1000

coefficients = [[R/2, R/2], [R/(2*L), -R/(2*L)]]
# Constants ======================================

f = open("Q5_1.csv", "w")
f.write("X,Y,Z,Degree")
f.write("\n")

# DEFINITIONS =====================================================
def R_mat_creator(theta):
    R = np.zeros((3, 3))
    R[0, 0] = R[1, 1] = math.cos(theta)
    R[2, 2] = 1
    R[0, 1] = math.sin(theta)
    R[1, 0] = -math.sin(theta)
    return R

def calculate_phis(v, omega, theta,l, r):
    I = np.zeros((3, 1))
    I[0, 0] = v * math.cos(math.radians(theta))
    I[1, 0] = v * math.sin(math.radians(theta))
    I[2, 0] = omega

    R = R_mat_creator(math.radians(theta))
    T = R.dot(I)

    tt = np.zeros((2, 1))
    tt[0, 0] = T[0, 0]
    tt[1, 0] = T[2, 0]

    coefficients = np.array([[r / 2, r / 2], [r / 2 * l, -r / 2 * l]])
    answer = np.linalg.solve(coefficients, tt)

    return answer[0][0], answer[1][0]

def calculate_v_omega(ro, alpha, beta):
    return K_RO * ro, K_ALPHA * alpha + K_BETA * beta

def between_pi_and_neg_pi(theta):
    while(theta < -math.pi):
        theta += 2*math.pi
    while(theta > math.pi):
        theta -= 2*math.pi
    return theta

def calculate_polar(delta_x, delta_y, theta):
    ro = math.sqrt((delta_x * delta_x) + (delta_y * delta_y))

    alpha = -math.radians(theta) + math.atan2(delta_y, delta_x)
    alpha = between_pi_and_neg_pi(alpha)

    beta = -math.radians(theta) - math.radians(alpha)
    beta = between_pi_and_neg_pi(beta)

    return ro, alpha, beta

def get_bearing_in_degrees():
    north = compass.getValues()
    rad = math.atan2(north[0], north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    bearing = 180 - bearing
    if bearing < 0.0:
        bearing += 360.0
    if bearing > 360.0:
        bearing -= 360.0
    return bearing

def differential_robot_controller():
    position = gps.getValues()
    theta = get_bearing_in_degrees()

    ro, alpha, beta = calculate_polar(DESIERED_X - position[0], DESIERED_Y - position[1], theta)

    v, omega = calculate_v_omega(ro, alpha, beta)

    phi1_dot, phi2_dot = calculate_phis(v, omega, theta, L, R)

    rightMotor.setVelocity(phi1_dot)
    leftMotor.setVelocity(phi2_dot)

    f.write(str(position[0]) + ', '+str(position[1]) + ', '+str(position[2]) + ', ' + str(theta))
    f.write("\n")

# DEFINITIONS =====================================================

# create the Robot instance.
robot = Robot()
gps = GPS('gps')
compass = Compass('compass')
gps.enable(1)
compass.enable(1)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


while robot.step(TIME_STEP) != -1:
    differential_robot_controller()
f.close()