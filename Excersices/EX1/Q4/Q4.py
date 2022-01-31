import csv
import numpy as np
import math

# Constants ======================================
R = 20.5/1000
L = 26/1000

K_RO = 3/1000
K_ALPHA = 8/1000
K_BETA = -1.5/1000

coefficients = [[R/2, R/2], [R/(2*L), -R/(2*L)]]
# Constants ======================================

# DEFINITIONS =====================================================
def calculate_new_cordinates(x_dot, y_dot, omega, current_x, current_y, current_theta, dt):
    return current_x + x_dot*dt, current_y + y_dot*dt, current_theta + omega*dt

def calculate_I_values(v, omega, theta):
    return v * math.cos(theta), v * math.sin(theta), omega


def calculate_v_omega(ro, alpha, beta):
    return K_RO * ro, K_ALPHA * alpha + K_BETA * beta

def between_pi_and_neg_pi(theta):
    while(theta < -math.pi):
        theta += 2 * math.pi
    while(theta > math.pi):
        theta -= 2 * math.pi
    return theta

def calculate_polar(delta_x, delta_y, theta):
    ro = math.sqrt((delta_x * delta_x) + (delta_y * delta_y))

    alpha = -theta + math.atan2(delta_y, delta_x)
    alpha = between_pi_and_neg_pi(alpha)

    beta = -theta - alpha
    beta = between_pi_and_neg_pi(beta)

    return ro, alpha, beta

def differnce(current_x, current_y ,desiered_x, desiered_y):
  return math.sqrt((desiered_x - current_x) * (desiered_x - current_x) + (desiered_y - current_y) * (desiered_y - current_y))

def calc_deltas(current_x, current_y ,desiered_x, desiered_y):
  return desiered_x - current_x, desiered_y - current_y

def differential_robot_controller(starting_x, starting_y, desiered_x, desiered_y, initial_theta, dt, threshold = 0.0001):
    Ys = []
    Xs = []
    THETAs = []
    
    theta = initial_theta
    current_x = starting_x
    current_y = starting_y

    while differnce(current_x, current_y ,desiered_x, desiered_y) > threshold:
        del_x, del_y = calc_deltas(current_x, current_y ,desiered_x, desiered_y)

        ro, alpha, beta = calculate_polar(del_x, del_y, theta)

        v, omega = calculate_v_omega(ro, alpha, beta)

        x_dot, y_dot, theta_dot = calculate_I_values(v, omega, theta)

        current_x, current_y, theta = calculate_new_cordinates(x_dot, y_dot, theta_dot, current_x, current_y, theta, dt)

        Ys.append(current_y)
        Xs.append(current_x)
        THETAs.append(theta)

    return Xs, Ys, THETAs
# DEFINITIONS =====================================================


# Initial theta is PI/2 and robot is in (0, 0)
import matplotlib.pyplot as plt
figure(figsize=(6, 6), dpi=80)

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, 10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, -10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, 10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, -10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 0, 10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 0, -10/1000, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, 0, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, 0, math.pi/2, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')



# Initial theta is 0 and robot is not in (0, 0)
import matplotlib.pyplot as plt

figure(figsize=(6, 6), dpi=80)

xx,yy,_ = differential_robot_controller(10/1000, 10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(-10/1000, -10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(-10/1000, 10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(10/1000, -10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, -10/1000, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(10/1000, 0, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(-10/1000, 0, 0, 0, 0, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')



# Initial theta is PI and robot is in (0, 0)
import matplotlib.pyplot as plt

figure(figsize=(6, 6), dpi=80)

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, 10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, -10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, 10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, -10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 0, 10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 0, -10/1000, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, 10/1000, 0, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')

xx,yy,_ = differential_robot_controller(0, 0, -10/1000, 0, math.pi, 0.01)
plt.plot(xx, yy, color='blue')
plt.plot([xx[0], xx[len(xx)-1]], [yy[0], yy[len(yy)-1]], 'ro-')