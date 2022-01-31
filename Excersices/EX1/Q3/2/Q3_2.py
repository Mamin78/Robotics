from controller import Robot, Motor, Compass, GPS
import math
import numpy as np

# Definitions
def get_bearing_in_degrees():
  north = compass.getValues()
  # print('noth is: ', north)
  rad = math.atan2(north[0], north[2])
  bearing = (rad - 1.5708) / math.pi * 180.0
  if (bearing < 0.0):
    bearing = bearing + 360.0
  return bearing

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28

# File
f = open("Q3_2.csv", "w")
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
RADIUS = rr = 0.5
L = 0.52

lm=0.025
rm=0.1
leftMotor.setVelocity(0.05 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

i = 0

while robot.step(TIME_STEP) != -1:
  gps_xyz = gps.getValues()
  deg = get_bearing_in_degrees()
  i += 1
  if i > 180:
      i = 0
      lm+=0.05
      rm+=0.05 
      leftMotor.setVelocity(lm * MAX_SPEED)
      rightMotor.setVelocity(rm * MAX_SPEED)

  f.write(str(gps_xyz[0]) + ', '+str(gps_xyz[1]) + ', '+str(gps_xyz[2]) + ', ' + str(deg))
  f.write("\n")
f.close()
   