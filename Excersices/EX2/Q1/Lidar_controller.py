from controller import Robot, Motor, Compass, GPS, Lidar, Camera, DistanceSensor
import math

# Constants ======================================
TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

lidar = Lidar('lidar')
lidar.enable(1)

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
# set up the motor speeds at 10% of the MAX_SPEED.

leftMotor.setVelocity(0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

lidar.enablePointCloud()

while robot.step(TIME_STEP) != -1:
    points = lidar.getLayerRangeImage(0)
    print(points)
    print(len(points))