from controller import Robot, Motor, Compass, GPS, Lidar, Camera, DistanceSensor
import math

# Constants ======================================
TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

camera = Camera('camera_1')
camera.enable(1)

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
# set up the motor speeds at 10% of the MAX_SPEED.

leftMotor.setVelocity(0.5 * MAX_SPEED)
rightMotor.setVelocity(0.5 * MAX_SPEED)

i = 0
while robot.step(TIME_STEP) != -1:
    i += 1
    if i >= 70:
        camera.saveImage(str(i)+'.png', 100)