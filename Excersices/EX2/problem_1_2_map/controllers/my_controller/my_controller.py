from controller import Robot, Motor, Compass, GPS, Lidar, Camera, DistanceSensor
import math

# Constants ======================================
TIME_STEP = 64
MAX_SPEED = 6.28


def get_bearing_in_degrees():
    north = compass.getValues()
    rad = math.atan2(north[0], north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    bearing = 180 - bearing
    if bearing < 0.0:
        bearing = bearing + 360.0
    if bearing > 360.0:
        bearing = bearing - 360.0
    return bearing


# create the Robot instance.
robot = Robot()

compass = Compass('compass')
compass.enable(1)


ds1 = DistanceSensor('ds1')
ds1.enable(1)

ds2 = DistanceSensor('ds2')
ds2.enable(1)

ds3 = DistanceSensor('ds3')
ds3.enable(1)

ds4 = DistanceSensor('ds4')
ds4.enable(1)

ds5 = DistanceSensor('ds5')
ds5.enable(1)

ds6 = DistanceSensor('ds3')
ds6.enable(1)

ds7 = DistanceSensor('ds7')
ds7.enable(1)

ds8 = DistanceSensor('ds8')
ds8.enable(1)
# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
# set up the motor speeds at 10% of the MAX_SPEED.

leftMotor.setVelocity(-0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

data = {}
data['ds1'] = []
data['ds2'] = []
data['ds3'] = []
data['ds4'] = []
data['ds5'] = []
data['ds6'] = []
data['ds7'] = []
data['ds8'] = []

init = 0

i = 0
j = 0
while robot.step(TIME_STEP) != -1:
    if i == 0:
        init = get_bearing_in_degrees()
        i += 1

    deg = get_bearing_in_degrees()

    if abs(deg - init) > 1.4:
        j += 1
        init = deg
        print("Next Step!", init)
        print(ds1.getValue())
        print(ds2.getValue())
        print(ds3.getValue())
        print(ds4.getValue())
        print(ds5.getValue())
        print(ds6.getValue())
        print(ds7.getValue())
        print(ds8.getValue())

        data['ds1'].append(ds1.getValue())
        data['ds2'].append(ds2.getValue())
        data['ds3'].append(ds3.getValue())
        data['ds4'].append(ds4.getValue())
        data['ds5'].append(ds5.getValue())
        data['ds6'].append(ds6.getValue())
        data['ds7'].append(ds7.getValue())
        data['ds8'].append(ds8.getValue())
        
        print()
    if j >= 32:
        break
   
for i in range(1, 9):
    f = open("ds"+str(i)+'.txt', "w")
    f.write(str(data['ds'+str(i)]))
    f.close()
