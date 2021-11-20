#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.robotics import *
from pybricks.media.ev3dev import *
import math

#####################################################################################
#DEKLARACE ZAŘÍZENÍ                                                                 #
ev3 = EV3Brick()                                                                    #
motorA = Motor(Port.A)                                                              #
motorD = Motor(Port.D)                                                              #
motorB = Motor(Port.B)                                                              #
left_motor = motorA                                                                 #
right_motor = motorD                                                                #
height_motor = motorB                                                               #
                                                                                    #
timer = StopWatch()                                                                 #
                                                                                    #
gyro = GyroSensor(Port.S1)                                                          #        
ultra = UltrasonicSensor(Port.S4)                                                   #
                                                                                    #
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)       #
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)        #
######################################################################################


#########################
#DEKLARACE PROMĚNNÝCH   #
offset = 0              #
lastOffset = 0          #
speed = 0               #
lastSpeed = 0           #
accel = 0               #
lastAccel = 0           #
jerk = 0                #
angle = 0               #
time_elapsed = 0        #
timeDelta = 0           #
motorSpeed = 0          #
                        #
td = 10                 #
                        #
lastUpdateTime = 0      #
#########################

#######################################################################################
#VARIABLES SETTINGS                                                                   #
MAX_OFFSET=20                       #maximální ochylka od středu v mm                 #
MIN_EFFECTIVE_DISTANCE = 50         #minimální vzdálenost vozítka od senzoru v mm     #
MAX_EFFECTIVE_DISTANCE = 190        #maximální vzdálenost vozítka od senzoru v mm     #
ANGLE_PERCENT = 100                 #koeficient odchylky úhly                         #
OFFSET_PERCENT = 20                 #koeficient odchylky vzdálenosti                  #
MAX_ANG_VELOC = 200/180*math.pi     #maximální úhlová rychlost                        #
#######################################################################################

gyro.reset_angle(0)

def clamp(x, minX, maxX): #OŘEZÁ HODNOTY
    return max(min(x, maxX), minX)

while True:
    #robot.drive(-150, 0)
    #wait(10)
    #robot.stop()
    wait(td)
    measureOffset = ultra.distance()-140

    lastOffset = offset
    lastSpeed = speed
    lastAccel = accel

    offset = measureOffset
    speed = (offset - lastOffset) / td
    accel = (speed - lastSpeed) / td
    jerk = (accel - lastAccel) / td

    angle = gyro.angle()

    #if(speed != 0):
    #print(offset, speed, accel, jerk)

    futureOffset = offset + td*speed + 1/2*td*td*accel
    
    if((abs(angle) > 1/180*math.pi or abs(offset) > MAX_OFFSET) and (ultra.distance() > MIN_EFFECTIVE_DISTANCE and ultra.distance() < MAX_EFFECTIVE_DISTANCE)):
        motorSpeed = (-(ANGLE_PERCENT*angle + OFFSET_PERCENT*futureOffset))
    else:
        motorSpeed = 0
    motorB.run(motorSpeed)