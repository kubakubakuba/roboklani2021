#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.tools import *
from pybricks.robotics import *
from pybricks.media.ev3dev import *
import math
import sys

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
color = ColorSensor(Port.S3)
touch = TouchSensor(Port.S2)
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
lastJerk = 0
snap = 0
angle = 0               #
time_elapsed = 0        #
timeDelta = 0           #
motorSpeed = 0          #
switchWays = 0
started = 0
haltMove = False
                        #
td = 1                 #
dtd = 0
                        #
lastUpdateTime = 0      #
turned = 0

moveSpeed = 180
steerVal = 2
#########################

#######################################################################################
#VARIABLES SETTINGS                                                                   #
MAX_OFFSET=20                       #maximální ochylka od středu v mm                 #
MIN_EFFECTIVE_DISTANCE = 50         #minimální vzdálenost vozítka od senzoru v mm     #
MAX_EFFECTIVE_DISTANCE = 190        #maximální vzdálenost vozítka od senzoru v mm     #
ANGLE_PERCENT = 100                 #koeficient odchylky úhlu                         #
OFFSET_PERCENT = 15               #koeficient odchylky vzdálenosti                  #
MAX_ANG_VELOC = 200/180*math.pi     #maximální úhlová rychlost                        #
#######################################################################################

gyro.reset_angle(0)

def clamp(x, minX, maxX): #OŘEZÁ HODNOTY
    return max(min(x, maxX), minX)

while (touch.pressed() == False):
    print('ready')
    wait(10)

print('pressed')
wait(1000)
while True:
    if(touch.pressed()):
        sys.exit()

    wait(td)
    measureOffset = ultra.distance()-125

    lastOffset = offset
    lastSpeed = speed
    lastAccel = accel
    lastJerk = jerk

    offset = measureOffset
    speed = (offset - lastOffset) / td
    accel = (speed - lastSpeed) / td
    jerk = (accel - lastAccel) / td
    snap = (jerk - lastJerk) / td

    angle = gyro.angle()

    futureOffset = offset + td*speed + 1/2*td*td*accel + 1/6*td*td*td*jerk
    
    if((abs(angle) > 1/180*math.pi or abs(futureOffset) > MAX_OFFSET) and (ultra.distance() > MIN_EFFECTIVE_DISTANCE and ultra.distance() < MAX_EFFECTIVE_DISTANCE)):
        motorSpeed = (-(ANGLE_PERCENT*angle + OFFSET_PERCENT*futureOffset))
    else:
        motorSpeed = 0

    motorB.run(motorSpeed)

    motorA.run(moveSpeed)
    motorD.run(moveSpeed + steerVal)

    dtd = dtd + 1
    started = started + 1

    lastColor1 = color.rgb()

    if(((lastColor1[0] == 21 or lastColor1[0] == 22 or lastColor1[0] == 20) and lastColor1[2] <= 7) and (turned == 0) and (dtd>85) and (started > 200)):

        moveSpeed = 0 #COMPLETELY STOP MOVEMENT
        print("ZASTAVENO!")
        sys.exit()

        switchWays = 1
        dtd = 0
        turned = 1
        print("OTOČENO!") #REVERSE

    if((color.color() == Color.YELLOW) and (turned == 2)):
        haltMove = True
        dtd = 0 #WAIT FOR STOP

    if(dtd >= 500 and turned == 1):
        turned = 2 #SEND FRO REVERSAL

    if(dtd >= 40 and turned == 1 and switchWays == 1):
        moveSpeed = -moveSpeed
        steerVal = -steerVal
        switchWays = 0
        dtd = 0         #REVERSE MOVEMENT

    if(haltMove and (dtd>=50)):
        #moveSpeed = 0 #COMPLETELY STOP MOVEMENT
        #print("ZASTAVENO!")
        #sys.exit()
        moveSpeed = -moveSpeed
        steerVal = -steerVal
        dtd = 0
        turned = 0
        haltMove = False
        started = 0 
        switchWays = 0