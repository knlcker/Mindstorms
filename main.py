#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()
# Initialize the motors.
arm_motor = Motor(Port.A)
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
claw_motor = Motor(Port.D)

distance_sensor = UltrasonicSensor(Port.S1)
color_sensor = ColorSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
turn_rate = 1
DRIVE_SPEED = 200
DRIVE_SPEED2 = -75
distance = 0
PROPORTIONAL_GAIN = 2

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

def drive_forewards(input):
    robot.straight(-input)

def drive_backwards(input):
    robot.straight(input)

def turn_left(input):
    robot.turn(input)

def turn_right(input):
    robot.turn(-input)

def pick_up():
     claw_motor.run_time(-1000, 3000)

def put_down():
    claw_motor.run_time(1000, 3000)

def pick_up_item_and_put_down():
    pick_up()
    drive_backwards(200)
    turn_left(200)
    drive_forewards(200)
    put_down()
    drive_backwards(200)
    turn_right(200)
    drive_forewards(200)

def press_button():
    arm_motor.run_time(180, 500)
    arm_motor.run_time(-185, 500)

def follow_line():
    deviation = (color_sensor.reflection() - threshold)
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(DRIVE_SPEED2, turn_rate)
    # wait(10)


while True:
    while distance_sensor.distance() > 70:
        follow_line()
    else:
        ev3.speaker.beep()
        robot.drive(0, 0)
        press_button()
        pick_up_item_and_put_down()

    #follow_line()
    #object_detection()

    # drive_forewards(100)
    # pick_up_item_and_put_down()
    # drive_backwards(100)
    # press_button()
    