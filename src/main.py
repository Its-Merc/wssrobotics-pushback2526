# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       daniel                                                       #
# 	Created:      11/24/2025, 3:33:16 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
# type: ignore
from vex import (
    Brain,
    Competition,
    wait,
    MSEC,
    CurrentUnits,
    AiVision,
    DriveTrain,
    Motor,
    MotorGroup,
    RIGHT,
    LEFT,
    DEGREES,
    AiVisionObject,
)

# from typing import List, Dict, Any
import urandom

import math


class EnumObj:
    def __init__(self, name: str, value):
        self.name: str = name
        self.as_int = value


BLUE = EnumObj("BLUE", 0)
RED = EnumObj("RED", 1)


class Side:
    def __init__(self, color: int):
        self.color = BLUE.name if color == BLUE.as_int else RED.name


# wait for rotation sensor to fully initialize
wait(30, MSEC)


br = Brain()
scr = br.screen  # 480x240 px screen


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = (
        br.battery.voltage(MV)
        + br.battery.current(CurrentUnits.AMP) * 100
        + br.timer.system_high_res()
    )
    urandom.seed(int(random))


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)


# takes a snapshot with the vision sensor and picks the first block
# then it uses the block's x position and trigonometry to find how many degrees to turn the robot toward the block
def align_to_block(block_index: int):
    global drivetrain
    objs = vision.take_snapshot(AiVision.ALL_AIOBJS)
    obj = objs[block_index]

    # 320x240 screen
    # relative position to the center
    scr_x = obj.centerX - 160

    # find the angle with inverse tangent
    angle_to_turn = math.degrees(math.atan2(1, scr_x))

    # turn left or right according to +/-
    direction = RIGHT if angle_to_turn > 0 else LEFT
    drivetrain.turn_for(direction, abs(angle_to_turn), DEGREES)


def grabbable_blocks(
    objs: list, grabbable_size: int, x_range: int = 35, min_y: int = 145
) -> bool:
    # if any obj in the detected objs is grabbable, return true

    for obj in objs:
        assert isinstance(obj, AiVisionObject)
        # if the obj is over a certain size
        # and the x position is close to the center of the screen
        # and the y position is close to the intake
        # it is grabbable
        if (
            obj.width >= grabbable_size
            and obj.height >= grabbable_size
            and obj.centerX > 160 - x_range
            and obj.centerX < 160 + x_range
            and obj.centerY > min_y
        ):
            return True
    return False


def on_auto():
    global blue
    print("Autonomous Mode Started")
    br.screen.clear_screen()
    br.screen.set_fill_color(blue)
    br.screen.print("Autonomous Mode Started")


def on_usr_control():
    global red
    print("Driver Control")
    br.screen.clear_screen()
    br.screen.set_fill_color(red)
    br.screen.print("Driver Control")

    while True:
        wait(20, MSEC)


# create competition instance
comp = Competition(on_usr_control, on_auto)

# setup drivetrain
m1 = Motor(1)
m2 = Motor(2)
m3 = Motor(3)
m4 = Motor(4)
mg_left = MotorGroup(m1, m3)
mg_right = MotorGroup(m2, m4)
drivetrain = DriveTrain(mg_left, mg_right)

# vision sensor
vision = AiVision(0, AiVision.ALL_AIOBJS)

red = 0x00FF0000
blue = 0x000000FF

# \/ on program starts \/
initializeRandomSeed()

scr.clear_screen()
scr.print("Starting...")

# rocket
scr.draw_circle(200, 120, 25, 0x00F0000)
scr.draw_circle(280, 120, 25, 0x00F0000)
scr.draw_rectangle(220, 40, 40, 80, 0x00F0000)

while True:
    # get detected objects
    objs = vision.take_snapshot(AiVision.ALL_AIOBJS)

    # clear the screen
    scr.clear_screen()

    # enumerate does not work apparently so...
    for i in range(len(objs)):
        obj = objs[i]

        # color of the block
        color = Side(obj.id)

        # debug stuff
        scr.set_cursor((4 * i) + 1, 1)

        scr.print(obj.centerX, obj.centerY, sep=", ")
        scr.next_row()
        scr.print(obj.originX, obj.originY, sep=", ")
        scr.next_row()
        scr.print(color.color)  # 0 = blue, 1 = red
        scr.next_row()
        scr.print(obj.score)

        scr.draw_rectangle(
            obj.originX,
            obj.originY,
            obj.width,
            obj.height,
            (
                (blue if color.color == BLUE.name else red)
                | max(0, min(255, math.ceil((1 - (obj.score / 100)) * 255))) << 24
            ),
        )

    # vision sensor bounds and center point
    scr.draw_pixel(160, 120)
    scr.draw_line(0, 0, 320, 0)
    scr.draw_line(0, 240, 320, 240)
    scr.draw_line(0, 0, 0, 240)
    scr.draw_line(320, 0, 320, 240)

    wait(50, MSEC)
