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
    FORWARD,
    REVERSE,
    Inertial,
    SmartDrive,
    GearSetting,
    Controller,
    Thread,
)

# from typing import List, Dict, Any
import urandom
import math

# constants
IDLE_TURN_SPEED = 8
IDLE_TURN_DIR = RIGHT

GRAB_SIZE_REQ = 35
GRAB_X_RANGE = 35
GRAB_Y_POS = 145

CTRLER_DEADZONE = 5

# wait for rotation sensor to fully initialize
wait(30, MSEC)


br = Brain()
scr = br.screen  # 480x240 px screen

# setup drivetrain
# from the auto generator
left_drive_smart = Motor(0, GearSetting.RATIO_18_1, False)
right_drive_smart = Motor(1, GearSetting.RATIO_18_1, True)
inertial_sensor = Inertial(2)
drivetrain = SmartDrive(
    left_drive_smart, right_drive_smart, inertial_sensor, 319.19, 320, 40, MM, 1
)

# controller
ctrler = Controller(PRIMARY)
ctrler_l_dead = False
ctrler_r_dead = False


# Make random actually random
# from the auto generator
def initialize_random_seed():
    wait(100, MSEC)
    random = (
        br.battery.voltage(MV)
        + br.battery.current(CurrentUnits.AMP) * 100
        + br.timer.system_high_res()
    )
    urandom.seed(int(random))


# from the auto generator
def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)


drivetrain_calibrated = False


# from the auto generator
def calibrate_drivetrain():
    # global drivetrain_is_calibrated, scr, inertial_sensor
    # Calibrate the Drivetrain Inertial
    global drivetrain_calibrated
    sleep(200, MSEC)
    scr.print("Calibrating")
    scr.next_row()
    scr.print("Inertial")
    inertial_sensor.calibrate()
    while inertial_sensor.is_calibrating():
        sleep(25, MSEC)
    drivetrain_calibrated = True
    scr.clear_screen()
    scr.set_cursor(1, 1)


# from the auto generator
# define a task that will handle monitoring inputs from ctrler
def ctrler_loop():
    global ctrler_l_dead, ctrler_r_dead, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if inertial_sensor.is_calibrating():
                left_drive_smart.stop()
                right_drive_smart.stop()
                while inertial_sensor.is_calibrating():
                    sleep(25, MSEC)

            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3
            # right = axis2
            drivetrain_left_side_speed = ctrler.axis3.position()
            drivetrain_right_side_speed = ctrler.axis2.position()

            # check if the value is inside of the deadband range
            if (
                drivetrain_left_side_speed < CTRLER_DEADZONE
                and drivetrain_left_side_speed > -CTRLER_DEADZONE
            ):
                # check if the left motor has already been stopped
                if ctrler_l_dead:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    ctrler_l_dead = True
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                ctrler_l_dead = False
            # check if the value is inside of the deadband range
            if (
                drivetrain_right_side_speed < CTRLER_DEADZONE
                and drivetrain_right_side_speed > -CTRLER_DEADZONE
            ):
                # check if the right motor has already been stopped
                if ctrler_r_dead:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    ctrler_r_dead = True
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                ctrler_r_dead = False

            # only tell the left drive motor to spin if the values are not in the deadband range
            if not ctrler_l_dead:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if not ctrler_r_dead:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)


# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(ctrler_loop)

# vision sensor
vision = AiVision(3, AiVision.ALL_AIOBJS)

# other stuff
intake = Motor(4)

red = 0x00FF0000
blue = 0x000000FF


class Side:
    def __init__(self, color: int):
        self.color = BLUE.name if color == BLUE.as_int else RED.name


class EnumObj:
    def __init__(self, name: str, value):
        self.name: str = name
        self.as_int = value


BLUE = EnumObj("BLUE", 0)
RED = EnumObj("RED", 1)

# 0 = blue, 1 = red
TEAM = RED


# takes a snapshot with the vision sensor and picks the first block
# then it uses the block's x position and trigonometry to find how many degrees to turn the robot toward the block
def align_to_block(block_xpos: int):
    global drivetrain

    # 320x240 screen
    # relative position to the center
    scr_x = block_xpos - 160

    # find the angle with inverse tangent
    angle_to_turn = math.degrees(math.atan2(1, scr_x))

    # turn left or right according to +/-
    direction = RIGHT if angle_to_turn > 0 else LEFT
    drivetrain.turn_for(direction, abs(angle_to_turn), DEGREES)


def detecting_grabbables(
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

# \/ -------------------------- on program starts -------------------------- \/
initialize_random_seed()
calibrate_drivetrain()

scr.clear_screen()
scr.print("Starting...")

# rocket
scr.draw_circle(200, 120, 25, 0x00F0000)
scr.draw_circle(280, 120, 25, 0x00F0000)
scr.draw_rectangle(220, 40, 40, 80, 0x00F0000)

while True:
    # clear the screen
    scr.clear_screen()

    # get detected objs
    objs = vision.take_snapshot(AiVision.ALL_AIOBJS)

    # get objs on own team colour
    target_objs = [obj for obj in objs if obj.id == TEAM.as_int]

    # TODO should probably update this later
    target_obj = target_objs[0] if len(target_objs) > 0 else obj

    # if anything we want to grab is detected
    if len(target_objs) > 0:
        # align to the first one
        align_to_block(target_obj.centerX)
    else:  # otherwise do something else
        drivetrain.turn_for(IDLE_TURN_DIR, IDLE_TURN_SPEED, DEGREES)

    # if we can grab something, grab it
    if detecting_grabbables(target_objs, GRAB_SIZE_REQ, GRAB_X_RANGE, GRAB_Y_POS):
        # spin the intake motor and stop moving
        intake.spin(FORWARD)
        # drivetrain.stop()
    # if we can grab something but wrong team, dont
    elif detecting_grabbables(objs, GRAB_SIZE_REQ, GRAB_X_RANGE, GRAB_Y_POS):
        # TODO considering changing this
        intake.spin(REVERSE)
        # drivetrain.drive(REVERSE)
    else:
        intake.stop()

    # drive
    drivetrain.drive(FORWARD)

    # debug stuff
    # enumerate does not work apparently so...
    for i in range(len(objs)):
        obj = objs[i]

        # color of the block
        color = Side(obj.id)

        # set cursor for info
        scr.set_cursor((4 * i) + 1, 1)

        # info stuff
        scr.print(obj.centerX, obj.centerY, sep=", ")
        scr.next_row()
        scr.print(obj.originX, obj.originY, sep=", ")
        scr.next_row()
        scr.print(color.color)  # 0 = blue, 1 = red
        scr.next_row()
        scr.print(obj.score)

        # draw rectangle around the object
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

        # highlight the target object
        if obj == target_obj:
            scr.draw_rectangle(
                obj.centerX - 2,
                obj.centerY - 2,
                obj.width + 4,
                obj.height + 4,
                0x00FFFFFF,
            )

    # vision sensor bounds and center point
    scr.draw_pixel(160, 120)
    scr.draw_line(0, 0, 320, 0)
    scr.draw_line(0, 240, 320, 240)
    scr.draw_line(0, 0, 0, 240)
    scr.draw_line(320, 0, 320, 240)

    wait(50, MSEC)
