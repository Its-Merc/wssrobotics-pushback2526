# imports ================================================================

# type: ignore
from vex import Controller
from vex import *

import urandom, math

# end imports ============================================================
# classes ================================================================


class Side:
    def __init__(self, color: int):
        self.color = BLUE_ENUM.name if color == BLUE_ENUM.as_int else RED_ENUM.name


class EnumObj:
    def __init__(self, name: str, value):
        self.name: str = name
        self.as_int = value


# end classes ============================================================
# constants ==============================================================

# colors -------------------------
colour_red = 0x00FF0000
colour_blue = 0x000000FF

BLUE_ENUM = EnumObj("BLUE", 0)
RED_ENUM = EnumObj("RED", 1)

# 0 = blue, 1 = red
TEAM = BLUE_ENUM

# motor settings -----------------

MOTOR_DEADZONE = 2  # set_motor_speed will not set the motor speed if the requested speed is less than this
SORTING_MOTOR_VEL = 100
RAMP_MOTOR_VEL = 100

# end motor settings -------------
# drivetrain/ctrler settings -----

DRIVETRAIN_TURN_THRESHOLD = 0.5  # how many degrees off the turning can be

MIN_THROTTLE_INPUT_NORM = (
    0.25  # the minimum normalized value the throttle joystick can have
)
MIN_TURN_INPUT_NORM = 0.25  # the minimum normalized value the turn joystick can have

# t in lerp for throttle
# higher values give less interpolation
# lower values gives more interpolation
THROTTLE_SMOOTHING = 0.08

# adds t in lerp depending how much throttle is applied
# bigger t = less interpolation
# allows for acceleration that doesn't take ages, while keeping a slow deceleration
THROTTLE_SMOOTHING_ADD = 0.15

# t in lerp for throttleturning
# give less interpolation
# lower values gives more interpolation
TURN_SMOOTHING = 0.5

# end drivetrain/ctrler settings -
# ramp settings ------------------

RAMP_POSITIONS = [120, -5, 25]  # the angles at which the front "ramp" will turn to
RAMP_DIR_OFFSET = 0  # kinda useless ngl

# note: attach arm 3rd space down from gear for ramp

# end ramp settings --------------
# sorting settings ---------------

# delays for the auto sorting
SORT_SWITCH_OFF_DELAY = 500  # ms
SORT_SWITCH_ON_DELAY = 25  # ms

# see the sort delay code
# essentially, the ratio of:
SORT_WAIT_RATIO = (
    5  # how much to delay the system figuring out what sort mode to choose
)
SORT_DELAY_RATIO = 4  # how much to delay the act of actually switching the sort mode

# end sorting settings -----------
# ai settings --------------------

# useless
IDLE_TURN_DEGREES = 1
IDLE_TURN_DIR = RIGHT

# settings for aligning with a block
ALIGN_MAX_ANGLE = 90  # during autonomous
ALIGN_MAX_ANGLE_DRIVE = 80  # during driver control
ALIGN_DEADZONE = 2  # essentially the driver turn threshold for aligning

# for auto grabbing
GRAB_SIZE_REQ = 35  # checks if the ball is bigger than this
GRAB_X_RANGE = 35  # checks if the ball is between (-x, x)
GRAB_Y_POS = 145  # checks if the ball is lower than that y pos

# end ai settings ----------------
# ports --------------------------

# port 1 -> 0

# 2, 1 front, 12, 11, back
DRIVETRAIN_PORTS = [1, 0, 11, 10]  # front motors, back motors, l -> r

MOTOR_INTAKE_PORT = 3
MOTOR_SCORING_PORT = 4
MOTOR_SORTING_PORT = 5
MOTOR_RAMP_PORT = 6

AI_SENSOR_PORT = 8
INERTIAL_SENSOR_PORT = 9
OPTICAL_SENSOR_PORT = 7
DIST_SENSOR_PORT = 12

# end ports ----------------------
# end constants ==========================================================

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# initialization =========================================================

br = Brain()
scr = br.screen  # 480x240 px screen, 240, 120 is the center

# drivetrain setup ---------------

motor_l_a = Motor(DRIVETRAIN_PORTS[0], GearSetting.RATIO_18_1, False)
motor_l_b = Motor(DRIVETRAIN_PORTS[2], GearSetting.RATIO_18_1, False)
motorgroup_l = MotorGroup(motor_l_a, motor_l_b)

motor_r_a = Motor(DRIVETRAIN_PORTS[1], GearSetting.RATIO_18_1, True)
motor_r_b = Motor(DRIVETRAIN_PORTS[3], GearSetting.RATIO_18_1, True)
motorgroup_r = MotorGroup(motor_r_a, motor_r_b)

inertial_sensor = Inertial(INERTIAL_SENSOR_PORT)

drivetrain = SmartDrive(
    motorgroup_l, motorgroup_r, inertial_sensor, 319.19, 320, 40, MM, 1
)

drivetrain.set_turn_threshold(DRIVETRAIN_TURN_THRESHOLD)
drivetrain.set_stopping(BrakeType.HOLD)  # coasting kinda sucks

drivetrain_calibrated = False


# from the auto generator on the vex v5 website
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


# end drivetrain setup -----------
# ctrler setup -------------------

ctrler = Controller(PRIMARY)

# a bunch of aliases for ease of use

# the 4 joystick axes
ctrler_r_horiz = ctrler.axis1
ctrler_r_vert = ctrler.axis2
ctrler_l_vert = ctrler.axis3
ctrler_l_horiz = ctrler.axis4

# scoring motor ctrls
ctrler_btn_scoring_up_hold = ctrler.buttonL1
ctrler_btn_scoring_down_hold = ctrler.buttonL2

# sorting overrides
ctrler_btn_sorting_out_toggle = ctrler.buttonX  # 1
ctrler_btn_sorting_in_toggle = ctrler.buttonB  # 2
ctrler_btn_sorting_no_toggle = ctrler.buttonA  # 0
sorting_override_btn_mode = -1  # current override mode

# intake motor ctrls
ctrler_btn_intake_in = ctrler.buttonR2
ctrler_btn_intake_out = ctrler.buttonR1

# ramp ctrls
ctrler_btn_ramp_high = ctrler.buttonUp
ctrler_btn_ramp_med = ctrler.buttonLeft
ctrler_btn_ramp_low = ctrler.buttonDown
ctrler_btn_ramp_in = ctrler.buttonUp  # debugging purposes
ctrler_btn_ramp_out = ctrler.buttonDown  # debugging purposes

# hold this button to use ai sensor for stuff during driving mode
ctrler_btn_ai_hold = ctrler.buttonY

ctrler_control_enabled = True  # flag for whether ctrler should be usable

# end ctrler setup ---------------
# sensor setup -------------------

vision = AiVision(AI_SENSOR_PORT, AiVision.ALL_AIOBJS)
optical = Optical(OPTICAL_SENSOR_PORT)
dist = Distance(DIST_SENSOR_PORT)

optical.set_light_power(20, PERCENT)


def on_collision():
    print("!!! collision detected !!!")


inertial_sensor.collision(on_collision)

# end sensor setup ---------------
# motors setup -------------------

motor_intake = Motor(MOTOR_INTAKE_PORT)
motor_scoring = Motor(MOTOR_SCORING_PORT)
motor_sorting = Motor(MOTOR_SORTING_PORT)
motor_ramp = Motor(MOTOR_RAMP_PORT)

motor_sorting.set_velocity(SORTING_MOTOR_VEL, PERCENT)
motor_ramp.set_velocity(RAMP_MOTOR_VEL, PERCENT)

# end motors setup ----------------
# bumper setup --------------------

bumper_1 = Bumper(br.three_wire_port.a)  # currently useless

# end bumper setup ---------------
# sorting setup -------------------

sorting_timer = Timer()
sorting_timer.reset()
current_sort = 0  # 0 = none, 1 = team, 2 = other

# waiting until the "signal" can be sent
sorting_ratio_wait = SORT_WAIT_RATIO / (SORT_WAIT_RATIO + SORT_DELAY_RATIO)
# how long until the "signal" is "received"
sorting_ratio_delay = SORT_DELAY_RATIO / (SORT_WAIT_RATIO + SORT_DELAY_RATIO)


# 0 = stop, 1 = out, 2 = in
def set_sort(new_sort: int):
    global motor_sorting, current_sort
    if new_sort == current_sort:
        return
    if new_sort == 0:
        motor_sorting.stop()
    elif new_sort == 2:
        motor_sorting.spin(FORWARD)
    elif new_sort == 1:
        motor_sorting.spin(REVERSE)
    current_sort = new_sort


# end sorting setup --------------
# misc funcs ---------------------


# from the auto generator on the vex v5 website
# Make random actually random
def initialize_random_seed():
    wait(100, MSEC)
    random = (
        br.battery.voltage(MV)
        + br.battery.current(CurrentUnits.AMP) * 100
        + br.timer.system_high_res()
    )
    urandom.seed(int(random))


def lerp(a: float, b: float, t: float) -> float:
    return a - (a - b) * t


# gets the min of two ints based on their absolute values
def min_neg(a: int, b: int) -> int:
    return math.copysign(1, b) * math.copysign(min(abs(a), abs(b)), a)


# gets the max of two ints based on their absolute values
def max_neg(a: int, b: int) -> int:
    return math.copysign(1, b) * math.copysign(max(abs(a), abs(b)), a)


# end misc funcs -----------------
# ctrler funcs -------------------


def toggle_sorting_btn(mode: int):
    global sorting_override_btn_mode
    if mode == sorting_override_btn_mode:
        sorting_override_btn_mode = -1
        return
    sorting_override_btn_mode = mode


def switch_ramp_btns(pos: int = 0):
    motor_ramp.spin_to_position(RAMP_POSITIONS[pos] + RAMP_DIR_OFFSET, DEGREES, False)


# 0 = stop, 1 = out, 2 = in
ctrler_btn_sorting_out_toggle.pressed(toggle_sorting_btn, [1])
ctrler_btn_sorting_in_toggle.pressed(toggle_sorting_btn, [2])
ctrler_btn_sorting_no_toggle.pressed(toggle_sorting_btn, [0])

ctrler_btn_ramp_high.pressed(switch_ramp_btns, [0])
ctrler_btn_ramp_med.pressed(switch_ramp_btns, [2])
ctrler_btn_ramp_low.pressed(switch_ramp_btns, [1])


# turning
def horiz_stick_func(x: int):
    if x == 0:
        return 0

    # x cant be less than y
    # abs x
    # cubed x
    x_prime = math.copysign(max(abs(x), 0.25) ** 3, x)
    return x_prime


# throttle
def vert_stick_func(x: int):
    if x == 0:
        return 0

    # x cant be less than y
    # abs x
    # cubed x
    x_prime = math.copysign(max(abs(x), 0.25) ** 3, x)
    return x_prime


# func for aligning
def ai_driver_align_func(x: int):
    if x == 0:
        return 0

    # abs x
    # quadratic x
    x_prime = math.copysign(abs(x) ** 2, x)
    return x_prime


def apply_function_on_stick(pos: int, max: int, func):
    norm = pos / max
    changed_norm = func(norm)
    return round(changed_norm * max)


# end ctrler funcs ---------------
# ai setup -----------------------

ai_objs = tuple()  # any objs
target_objs = list()  # objs that are of same team colour
target_obj = AiVisionObject()  # the first obj in target objs

distance: DistanceUnits = 0


def get_ai_objs():
    global ai_objs, target_objs, target_obj, distance
    ai_objs = vision.take_snapshot(AiVision.ALL_AIOBJS)

    # get objs on own team colour
    target_objs = [obj for obj in ai_objs if obj.id == TEAM.as_int]

    # TODO should probably update this later
    target_obj = target_objs[0] if len(target_objs) > 0 else ai_objs[0]

    distance = dist.object_distance(DistanceUnits.MM) if dist.installed() else 360


# Thread(get_ai_objs)


# debug stuff for ai sensor
def ai_sensor_debug(objs: tuple[AiVisionObject], target_obj: AiVisionObject):
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
                (colour_blue if color.color == BLUE_ENUM.name else colour_red)
                | max(0, min(255, math.ceil((1 - (obj.score / 100)) * 255))) << 24
            ),
        )

        # highlight the target object
        if obj == target_obj:
            scr.draw_rectangle(
                obj.originX - 2,
                obj.originY - 2,
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


# takes a snapshot with the vision sensor and picks the first block
# then it uses the block's x position and trigonometry to find how many degrees to turn the robot toward the block
def get_align_to_blocks(blocks: list[AiVisionObject], max_angle=ALIGN_MAX_ANGLE) -> int:
    global drivetrain, distance
    # print("attempting align")

    if len(blocks) <= 0:
        return 0

    xpos: int = 0
    if len(blocks) == 1:
        xpos = blocks[0].centerX
    else:
        blocks.sort(key=lambda block: (block.width * block.height))
        xpos = blocks[0]

    # 320x240 screen
    # relative position to the center
    scr_x = xpos - 160

    # find the angle with inverse tangent
    # tan = opp/adj, opp = src_x, adj = dist to obj
    # 100 as approx distance for now
    angle_to_turn = math.degrees(math.atan2(scr_x, distance))
    if abs(angle_to_turn) > max_angle or abs(angle_to_turn) < ALIGN_DEADZONE:
        return 0
    # print(angle_to_turn)

    # turn left or right according to +/-
    # direction = RIGHT if angle_to_turn > 0 else LEFT
    # print(
    #     str(angle_to_turn)
    #     + " degrees in direction "
    #     + str(direction)
    #     + ", with dist "
    #     + str(distance)
    # )
    return angle_to_turn
    # Thread(drivetrain.turn_for, [direction, abs(angle_to_turn), DEGREES, 40, PERCENT])


# brokey
def go_to_block():
    global drivetrain, distance
    print("attempting going to block")

    # sin(theta) = o/h
    # o = h * sin(theta)


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


# uses optical sensor hue to detect red vs blue for sorting
def optical_red_or_blue():
    global optical
    hue = optical.hue()
    # print(hue)
    if hue < 23 or hue > 340:
        return RED_ENUM.name
    elif hue > 50 and hue < 340:
        return BLUE_ENUM.name
    return None


# end ai setup -------------------
# comp setup ---------------------


def on_auto():
    global ai_objs, target_objs, target_obj, current_sort, ctrler_control_enabled
    print("Autonomous Mode Started")
    br.screen.clear_screen()
    br.screen.print("Autonomous Mode Started")

    ctrler_control_enabled = False

    # calibrating stuff

    while not drivetrain_calibrated:
        calibrate_drivetrain()

    if inertial_sensor.is_calibrating():
        motorgroup_l.stop()
        motorgroup_r.stop()
        while inertial_sensor.is_calibrating():
            sleep(25, MSEC)

    # ramp in up pos
    switch_ramp_btns(0)

    current_sort = 1

    turn_vel = 20
    throttle_vel = 20

    go_loader_dist_horiz = 30
    parking_zone_dist = 17
    loader_dist = parking_zone_dist - 3.5
    go_back_vert_offset = 4.5
    go_back_extra = 6

    # drive to loader (exclusively right currently)
    # 1. drive forward to go past the parking zone
    # 2. turn 90 degrees to loader
    # 3. drive to loader
    # 4. turn to face loader
    # 5. drive a bit closer

    # drivetrain.drive_for(FORWARD, 36, INCHES, 50, PERCENT, False)
    drivetrain.drive_for(FORWARD, parking_zone_dist, INCHES, throttle_vel, PERCENT)
    # wait(500, MSEC)
    drivetrain.turn_to_rotation(90, DEGREES, turn_vel, PERCENT)
    # wait(500, MSEC)
    drivetrain.drive_for(FORWARD, go_loader_dist_horiz, INCHES, throttle_vel, PERCENT)
    # wait(500, MSEC)
    drivetrain.turn_to_rotation(180, DEGREES, turn_vel - 5, PERCENT)
    wait(500, MSEC)
    drivetrain.drive_for(FORWARD, 6, INCHES, 60, PERCENT)

    # find the loader blocks
    turn_degrees = 0
    timer = Timer()
    while timer.value() < 2 and turn_degrees == 0:
        # print("fokeofsokfesioesosoi")
        get_ai_objs()
        br.screen.clear_screen()
        ai_sensor_debug(ai_objs, target_obj)
        if len(target_objs) > 0:
            turn_degrees = get_align_to_blocks(target_objs)

        wait(20, MSEC)

    direction = RIGHT if turn_degrees > 0 else LEFT
    drivetrain.turn_for(direction, turn_degrees, DEGREES, turn_vel, PERCENT)
    wait(500, MSEC)

    # spin intake motor to suck up
    motor_intake.spin(REVERSE, 100, PERCENT)

    # ramp in down pos
    switch_ramp_btns(1)
    wait(500, MSEC)

    drivetrain.drive_for(FORWARD, loader_dist, INCHES, throttle_vel, PERCENT)
    wait(500, MSEC)

    # rattle em
    drivetrain.drive(FORWARD, 100, PERCENT)

    wait(500, MSEC)

    # ramp in slightly up pos
    switch_ramp_btns(2)

    timer = Timer()
    while timer.value() < 2.5:
        drivetrain.drive(FORWARD, 100, PERCENT)
        wait(200, MSEC)
        drivetrain.drive(REVERSE, 100, PERCENT)
        wait(130, MSEC)

    # reverse back to original position
    drivetrain.drive_for(
        REVERSE, loader_dist - go_back_vert_offset, INCHES, throttle_vel, PERCENT
    )
    wait(500, MSEC)

    switch_ramp_btns(0)
    # drivetrain.turn_to_rotation(0, DEGREES, 30, PERCENT)
    # wait(500, MSEC)

    # go back to bottom center goal

    drivetrain.turn_for(RIGHT, 135, DEGREES, turn_vel, PERCENT)
    drivetrain.turn_to_heading(315, DEGREES, turn_vel, PERCENT)
    # wait(500, MSEC)

    # pythagorean theorem
    go_back_dist = math.sqrt((go_loader_dist_horiz**2) * 2)

    # turn_degrees = 0
    # timer = Timer()
    # while timer.value() < 2 and turn_degrees == 0:
    #     # print("fokeofsokfesioesosoi")
    #     get_ai_objs()
    #     br.screen.clear_screen()
    #     ai_sensor_debug(ai_objs, target_obj)
    #     if len(target_objs) > 0:
    #         turn_degrees = get_align_to_blocks(target_objs)

    #     wait(20, MSEC)

    # direction = RIGHT if turn_degrees > 0 else LEFT
    # drivetrain.turn_for(direction, turn_degrees, DEGREES, turn_vel, PERCENT)
    # wait(500, MSEC)

    drivetrain.drive_for(FORWARD, go_back_dist, INCHES, throttle_vel, PERCENT)
    wait(500, MSEC)
    motor_intake.spin(FORWARD, 100, PERCENT)
    drivetrain.drive_for(FORWARD, go_back_extra, INCHES, 20, PERCENT)
    wait(1000, MSEC)

    # should arrive at center goal by now

    # motor_intake.spin(FORWARD, 100, PERCENT)

    # drivetrain.turn_to_rotation(135, DEGREES, 100, PERCENT)
    # drivetrain.drive_for(FORWARD, 6, INCHES, 60, PERCENT)
    # drivetrain.turn_to_rotation(180, DEGREES, 100, PERCENT)

    # wait(1000, MSEC)

    # drivetrain.turn_to_rotation(0, DEGREES, 100, PERCENT)
    # drivetrain.drive_for(FORWARD, 6, INCHES, 60, PERCENT)

    motor_intake.stop()
    drivetrain.stop()

    current_sort = 0


def on_usr_control():
    global motor_intake, motor_scoring, ctrler_control_enabled, current_sort, sorting_override_btn_mode

    print("Driver Control")
    br.screen.clear_screen()
    br.screen.print("Driver Control")

    ctrler_control_enabled = True

    while not drivetrain_calibrated:
        calibrate_drivetrain()

    def set_motor_speed(motor: Motor | MotorGroup, speed: int):
        if abs(speed) < MOTOR_DEADZONE:
            motor.stop()
            return
        motor.set_velocity(speed, PERCENT)
        motor.spin(FORWARD)

    last_d_l_left = 0
    last_d_r_horiz = 0

    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while comp.is_driver_control():
        wait(20, MSEC)
        if not ctrler_control_enabled:
            continue
        # calibrate stuff
        if inertial_sensor.is_calibrating():
            motorgroup_l.stop()
            motorgroup_r.stop()
            while inertial_sensor.is_calibrating():
                sleep(25, MSEC)
            motor_ramp.set_position(120, DEGREES)

        # calculate the drivetrain motor velocities from the controller joystick axies
        # left = axis3
        # right = axis2

        # turning input
        new_d_r_horiz = apply_function_on_stick(
            ctrler_r_horiz.position(), 100, horiz_stick_func
        )

        # ai align turning override
        get_ai_objs()
        if ctrler_btn_ai_hold.pressing() and len(target_objs) > 0:
            angle = get_align_to_blocks([target_obj], ALIGN_MAX_ANGLE_DRIVE)
            normalized = (angle / ALIGN_MAX_ANGLE_DRIVE) * 100
            new_d_r_horiz = apply_function_on_stick(
                normalized, 100, ai_driver_align_func
            )
            # doesnt use the direct drivetrain turning because it doesnt allow for moving forward at the same time

        # d_r_horiz = new_d_r_horiz
        d_r_horiz = lerp(last_d_r_horiz, new_d_r_horiz, TURN_SMOOTHING)  # interpolate

        # throttle input
        new_d_l_vert = apply_function_on_stick(
            ctrler_l_vert.position(), 100, vert_stick_func
        )

        # d_l_vert = new_d_l_vert
        d_l_vert = lerp(
            last_d_l_left,
            new_d_l_vert,
            THROTTLE_SMOOTHING + (THROTTLE_SMOOTHING_ADD * abs(new_d_l_vert / 100)),
        )  # interpolate

        # print("turning: " + str(d_r_horiz))
        # print("throttle: " + str(d_l_vert))

        # round to the closest whole number that is closer to zero
        d_l_vert_round = math.copysign(math.floor(abs(d_l_vert)), d_l_vert)
        d_r_horiz_round = math.copysign(math.floor(abs(d_r_horiz)), d_r_horiz)

        # caps the velocities to magnitude 100
        l_drivetrain_vel = min_neg(d_l_vert_round + d_r_horiz_round, 100)
        r_drivetrain_vel = min_neg(d_l_vert_round - d_r_horiz_round, 100)

        # print("left motorgroup vel: " + str(l_drivetrain_vel))
        # print("right motorgroup vel: " + str(r_drivetrain_vel))
        # print()

        set_motor_speed(motorgroup_l, l_drivetrain_vel)
        set_motor_speed(motorgroup_r, r_drivetrain_vel)

        # intake in -> 100
        # intake out -> -100
        # no input -> 0
        set_motor_speed(
            motor_intake,
            100 * (ctrler_btn_intake_in.pressing() - ctrler_btn_intake_out.pressing()),
        )

        # scoring up -> -100
        # scoring down -> 100
        # no input -> 0
        set_motor_speed(
            motor_scoring,
            -100
            * (
                ctrler_btn_scoring_up_hold.pressing()
                - ctrler_btn_scoring_down_hold.pressing()
            ),
        )
        # set_motor_speed(
        #     motor_ramp,
        #     -100 * (ctrler_btn_ramp_in.pressing() - ctrler_btn_ramp_out.pressing()),
        # )

        # no override -> whatever its supposed to be
        # override 0 -> 0
        # override 1 -> 100
        # override 2 -> -100
        if current_sort == 0:
            set_motor_speed(
                motor_sorting,
                100
                * (
                    (
                        (sorting_override_btn_mode == 1)
                        - (sorting_override_btn_mode == 2)
                    )
                ),
            )

        last_d_l_left = d_l_vert
        last_d_r_horiz = d_r_horiz


def unit_testing():
    print("Unit tests start")

    # testing ramp settings
    print("Unit test 1")
    switch_ramp_btns(0)
    wait(2500, MSEC)
    switch_ramp_btns(1)
    wait(2500, MSEC)
    switch_ramp_btns(2)
    wait(2500, MSEC)

    print("Unit tests done")


# create competition instance
comp = Competition(on_usr_control, on_auto)

# end comp setup -----------------
# end initialization =====================================================

# ------------------------------------------------------------------------
# program start ==========================================================
# ------------------------------------------------------------------------

initialize_random_seed()
calibrate_drivetrain()

scr.clear_screen()
scr.print("Starting...")

# rocket drawing!
scr.draw_circle(200, 120, 25, 0x00F0000)
scr.draw_circle(280, 120, 25, 0x00F0000)
scr.draw_rectangle(220, 40, 40, 80, 0x00F0000)

# assume the motor is at position 120 at start
motor_ramp.set_position(120, DEGREES)

# unit_testing()

while True:
    # print(optical.hue())
    if not comp.is_autonomous():  # gives autonomous control over sorting currently
        detect = optical_red_or_blue() or "NONE"
    # scr.print("optical detecting: " + detect)
    # scr.next_row()

    # scr.print("current sort: " + str(current_sort))
    # scr.next_row()
    # scr.print("sorting timer: " + str(sorting_timer.time(MSEC)))
    # scr.next_row()

    # print("optical detecting: " + detect)
    # print("current sort: " + str(current_sort))
    # print("sorting timer: " + str(sorting_timer.time(MSEC)))
    # print()

    # sorting logic past here

    # override on means current sort is 0
    if sorting_override_btn_mode != -1:
        current_sort = 0
        scr.print("sort override: " + str(sorting_override_btn_mode))
        wait(20, MSEC)  # do the delay
        continue

    # if block is enemy's, take them and store them
    # else don't
    new_sort = 0
    if detect == TEAM.name:
        new_sort = 2
    elif detect != "NONE":
        new_sort = 1
    else:
        new_sort = 0

    # print("new sort: " + str(new_sort))

    if comp.is_autonomous():
        new_sort = 2

    if new_sort != current_sort:
        # if the new_sort is nothing and the current sort is 1 or 2, i.e. sorting, check for the switch off delay
        # if the new_sort is not nothing, check for the sort switch on delay

        if (
            new_sort != 0
            and sorting_timer.time(MSEC) >= SORT_SWITCH_ON_DELAY * sorting_ratio_wait
        ):
            sorting_timer.reset()
            sorting_timer.event(
                set_sort, round(SORT_SWITCH_ON_DELAY * sorting_ratio_delay), [new_sort]
            )
        elif (
            current_sort != 0
            and new_sort == 0
            and sorting_timer.time(MSEC) >= SORT_SWITCH_OFF_DELAY * sorting_ratio_wait
        ):
            sorting_timer.reset()
            sorting_timer.event(
                set_sort, round(SORT_SWITCH_OFF_DELAY * sorting_ratio_delay), [new_sort]
            )

    wait(20, MSEC)
