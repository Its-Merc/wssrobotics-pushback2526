# imports ================================================================

# type: ignore
from vex import Controller
from vex import *

import urandom, math

# end imports ============================================================
# classes ================================================================


class Side:
    def __init__(self, color: int):
        self.color = BLUE.name if color == BLUE.as_int else RED.name


class EnumObj:
    def __init__(self, name: str, value):
        self.name: str = name
        self.as_int = value


# end classes ============================================================
# constants ==============================================================

# colors -------------------------
red = 0x00FF0000
blue = 0x000000FF

BLUE = EnumObj("BLUE", 0)
RED = EnumObj("RED", 1)

# 0 = blue, 1 = red
TEAM = RED

# drivetrain settings ------------

DRIVETRAIN_DEADZONE = 2
DRIVETRAIN_TURN_THRESHOLD = 2
THROTTLE_SMOOTHING = 0.1
TURN_SMOOTHING = 0.2

# end drivetrain settings --------
# ramp settings ------------------

RAMP_POSITIONS = [90, -43, -20]
RAMP_DIR_OFFSET = 0

# end ramp settings --------------
# sorting settings ---------------

SORT_SWITCH_DELAY = 400  # ms

# end sorting settings -----------
# ai settings --------------------

IDLE_TURN_DEGREES = 1
IDLE_TURN_DIR = RIGHT

ALIGN_DEADZONE = 2

GRAB_SIZE_REQ = 35
GRAB_X_RANGE = 35
GRAB_Y_POS = 145

# end ai settings ----------------
# ports --------------------------

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
scr = br.screen  # 480x240 px screen

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
drivetrain.set_stopping(BrakeType.BRAKE)

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


# end drivetrain setup -----------
# ctrler setup -------------------

ctrler = Controller(PRIMARY)
ctrler_r_horiz = ctrler.axis1
ctrler_r_vert = ctrler.axis2
ctrler_l_vert = ctrler.axis3
ctrler_l_horiz = ctrler.axis4

ctrler_l_dead = False
ctrler_r_dead = False

ctrler_btn_scoring_up_hold = ctrler.buttonL1
ctrler_btn_scoring_down_hold = ctrler.buttonL2

ctrler_btn_sorting_out_toggle = ctrler.buttonX  # 1
ctrler_btn_sorting_in_toggle = ctrler.buttonB  # 2
ctrler_btn_sorting_no_toggle = ctrler.buttonA  # 0
sorting_override_btn_mode = -1

ctrler_btn_intake_in = ctrler.buttonR2
ctrler_btn_intake_out = ctrler.buttonR1

ctrler_btn_ramp_high = ctrler.buttonUp
ctrler_btn_ramp_med = ctrler.buttonLeft
ctrler_btn_ramp_low = ctrler.buttonDown

ctrler_btn_ai_toggle = ctrler.buttonY
ai_toggled = False

ctrler_control_enabled = True

# end ctrler setup ---------------
# sensor setup -------------------

vision = AiVision(AI_SENSOR_PORT, AiVision.ALL_AIOBJS)
optical = Optical(OPTICAL_SENSOR_PORT)
dist = Distance(DIST_SENSOR_PORT)

optical.set_light_power(100, PERCENT)


def on_collision():
    print("!!! collision detected !!!")


inertial_sensor.collision(on_collision)

# end sensor setup ---------------
# motors setup -------------------

motor_intake = Motor(MOTOR_INTAKE_PORT)
motor_scoring = Motor(MOTOR_SCORING_PORT)
motor_sorting = Motor(MOTOR_SORTING_PORT)
motor_ramp = Motor(MOTOR_RAMP_PORT)

motor_sorting.set_velocity(100, PERCENT)

# end motors setup ----------------
# sorting setup -------------------

sorting_timer = Timer()
sorting_timer.reset()
current_sort = 0  # 0 = none, 1 = team, 2 = other


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


def lerp(a: float, b: float, t: float) -> float:
    return a - (a - b) * t


def min_neg(a: int, b: int) -> int:
    return math.copysign(1, b) * math.copysign(min(abs(a), abs(b)), a)


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


def on_ai_toggle():
    global ai_toggled, drivetrain, motor_intake
    # stop drivetrain and intake when toggling ai
    ai_toggled = not ai_toggled
    drivetrain.stop()
    motor_intake.stop()

    print("AI Disabled" if ai_toggled else "AI Enabled")


ctrler_btn_sorting_out_toggle.pressed(toggle_sorting_btn, [1])
ctrler_btn_sorting_in_toggle.pressed(toggle_sorting_btn, [2])
ctrler_btn_sorting_no_toggle.pressed(toggle_sorting_btn, [0])

ctrler_btn_ramp_high.pressed(switch_ramp_btns, [0])
ctrler_btn_ramp_med.pressed(switch_ramp_btns, [2])
ctrler_btn_ramp_low.pressed(switch_ramp_btns, [1])

ctrler_btn_ai_toggle.pressed(on_ai_toggle)


def stick_func(x: int):
    if x == 0:
        return 0
    squared = math.copysign(max(abs(x), 0.25) ** 2, x)
    return squared


def apply_function_on_stick(pos: int, max: int, func):
    norm = pos / max
    changed_norm = func(norm)
    return round(changed_norm * max)


# end ctrler funcs ---------------
# ai setup -----------------------

ai_objs = tuple()
target_objs = list()
target_obj = AiVisionObject()

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
                (blue if color.color == BLUE.name else red)
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
def align_to_block(block_xpos: int):
    global drivetrain, distance
    print("attempting align")

    # 320x240 screen
    # relative position to the center
    scr_x = block_xpos - 160

    # find the angle with inverse tangent
    # tan = opp/adj, opp = src_x, adj = dist to obj
    # 100 as approx distance for now
    angle_to_turn = math.degrees(math.atan2(scr_x, distance))
    if abs(angle_to_turn) > 90 or abs(angle_to_turn) < ALIGN_DEADZONE:
        return
    # print(angle_to_turn)

    # turn left or right according to +/-
    direction = RIGHT if angle_to_turn > 0 else LEFT
    print(
        str(angle_to_turn)
        + " degrees in direction "
        + str(direction)
        + ", with dist "
        + str(distance)
    )
    Thread(drivetrain.turn_for, [direction, abs(angle_to_turn), DEGREES, 40, PERCENT])


def go_to_block():
    global drivetrain, distance
    print("attempting going to block")

    drivetrain.drive(FORWARD, 100, PERCENT)
    while target_obj.centerY >= 120:
        get_ai_objs()
    drivetrain.stop()

    # sin(theta) = o/h
    # o = h * sin(theta)
    act_dist = distance * math.sin(math.radians(45))

    drivetrain.drive_for(FORWARD, act_dist, DistanceUnits.MM, 100, PERCENT)


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


def optical_red_or_blue():
    global optical
    hue = optical.hue()
    if hue < 27 or hue > 340:
        return RED.name
    elif hue > 50 and hue < 340:
        return BLUE.name
    return None


# end ai setup -------------------
# comp setup ---------------------


def on_auto():
    global ai_objs, target_objs, target_obj
    print("Autonomous Mode Started")
    br.screen.clear_screen()
    br.screen.print("Autonomous Mode Started")

    while comp.is_autonomous():
        get_ai_objs()
        br.screen.clear_screen()
        ai_sensor_debug(ai_objs, target_obj)
        if len(target_objs) > 0:
            # align to the target
            align_to_block(target_obj.centerX)
            # go_to_block()
        wait(20, MSEC)


def on_usr_control():
    global ctrler_l_dead, ctrler_r_dead, motor_intake, motor_scoring, ctrler_control_enabled, current_sort, sorting_override_btn_mode

    print("Driver Control")
    br.screen.clear_screen()
    br.screen.print("Driver Control")

    def set_motor_speed(motor: Motor | MotorGroup, speed: int):
        if abs(speed) < DRIVETRAIN_DEADZONE:
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
        # stop the motors if the brain is calibrating
        if inertial_sensor.is_calibrating():
            motorgroup_l.stop()
            motorgroup_r.stop()
            while inertial_sensor.is_calibrating():
                sleep(25, MSEC)
            motor_ramp.set_position(90, DEGREES)

        # calculate the drivetrain motor velocities from the controller joystick axies
        # left = axis3
        # right = axis2

        new_d_r_horiz = apply_function_on_stick(
            ctrler_r_horiz.position(), 100, stick_func
        )
        d_r_horiz = new_d_r_horiz
        # d_r_horiz = lerp(last_d_r_horiz, new_d_r_horiz, TURN_SMOOTHING)

        new_d_l_vert = ctrler_l_vert.position()
        d_l_vert = new_d_l_vert
        # d_l_vert = lerp(last_d_l_left, new_d_l_vert, THROTTLE_SMOOTHING)

        print("turning: " + str(d_r_horiz))
        print("throttle: " + str(d_l_vert))

        d_l_vert_round = math.copysign(math.floor(abs(d_l_vert)), d_l_vert)
        d_r_horiz_round = math.copysign(math.floor(abs(d_r_horiz)), d_r_horiz)

        l_drivetrain_vel = min_neg(d_l_vert_round + d_r_horiz_round, 100)
        r_drivetrain_vel = min_neg(d_l_vert_round - d_r_horiz_round, 100)

        print("left motorgroup vel: " + str(l_drivetrain_vel))
        print("right motorgroup vel: " + str(r_drivetrain_vel))
        print()

        set_motor_speed(motorgroup_l, l_drivetrain_vel)
        set_motor_speed(motorgroup_r, r_drivetrain_vel)

        set_motor_speed(
            motor_intake,
            100 * (ctrler_btn_intake_in.pressing() - ctrler_btn_intake_out.pressing()),
        )
        set_motor_speed(
            motor_scoring,
            -100
            * (
                ctrler_btn_scoring_up_hold.pressing()
                - ctrler_btn_scoring_down_hold.pressing()
            ),
        )
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

# rocket
scr.draw_circle(200, 120, 25, 0x00F0000)
scr.draw_circle(280, 120, 25, 0x00F0000)
scr.draw_rectangle(220, 40, 40, 80, 0x00F0000)

# print("Unit test 1")

# motor_sorting.spin_for(FORWARD, 1000, MSEC)
# motor_sorting.spin_for(REVERSE, 1000, MSEC)

# drivetrain.turn_to_heading(45, RotationUnits.DEG, 70, VelocityUnits.PERCENT)

# print("Unit test 2")
# drivetrain.turn_for(LEFT, 45, DEGREES, 70, PERCENT)

# print("Unit tests done")


while True:
    if ai_toggled and len(ai_objs) > 0:
        get_ai_objs()
        # if anything we want to grab is detected
        if len(target_objs) > 0:
            # align to the first one
            align_to_block(target_obj.centerX)
        else:  # otherwise do something else
            # drivetrain.turn_for(IDLE_TURN_DIR, IDLE_TURN_DEGREES, DEGREES)
            pass

        # if we can grab something, grab it
        if detecting_grabbables(target_objs, GRAB_SIZE_REQ, GRAB_X_RANGE, GRAB_Y_POS):
            # spin the intake motor and stop moving

            Thread(motor_intake.spin, [FORWARD])
            # Thread(drivetrain.stop)
            pass
        # if we can grab something but wrong team, dont
        elif detecting_grabbables(ai_objs, GRAB_SIZE_REQ, GRAB_X_RANGE, GRAB_Y_POS):
            # TODO considering changing this

            Thread(motor_intake.spin, [REVERSE])
            # Thread(drivetrain.drive, [REVERSE])
            pass
        else:
            motor_intake.stop()

    # sorting logic ------------------

    # print(optical.hue())
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

    if sorting_override_btn_mode != -1:
        current_sort = 0
        scr.print("sort override: " + str(sorting_override_btn_mode))
        wait(50, MSEC)
        continue

    new_sort = 0
    if detect == TEAM.name:
        new_sort = 1
    elif detect != "NONE":
        new_sort = 2
    else:
        new_sort = 0

    scr.print("new sort: " + str(new_sort))

    if new_sort != current_sort and sorting_timer.time(MSEC) >= SORT_SWITCH_DELAY:
        sorting_timer.reset()
        sorting_timer.event(set_sort, SORT_SWITCH_DELAY, [new_sort])

    wait(20, MSEC)
