from math import inf, sqrt
import numpy as np
from numpy.lib.npyio import loads
from initialization import *
import math
import enum


# DEFINE STATES HERE!
class States(enum.Enum):
    Init = 0
    Forward = 1
    Wall_F = 2
    Check = 3
    CWTurn = 4
    saf = 5
    Delay = 6
    Goal = 7
    Straight = 8
    Start = 9
    NotReachable=11

# Global
MAX_SPEED = 6.28
state = States.Start
turn = "LL"

mdeg = 300
goal_position = np.array([1.3, 6.15])
Divar_F = False


# Functions
def update_robot_state():
    read_sensors_values()
    global robot_velocity
    global robot_position
    global robot_omega
    # updating the current theta
    robot_position[2] = math.atan2(compass_val[0], compass_val[1])
    # updating the currnet robot position
    robot_position[0] = gps_values[0]
    robot_position[1] = gps_values[1]


def zdegree():
    compass = read_sensors_values()[1]
    rad = math.atan2(compass[0], compass[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing = bearing if bearing >= 0 else bearing + 360
    return bearing

def head_to_goal():
    global goal_position
    global robot_position
    zavie = math.atan2(goal_position[1] - robot_position[1], goal_position[0] - robot_position[0])
    zavie = ((zavie - 1.57) / 3.14 * 180.0)-60
    zavie = zavie if zavie >= 0 else zavie + 360
    return zavie

def my_sign(number):
    if number < 0:
        return -1
    else:
        return 1


def forward():
    update_motor_speed(input_omega=[(+MAX_SPEED), 0, -MAX_SPEED])
    update_robot_state()


def left():
    update_motor_speed(input_omega=[-(MAX_SPEED * 0.5), (MAX_SPEED), -(MAX_SPEED * 0.5)])
    update_robot_state()


def down():
    update_motor_speed(input_omega=[-MAX_SPEED, 0, +MAX_SPEED])
    update_robot_state()


def cal_slope():
    update_robot_state()
    slope = (goal_position[1] - robot_position[1]) / (goal_position[0] - robot_position[0])
    return slope


def turnL():
    global turn
    global mdeg
    speed = +MAX_SPEED if (turn == "R" or turn == "RR") else -MAX_SPEED
    speed = speed / 5
    if (turn == "L"):
        mdeg = mdeg + (90)
        mdeg = mdeg if mdeg <= 360 else (mdeg - 360)
        turn = "LL"
    if (turn == "R"):
        mdeg = mdeg - (90)
        mdeg = mdeg if mdeg >= 0 else (mdeg + 360)
        turn = "RR"
    if (turn == "LL" or turn == "RR"):
        if (mdeg < zdegree() < mdeg + 0.3):
            update_motor_speed(input_omega=[0, 0, 0])
            turn = ""
        else:
            update_motor_speed(input_omega=[speed, speed, speed])


if __name__ == "__main__":
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0, 0, 0], in_omega=[0, 0, 0])
    goal_position = np.array([1.3, 6.15])
    turn = "LL"
    is_corner = False
    counter_saf = 0
    now_sign = 1
    last_sign = 1
    saf_timer = 0

    while robot.step(TIME_STEP) != -1:
        gps_values, compass_val, sonar_value, encoder_value, ir_value = read_sensors_values()
        update_robot_state()

        if state == States.Start:
            print("Start")
            firstT = zdegree()
            mainslope = (goal_position[1] - robot_position[1]) / (goal_position[0] - robot_position[0])
            # set_degree
            leave_point = [robot_position[0], robot_position[1]]
            mdeg = head_to_goal()
            state = States.Init

        if state == States.Init:
            print("INIT")
            if (turn == ""):
                counter_init = robot.getTime()
                state = States.Forward
            else:
                turnL()


        elif (state == States.Forward):
            # check_for_goal
            distance = math.sqrt(
                (goal_position[1] - robot_position[1]) ** 2 + (goal_position[0] - robot_position[0]) ** 2)
            if (distance < 0.4):
                state = States.Goal

            forward()
            # is_forward_wall_reached
            if (sonar_value[0] < 20 or ir_value[2] < 600 or ir_value[5] < 600):
                read_sensors_values()
                state = States.saf
                turn = "LL"
                update_robot_state()
                leave_point = (robot_position[0], robot_position[1],robot.getTime())
                update_motor_speed(input_omega=[0, 0, 0])
                counter_saf = robot.getTime()
            # is_kaj
            elif (counter_init + 10 < robot.getTime() and abs(zdegree() - head_to_goal()) > 3):
                state = States.Init
                turn = "LL"

        elif (state == States.saf):
            print("Saf")
            # see_wall_for_first_time_or_being_blind
            if Divar_F == False or (Divar_F == True and robot.getTime() < counter_saf + 5):
                # is_not_kaj
                if abs(ir_value[2] - ir_value[5]) < 15 and ir_value[2] != 1000 and ir_value[5] != 1000:
                    # change_state_wall_follow
                    update_motor_speed(input_omega=[0, 0, 0])
                    divar = zdegree()
                    mdeg = divar
                    Divar_F = True
                    state = States.Wall_F
                else:
                    update_motor_speed(input_omega=[(1 / 5) * MAX_SPEED, (1 / 5) * MAX_SPEED, (1 / 5) * MAX_SPEED])
                    read_sensors_values()
            else:
                # saf kardan nesbat be diver ghabli

                if (divar - 0.1 < zdegree() < divar + 0.2):
                    update_motor_speed(input_omega=[0, 0, 0])
                    mdeg = divar
                    state = States.Wall_F
                else:
                    update_motor_speed(input_omega=[(1 / 5) * MAX_SPEED, (1 / 5) * MAX_SPEED, (1 / 5) * MAX_SPEED])
                    read_sensors_values()

        elif state == States.Straight:
            print("Straight")

            read_sensors_values()
            # check for too close!
            if sonar_value[0] < 40:
                down()
            # check for not far!
            if sonar_value[0] > 100:
                state = state.Wall_F
                update_motor_speed(input_omega=[0, 0, 0])

        elif state == States.Wall_F:
            print("Wall")
            update_robot_state()
            now_sign = my_sign(cal_slope())
            distance = math.sqrt(
                (leave_point[1] - robot_position[1]) ** 2 + (leave_point[0] - robot_position[0]) ** 2)
            if (distance<0.1 and robot.getTime()>leave_point[2]+60):
                state=States.NotReachable
                update_motor_speed(input_omega=[0, 0, 0])
            
            
            # is_line_reach
            elif (now_sign != last_sign and robot_position[1] - leave_point[1] > 0.05 and abs(
                    robot_position[0] - leave_point[0]) < 0.9 and (goal_position[1] - robot_position[1]) >= 0):
                state = States.Check
            # is_left_wall_reached
            elif (sonar_value[1] < 200 or ir_value[0] < 800 or ir_value[3] < 800):
                update_motor_speed(input_omega=[0, 0, 0])
                turn = "L"
                state = States.CWTurn
            # is_resume_left
            elif (sonar_value[0] < 100 and ir_value[2] < 900 and ir_value[5] < 900):
                # is_kaj_or_too_close
                if ((abs(ir_value[2] - ir_value[5]) > 100 and (ir_value[2] + ir_value[5] > 500)) or sonar_value[
                    0] < 20):
                    state = States.Straight
                else:
                    left()
            # is_at_least_one_sensor_detecting
            elif (sonar_value[0] < 200 or ir_value[2] < 900 or ir_value[5] < 900):
                left()
                is_corner = False

            elif is_corner:
                left()
            # check_empty
            elif ir_value[0] > 900 and ir_value[1] > 900 and ir_value[2] > 900 and ir_value[3] > 900 and ir_value[
                4] > 900 and ir_value[5] > 900:
                if (sonar_value[0] > 500 and sonar_value[1] > 700 and sonar_value[2] > 700):
                    left()
                    is_corner = True
                    delay = robot.getTime()
                    state = States.Delay
            last_sign = now_sign

        elif (state == States.Delay):
            print("Delay")
            # check for a second
            if (delay + 1.1 < robot.getTime()):
                turn = "R"
                state = States.CWTurn
                update_motor_speed(input_omega=[0, 0, 0])
        elif (state == States.NotReachable):
            print("'Notreachable")
            update_motor_speed(input_omega=[0, 0, 0])
        elif (state == States.Check):
            print("Check")
            # set_head_toward_goal
            mdeg = head_to_goal()
            turn = "LL"
            if (mdeg < zdegree() < mdeg + 0.3):
                state = States.Forward
                turn = ""
                counter_init = robot.getTime()
            else:
                turnL()
                turn = "L"

        elif (state == States.CWTurn):
            print("CWTurn")
            # turn or go to wall follow
            if turn != "":
                turnL()
            else:
                state = States.Wall_F

        elif (state == States.Goal):
            print("Goal")
            mdeg = firstT
            turn = "LL"
            if (firstT <= zdegree() < firstT + 0.2):
                update_motor_speed(input_omega=[0, 0, 0])
                turn = ""
            else:
                turnL()

        update_robot_state()
