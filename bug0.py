
from math import inf, sqrt
import numpy as np
from numpy.lib.npyio import loads
from initialization import *
import math
import enum


# DEFINE STATES HERE!
class States(enum.Enum):
    Head_to_goal = 0
    Forward = 1
    Wall_F = 2
    CWTurn = 3
    Delay = 4
    Goal = 5
    Straight =6
    Start = 7
    Saf = 8
    Dir_correction = 9
    Notreachable= 10


# Global
MAX_SPEED = 6.28
state = States.Start
turn = "LL"
leave_points=list()
goal_position = np.array([1.3, 6.15])

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

def head_to_goal():
    global goal_position
    global robot_position
    zavie = math.atan2(goal_position[1] - robot_position[1], goal_position[0] - robot_position[0])
    zavie = ((zavie - 1.57) / 3.14 * 180.0)-60
    zavie = zavie if zavie >= 0 else zavie + 360
    return zavie

def zdegree():
    compass = read_sensors_values()[1]
    rad = math.atan2(compass[0], compass[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing = bearing if bearing >= 0 else bearing + 360
    return bearing


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


# def main1():
if __name__ == "__main__":
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0, 0, 0], in_omega=[0, 0, 0])
    goal_position = np.array([1.3, 6.15])

    
    while robot.step(TIME_STEP) != -1:
        gps_values, compass_val, sonar_value, encoder_value, ir_value = read_sensors_values()
        update_robot_state()

        if state == States.Start:
            print("Start")
            turn = "LL"
            counter_saf = 0            
            firstT = zdegree()
            # mainslope = (goal_position[1] - robot_position[1]) / (goal_position[0] - robot_position[0])
            # set_degree
            leave_point = [robot_position[0], robot_position[1]]
            mdeg = head_to_goal()
            state = States.Head_to_goal

        elif state == States.Head_to_goal:
            update_motor_speed(input_omega=[0, 0, 0])
            mdeg=head_to_goal()
            print("Head_to_goal")
            if (turn == ""):
                counter_init = robot.getTime()
                state = States.Forward
            else:
                turnL()


        elif (state == States.Forward):
            distance = math.sqrt(
                (goal_position[1] - robot_position[1]) ** 2 + (goal_position[0] - robot_position[0]) ** 2)
            if (distance < 0.4):
                state = States.Goal            
            # is_forward_wall_reached
            if (sonar_value[0] < 20 or ir_value[2] < 750 or ir_value[5] < 750):
                read_sensors_values()
                temp2=zdegree()
                state = States.Saf
                turn = "LL"
                update_robot_state()
                #Check_for_unreachable_state
                pointcount=0
                for point in leave_points:
                    if (math.sqrt((point[1] - robot_position[1]) ** 2 + (point[0] - robot_position[0]) ** 2)>0.3):
                        pointcount+=1

                if (pointcount>2):
                    state=States.Notreachable        
                leave_points.append ((robot_position[0], robot_position[1]))
                update_motor_speed(input_omega=[0, 0, 0])
                counter_saf = robot.getTime()
            # is_kaj
            elif (counter_init + 10 < robot.getTime() and abs(zdegree() - head_to_goal()) > 3):
                state = States.Head_to_goal
                turn = "LL"
            else:
                forward()

        elif (state == States.Saf):
            print("Saf")
            
            # see_wall_for_first_time_or_being_blind
            if (robot.getTime() < counter_saf + 15):
                # is_not_kaj
                if (abs(ir_value[2] - ir_value[5]) < 100 and ir_value[2] != 1000 and ir_value[5] != 1000) :
                    # change_state_wall_follow
                    update_motor_speed(input_omega=[0, 0, 0])
                    divar = zdegree()
                    mdeg = divar
                    state = States.Wall_F
                else:
                    update_motor_speed(input_omega=[(1/3)*MAX_SPEED,(1/3)*MAX_SPEED,(1/3)*MAX_SPEED])
                    read_sensors_values()

            else:
                state=States.Dir_correction
                update_motor_speed(input_omega=[0, 0, 0])
                temp2= 300 if 240<temp2<330 else 210 if 150<temp2<240 else 30 if 330<temp2<60 else 120
                
        #make_robot_parallel_to_wall
        elif (state == States.Dir_correction):
            print("dircorrect")
            if(temp2-0.3<zdegree()<temp2+0.3):
                update_motor_speed(input_omega=[0, 0, 0])
                left()
                if(robot.getTime()>time+1):
                    state=States.Wall_F
                    mdeg=temp2
            else:
                update_motor_speed(input_omega=[(1/5)*MAX_SPEED, (1/5)*MAX_SPEED,(1/5)*MAX_SPEED])                    
                time=robot.getTime()
        elif state == States.Wall_F:
            print("Wall")

            update_robot_state()

            # is_left_wall_reached
            if (sonar_value[1] < 200 or ir_value[0] < 800 or ir_value[3] < 800):
                update_motor_speed(input_omega=[0, 0, 0])
                turn = "L"
                state = States.CWTurn
            # is_resume_left
            elif (sonar_value[0] < 100 and ir_value[2] < 900 and ir_value[5] < 900):
                # is_kaj_or_too_close
                if ( sonar_value[0] < 20):
                    state = States.Straight
                else:
                    left()
            # is_at_least_one_sensor_detecting
            elif (sonar_value[0] < 200 or ir_value[2] < 900 or ir_value[5] < 900):
                left()

            # check_empty
            elif ir_value[0] > 900 and ir_value[1] > 900 and ir_value[2] > 900 and ir_value[3] > 900 and ir_value[
                4] > 900 and ir_value[5] > 900:
                if (sonar_value[0] > 700 and sonar_value[1] > 700 and sonar_value[2] > 700):
                    
                    delay = robot.getTime()
                    state = States.Delay

        elif (state == States.Notreachable):
            print("'Notreachable")
            update_motor_speed(input_omega=[0, 0, 0])

                
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


        elif (state == States.Delay):
            print("Delay")
            # check for a second
            if (delay + 1.5 < robot.getTime()):
                turn = "LL"
                state=States.Head_to_goal


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
