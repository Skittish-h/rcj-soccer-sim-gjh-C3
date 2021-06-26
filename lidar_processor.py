from typing import Type
from CoordinateRecalculator import robot_pos_recalc

from CoordinateRecalculator import coor_recalc

import math

BLUE_GOAL_BOUNDARY = 1
YELLOW_GOAL_BOUNDARY = 1


def get_angles(ball_pos: dict, robot_pos: dict):

    robot_angle: float = robot_pos['orientation']

    # Get the angle between the robot and the ball
    angle = math.atan2(
        ball_pos['y'] - robot_pos['y'],
        ball_pos['x'] - robot_pos['x'],
    )

    if angle < 0:
        angle = 2 * math.pi + angle

    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    robot_ball_angle = math.degrees(angle + robot_angle)

    # Axis Z is forward
    # TODO: change the robot's orientation so that X axis means forward
    robot_ball_angle -= 90
    if robot_ball_angle > 360:
        robot_ball_angle -= 360

    return robot_ball_angle


def angledst_to_point(angle, distance, rob_Pos):
    y = rob_Pos['y'] + distance*math.sin(math.radians(angle))
    x = rob_Pos['x'] + distance*math.cos(math.radians(angle))

    return {'x':x, 'y':y}


def get_gap_groups(coordinates: list):
    gap_g = [[]]
    
    for i in range(len(coordinates)):
        if coordinates[i]['x'] > BLUE_GOAL_BOUNDARY:
            gap_g[-1].append(coordinates[i]['y'])
        elif gap_g[-1]:
            gap_g.append([])
    return gap_g


def get_gap_coordinate(coordinates: list):
    
    gaps_groups = get_gap_groups(coordinates)
    print(gaps_groups)
    if gaps_groups != [[]]:
        max = []
        for gap_g in gaps_groups:
            if len(gap_g) > len(max):
                max = gap_g
        biggest_gap = max
        gap_avg_y = sum(biggest_gap)/len(biggest_gap)

        return gap_avg_y
    return -1



#print(get_gap_coordinate(points))
def Get_Goal_Angles(rob_pos, Team):
    #AFTER GOAL SWICHT SCORE ON BLUE TO FALSE OR TRUE (after every goal u switch it)
    scoreOnBlue = True
    topOfGoalY = -0.1825361138777991
    topOfGoalX = 0.75
    if(scoreOnBlue):
        abs(topOfGoalX)
    else:
        topOfGoalX*= -1

    angleToTop = get_angles({'x':topOfGoalX, 'y': topOfGoalY}, rob_pos)
    angleToBottom = get_angles({'x':topOfGoalX, 'y': -topOfGoalY}, rob_pos)

    
    if(angleToBottom < 0):
        angleToBottom +=360
    if(angleToTop < 0 ):
        angleToTop +=360
    #print("Angle to top: ", angleToTop, " Angle to butt: ", angleToBottom)
    #angle from point tot the top of goal

    #print("angle to goal: " ,math.degrees(smallQ))
    #`print("Robot Orientation: ", math.degrees(rob_pos['orientation']))
    # if(angleToTop > angleToBottom):
    #     print("THIS HAPPENS")
    #     return {'max':angleToTop , 'min': angleToBottom}
    # elif(angleToBottom > angleToTop):
    
    return {'max': angleToBottom, 'min': angleToTop}


def Get_Lidar_Range(rob_pos, Team, Lid):
    MaxNMinGoals= Get_Goal_Angles(rob_pos, Team)
    LidGoalRange = []
    center_aligned = MaxNMinGoals['min'] >= MaxNMinGoals['max']
    if center_aligned:
        for i in list(range(int(MaxNMinGoals['min']), 360)) + list(range(int(MaxNMinGoals['max']))):
            real_angle = i - (math.degrees(rob_pos['orientation'])-90)
            LidGoalRange.append(coor_recalc(angledst_to_point(real_angle, Lid[i], rob_pos)['x'], angledst_to_point(real_angle, Lid[i], rob_pos)['y']))
        return get_gap_coordinate(LidGoalRange)
    for i in range (int(MaxNMinGoals['min']), int(MaxNMinGoals['max'])):
        real_angle = i - (math.degrees(rob_pos['orientation'])-90)
        LidGoalRange.append(coor_recalc(angledst_to_point(real_angle, Lid[i], rob_pos)['x'], angledst_to_point(real_angle, Lid[i], rob_pos)['y']))
    return get_gap_coordinate(LidGoalRange)
        
