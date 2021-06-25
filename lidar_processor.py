from CoordinateRecalculator import robot_pos_recalc

import math



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


def angledst_to_point(angle, distance):
    y = distance*math.sin(math.radians(angle))
    x = distance*math.cos(math.radians(angle))
    return {'x':x, 'y':y}


def Get_Goal_Angles(rob_pos, Team):
    #AFTER GOAL SWICHT SCORE ON BLUE TO FALSE OR TRUE (after every goal u switch it)
    scoreOnBlue = True
    topOfGoalY = -0.1825361138777991;
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
    print("Angle to top: ", angleToTop, " Angle to butt: ", angleToBottom)
    #angle from point tot the top of goal

    #print("angle to goal: " ,math.degrees(smallQ))
    #`print("Robot Orientation: ", math.degrees(rob_pos['orientation']))
    if(angleToTop > angleToBottom):
        return {'max':angleToTop , 'min': angleToBottom}
    elif(angleToBottom > angleToTop):
        return {'max': angleToBottom, 'min': angleToTop}