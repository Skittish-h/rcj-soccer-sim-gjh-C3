from controller import Robot

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math


TIME_STEP = 64
from intercepts import interceptCalculator
from CoordinateRecalculator import coor_recalc, robot_pos_recalc
from GoToFunc import goTo

from MovementCalculator import fit_parabola, get_tangent_point, passes_boundary, scores_own_goal

INTERCEPT_CONST = 0.03


class MyScoringRobot(RCJSoccerRobot):
    def getIntercepts(self, robot_pos, Team):
        #get our robots
        r1 = robot_pos_recalc(robot_pos, Team=Team)

        #prep dictionary
        
        #get all the intercepts, with a lota samples
        intercept = self.intercept_c.calculateOptimumIntercept(r1, Team, sample_count=200)
        

        #dict with times

        #dict with our robots intercepts

        return intercept
    def be_attacker(self, myi, robot_pos, team, ball_pos):
        stuff = 0
        point = 0
        if (myi['x'] < robot_pos['x']) if team else (myi['x'] > robot_pos['x']):
            
            #print(intercepts)
            x = fit_parabola(myi, robot_pos ,{'x':(0.0 if team else 1.0),"y":0.5})
            if not passes_boundary(x):
                point = get_tangent_point(robot_pos, x, team)
                ball_angle, robot_angle = self.get_angles(point, robot_pos)
                
                return goTo(point['x'], point['y'], robot_pos, robot_angle, should_soften=False, ball_pos=ball_pos)
                
            else:
                ball_angle, robot_angle = self.get_angles(myi, robot_pos)
                
                return goTo(myi['x'], myi['y'], robot_pos, robot_angle, should_soften=False, ball_pos=ball_pos)
                
        else:
            ball_angle, robot_angle = self.get_angles(myi, robot_pos)
            
            return goTo(myi['x'], myi['y'], robot_pos, robot_angle, should_soften=False, ball_pos=ball_pos)
    def run(self):
        self.intercept_c = interceptCalculator(3)
        Team = (self.team == "B")

        while self.robot.step(TIME_STEP) != -1:
            # Get the position of our robot
            
            robot_pos = self.get_posfrom_devices()
            # Get lidar ranges in an array indexed 0..359, index 0 = front, 90-1 = right, 180-1 = back, 270-1 = left
            lidar_ranges = self.lidar.getRangeImage()
            # just for experiments: print sensor values
            #print(robot_pos, 'front range:', lidar_ranges[0])

            # for i in range(len(lidar_ranges)):
            #     print(i, lidar_ranges[i])


            if self.is_new_data():
                
                data = self.get_new_data()
                #due to extensive openAI gym testing we know that desync DOES occur
                while self.is_new_data():            
                    data = self.get_new_data()

                # Get the position of the ball
                ball_pos = data['ball']
                robot_pos = robot_pos_recalc(robot_pos, Team)
                ball_pos = coor_recalc(data['ball']['x'], data['ball']['y'], Team)
                # YOUR CODE HERE
                self.intercept_c.pushPoint(ball_pos)
                
                myi = self.getIntercepts(robot_pos, Team)

                out = self.be_attacker(myi, robot_pos, Team, ball_pos)

                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])
                # At first you might want to consider some experiments with still robot, then turning faster and faster
                


my_scoring_robot = MyScoringRobot(Robot())
my_scoring_robot.run()
