from controller import Robot

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math
import statistics

TIME_STEP = 64
from intercepts import interceptCalculator
from CoordinateRecalculator import coor_recalc, robot_pos_recalc
from GoToFunc import goTo
from lidar_processor import Get_Lidar_Range
from MovementCalculator import fit_parabola, get_tangent_point, passes_boundary, scores_own_goal

INTERCEPT_CONST = 0.03


class MyScoringRobot(RCJSoccerRobot):
    def getIntercepts(self, robot_pos, Team):
        #get our robots
        r1 = robot_pos

        #prep dictionary
        
        #get all the intercepts, with a lota samples
        intercept = self.intercept_c.calculateOptimumIntercept(r1, Team, sample_count=200)
        

        #dict with times

        #dict with our robots intercepts

        return intercept
    def be_attacker(self, myi, robot_pos, team, ball_pos, target):
        stuff = 0
        point = 0
        if (myi['x'] < robot_pos['x']) if team else (myi['x'] > robot_pos['x']):
            x = fit_parabola(myi, robot_pos ,target)
            x['a'] = 1*x['a']
            if not passes_boundary(x):
                point = get_tangent_point(robot_pos, x, team)
                ball_angle, robot_angle = self.get_angles(point, robot_pos)
                
                return goTo(point['x'], point['y'], robot_pos, robot_angle, team,should_soften=False, ball_pos=ball_pos)
                
            else:
                ball_angle, robot_angle = self.get_angles(myi, robot_pos)
                
                return goTo(myi['x'], myi['y'], robot_pos, robot_angle, team, should_soften=False, ball_pos=ball_pos)
                
        else:
            ball_angle, robot_angle = self.get_angles(myi, robot_pos)
            
            return goTo(myi['x'], myi['y'], robot_pos, robot_angle, team, should_soften=False, ball_pos=ball_pos)
    def run(self):
        median_filter = [0.5, 0.5, 0.5, 0.5, 0.5,0.5, 0,5]
        iterator = 0
        
        prev_ball_x = 0
        self.intercept_c = interceptCalculator(3)
        Team = (self.team != "B")
        myi = None
        cur_gap = None
        ball_pos = None
        while self.robot.step(TIME_STEP) != -1:
            # Get the position of our robot
            
            robot_pos = self.get_posfrom_devices()

            #print(math.degrees(robot_pos['orientation'])-90)
            #print(robot_pos)
            # Get lidar ranges in an array indexed 0..359, index 0 = front, 90-1 = right, 180-1 = back, 270-1 = left
            lidar_ranges = self.lidar.getRangeImage()
            # just for experiments: print sensor values
            #print(robot_pos, 'front range:', lidar_ranges[0])
            gap_y = Get_Lidar_Range(robot_pos, Team,lidar_ranges)

            if gap_y != -1:
                cur_gap = {'x': 1 if not Team else 0, 'y': gap_y}
                median_filter[iterator] = gap_y
                iterator+=1
                if iterator>7:
                    iterator = 0
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
                target = {'x':1 if not Team else 0, 'y': statistics.median(median_filter)}



                myi = self.getIntercepts(robot_pos, Team)

                out = self.be_attacker(myi, robot_pos, Team, ball_pos, target)
                if abs(prev_ball_x - ball_pos['x']) > 0.1:
                    Team = not Team
                    cur_gap=None
                    print("SWITCHERO")

                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])
                prev_ball_x = ball_pos['x']
                # At first you might want to consider some experiments with still robot, then turning faster and faster
            elif myi != None and ball_pos != None and target != None:
                robot_pos = robot_pos_recalc(robot_pos, Team)
                out = self.be_attacker(myi, robot_pos, Team, ball_pos, target)
                self.left_motor.setVelocity(out[1])
                self.right_motor.setVelocity(out[0])



my_scoring_robot = MyScoringRobot(Robot())
my_scoring_robot.run()
