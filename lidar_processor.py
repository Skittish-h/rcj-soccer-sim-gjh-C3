import math

BLUE_GOAL_BOUNDARY = 1
YELLOW_GOAL_BOUNDARY = 1

def angledst_to_point(angle, distance):
    y = distance*math.sin(math.radians(angle))
    x = distance*math.cos(math.radians(angle))
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
    biggest_gap = max(gaps_groups)

    gap_avg_y = sum(biggest_gap)/len(biggest_gap)

    return gap_avg_y


points = [
    {'x':0.9,'y':0.7},
    {'x':0.9,'y':0.65},
    {'x':0.9,'y':0.6},
    {'x':1.1,'y':0.55},
    {'x':0.9,'y':0.5},
    {'x':0.9,'y':0.45},
    {'x':1.1,'y':0.4},
    {'x':1.1,'y':0.35},
    {'x':1.1,'y':0.3},
]

print(get_gap_coordinate(points))