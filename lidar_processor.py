import math

def angledst_to_point(angle, distance):
    y = distance*math.sin(math.radians(angle))
    x = distance*math.cos(math.radians(angle))
    return {'x':x, 'y':y}