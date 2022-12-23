import math

class MathExt:
    def get_random(min, max):
        min = math.ceil(min)
        max = math.floor(max)
        return math.floor(math.random() * (max - min + 1)) + min

    def to_radians(degree):
        return degree * (math.pi / 180)