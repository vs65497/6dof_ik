# Custom Linear Algebra Library with Quaternion support
# 9.19.21

import math
import numpy

class Vector:
    # gotta fix this!!!!
    def __init__(self, here, there={"x":0,"y":0,"z":0}):
    #def __init__(self, there, here={"x":0,"y":0,"z":0}):

        if(type(here) == Vector):
            here = here.v

        if(type(there) == Vector):
            there = there.v

        # assume i got this right
        if(len(here) != len(there)):
            raise Exception("Both points must have same number of rows")

        self.p_from = here
        self.p_to = there

        self.v = Vector.create(here, there)

        self.dim = len(there)
        self.length = self.magnitude()

    def create(a, b):
        return {
            "x": b["x"] - a["x"],
            "y": b["y"] - a["y"],
            "z": b["z"] - a["z"]
        }

    # check is zero vector
    def isZero(self):
        for i in self.p_from:
            if(self.p_from[i] != 0): return False

        return True

    # get length
    def magnitude(self):
        total = 0

        for i in self.v:
            total += self.v[i]**2

        return math.sqrt(total)

    # dot product
    def dot(a, b):
        if(type(a) == Vector):
            a = a.v
        
        if(type(b) == Vector):
            b = b.v

        #if(a.dim != b.dim):
        if(len(a) != len(b)):
            raise Exception("Vectors must be same size")

        sum = 0
        for i in a:
            sum += a[i] * b[i]

        return sum

    # cross product
    def cross(a, b):
        if(type(a) == Vector):
            a = a.v
        
        if(type(b) == Vector):
            b = b.v

        if(len(a) != len(b)):
            raise Exception("Vectors must be same size")

        if(len(b) != 3):
            raise Exception("Can only cross vectors in R3")

        return {
            "x": (a["y"] * b["z"]) - (a["z"] * b["y"]),
            "y": (a["z"] * b["x"]) - (a["x"] * b["z"]),
            "z": (a["x"] * b["y"]) - (a["y"] * b["x"])
        }

    # vector addition
    def __add__(a, b):
        if(a.dim != b.dim):
            raise Exception("Vectors must be same size")

        a = a.v
        b = b.v

        return {
            "x": a["x"] + b["x"],
            "y": a["y"] + b["y"],
            "z": a["z"] + b["z"]
        }

    # vector subtraction
    def __sub__(a, b):
        if(a.dim != b.dim):
            raise Exception("Vectors must be same size")

        a = a.v
        b = b.v

        return {
            "x": a["x"] + b["x"],
            "y": a["y"] + b["y"],
            "z": a["z"] + b["z"]
        }

    # scalar multiplication
    def __mul__(a, b):
        """ if(isinstance(a, list)):
            raise Exception("A must be a vector")

        if(isinstance(b, int) or isinstance(b, float)):
            raise Exception("B must be a scalar") """

        # B is the Vector
        if((type(a) == int or type(a) == float) and type(b) == Vector):
            temp = a
            a = b.v # make A the Vector
            b = temp

        # A is the Vector
        elif((type(b) == int or type(b) == float) and type(a) == Vector):
            a = a.v
            b = b

        # A and B are Vectors
        else:
            raise Exception("A and B both Vectors")

        return {
            "x": a["x"] * b,
            "y": a["y"] * b,
            "z": a["z"] * b
        }

    # gets angle between two vectors
    def angle(self, a, b):
        return numpy.arccos(Vector.dot(a,b)/(a.length * b.length))

    # hamilton for quaternions
    def hamilton(**matrix):
        
        a = matrix["a"]
        b = matrix["b"]

        return {
            "w": (a["w"] * b["w"]) - (a["x"] * b["x"]) - (a["y"] * b["y"]) - (a["z"] * b["z"]),
            "x": (a["w"] * b["x"]) + (a["x"] * b["w"]) + (a["y"] * b["z"]) - (a["z"] * b["y"]),
            "y": (a["w"] * b["y"]) - (a["x"] * b["z"]) + (a["y"] * b["w"]) + (a["z"] * b["x"]),
            "z": (a["w"] * b["z"]) + (a["x"] * b["y"]) - (a["y"] * b["x"]) + (a["z"] * b["w"])
        }

    # quaternion rotation
    def rotate(**kwargs):
        vec = kwargs.get("vector")
        origin = kwargs.get("origin")
        angle = kwargs.get("angle") /2
        axis = kwargs.get("axis")

        if(len(vec) != 3):
            raise Exception("Quaternions only on vectors in R3")

        p = {
            "w": 0,
            "x": vec["x"] - origin["x"],
            "y": vec["y"] - origin["y"],
            "z": vec["z"] - origin["z"]
        }

        r = {
            "w": math.cos(angle),
            "x": math.sin(angle)*axis["i"],
            "y": math.sin(angle)*axis["j"],
            "z": math.sin(angle)*axis["k"]
        }

        r1 = {
            "w": r["w"],
            "x": -r["x"],
            "y": -r["y"],
            "z": -r["z"]
        }

        matA = {"a": r, "b": p}
        ham_a = Vector.hamilton(**matA)
        matB = {"a": ham_a, "b": r1}
        p_to = Vector.hamilton(**matB)

        #p_to = Vector.hamilton(Vector.hamilton(r, p), r1)

        return {
            "x": p_to["x"] + origin["x"],
            "y": p_to["y"] + origin["y"],
            "z": p_to["z"] + origin["z"]
        }