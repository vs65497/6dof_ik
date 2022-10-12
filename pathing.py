# 9.19.21
# Path Planner for robotic arm with 6DoF
# Bezier Curves created based on triplets of points.
#  Handles for curves are automatically generated.

from re import X
import numpy
import math
from linalg import Vector

class ArmPath:
    def __init__(self, points, resolution, max_sample_dist):
        self.setpoints = points
        self.max_sample_dist = max_sample_dist
        self.resolution = resolution +1 # plus one because we only count inner points
        self.circles = []

        # finely sampled and smooth trajectory
        self.path = self.draw()

    # Interpolates or extrapolates, and samples
    #  returns sampled points as move locations
    def draw(self):
        # interpolate circles, extrapolate between circles
        # for every 2 points, sample
        # build instructions
        self.encircle()

        quadratic_curves = []
        for c in self.circles:
            # COLINEAR points
            if(c["radius"] == -1):
                self.sample([c["a"], c["c"]])

            # Non-COLINEAR points
            else:
                h1 = self.interpolate(c["a"], c["b"], c["center"], c["radius"])
                h2 = self.interpolate(c["b"], c["c"], c["center"], c["radius"])
                quadratic_curves.append( self.sample([c["a"], h1, c["b"]]) )
                quadratic_curves.append( self.sample([c["b"], h2, c["c"]]) )

        # still need to account for orphans
        cubic_curves = []
        for i in range(0, len(self.circles)-1, 2):
            handles = self.extrapolate(self.circles[i], self.circles[i+1])
            cubic_curves.append(
                self.sample(
                    [
                        self.circles[i]["c"],
                        handles[0],
                        handles[1],
                        self.circles[i+1]["a"]
                    ]
                )
            )

        # aggreggate curves
        curves = []
        i = 0
        for j in range(0, len(quadratic_curves)-1, 2):
            if(len(quadratic_curves) > 0):
                curves.append(quadratic_curves[j])
                curves.append(quadratic_curves[j+1])
            
            if(len(cubic_curves) > 0 and i < len(cubic_curves)):
                curves.append(cubic_curves[i])
                i += 1

        # ONE Orphan
        if(len(self.setpoints) %3 == 1):
            last = len(self.setpoints) -1
            c = len(self.circles) -1
            circle = self.circles[c]

            # grab the final handle from the final circle
            # move the handle to the other side of the tangent
            # use this handle as a shared point for quadratic bezier curve
            h0 = Vector(circle["c"], self.interpolate(circle["b"], circle["c"], circle["center"], circle["radius"]))
            h1 = circle["c"] - h0
            
            a = self.setpoints[last]
            b = self.setpoints[last -1]
            curves.append( self.sample([a, h1, b]) )
        #else:
            #print("No orphans!")

        # generate trajectory
        path = []
        k = 0
        for c in curves:
            path.append(self.setpoints[k])
            k += 1
            for p in range(len(c)-1):
                # scrape off duplicate points
                # if not endpoint in curve then append
                if(p != 0):
                    path.append(c[p])

        # append final set point
        path.append(self.setpoints[len(self.setpoints)-1])

        return path

    # Gets points on a curve by sampling every dist length
    def sample(self, points):
        dist = self.max_sample_dist # distance between samples
        n = self.resolution # resolution
        arclength = 0
        samples = []
        f = ''

        if(  len(points) == 2): f = 'self.linear(points[0], points[1], '
        elif(len(points) == 3): f = 'self.quadratic(points[0], points[1], points[2], '
        elif(len(points) == 4): f = 'self.cubic(points[0], points[1], points[2], points[3], '

        for i in range(n-1):
            a = eval(f+str((i+1)/n)+')')
            b = eval(f+str(i/n)+')')
            vec = Vector(b,a)
            arclength = vec.length
        
        # if dist = -1, then autocalculate sample distance
        if(dist == -1):
            dist = arclength/n

        for j in range(n):
            jt = j * (dist/arclength)

            # if the point is beyond the arclength
            if(jt > 1): break

            #print(jt)
            samples.append(eval(f+str(jt)+')'))

        # include final point (t = 1)
        samples.append(eval(f+'1)'))

        return samples

    # p0, p1 are endpoints
    def linear(self, p0, p1, t):
        final = {"x":0,"y":0,"z":0}
        final["x"] = (1-t)*p0["x"] + t*p1["x"]
        final["y"] = (1-t)*p0["y"] + t*p1["y"]
        final["z"] = (1-t)*p0["z"] + t*p1["z"]
        
        return final

    # p0, p2 are endpoints
    # p1 is the handle
    def quadratic(self, p0, p1, p2, t):
        final = {"x":0,"y":0,"z":0}
        final["x"] = p0["x"]*(1-t)**2 + 2*t*p1["x"]*(1-t) + (t**2)*p2["x"]
        final["y"] = p0["y"]*(1-t)**2 + 2*t*p1["y"]*(1-t) + (t**2)*p2["y"]
        final["z"] = p0["z"]*(1-t)**2 + 2*t*p1["z"]*(1-t) + (t**2)*p2["z"]

        return final

    # p0, p3 are endpoints
    # p1, p2 are handles
    def cubic(self, p0, p1, p2, p3, t):
        final = {"x":0,"y":0,"z":0}
        final["x"] = p0["x"]*(1-t)**3 + 3*t*p1["x"]*(1-t)**2 + 3*(t**2)*p2["x"]*(1-t) + (t**3)*p3["x"]
        final["y"] = p0["y"]*(1-t)**3 + 3*t*p1["y"]*(1-t)**2 + 3*(t**2)*p2["y"]*(1-t) + (t**3)*p3["y"]
        final["z"] = p0["z"]*(1-t)**3 + 3*t*p1["z"]*(1-t)**2 + 3*(t**2)*p2["z"]*(1-t) + (t**3)*p3["z"]
        
        return final

    def encircle(self):
        groups = []
        g = 0

        # encircling every 3 points
        for i in range(len(self.setpoints)):
            pos = (3+i) %3
            name = ""

            if(pos==0):
                groups.append({})
                name = "a"
            if(pos==1):
                name = "b"
            if(pos==2):
                name = "c"

            groups[g][name] = self.setpoints[i]

            if(pos == 2):
                args = self.findCenter(groups[g]["a"], groups[g]["b"], groups[g]["c"])
                groups[g]["center"] = args["center"]
                groups[g]["radius"] = args["radius"]
                g += 1

        # ORPHANS

        # TWO orphans
        if(len(self.setpoints) %3 == 2):
            last = len(self.setpoints) -1
            a = self.setpoints[last-1]
            c = self.setpoints[last]

            x_hat = Vector([1,0,0])

            ac = Vector(c,a)
            length = ac.length
            mid = 0.5 * ac
            d = (length * x_hat) + mid # circle center
            r = (length**2 + (length/2)**2)**0.5 # circle radius
            b = (-(r-length) * x_hat) + mid # b point

            #args = self.findCenter(a,b,c)

            groups[len(groups)] = {
                "a": a,
                "b": b,
                "c": c,
                "center": d,
                "radius": r
            }

        # ONE orphan
        # in the case of one orphan handle in curve loop

        self.circles = groups

    # Finds the center of a circle given 3 points on its perimeter
    #  IMPORTANT: returns a center of -1 if set is in a line
    def findCenter(self, a, b, c):
        ab = Vector(b, a)
        cb = Vector(b, c)
        ac = Vector(c, a)
        
        # Ensures not Colinear
        # Ensures not same point
        if(Vector.angle(ab, cb) != math.pi and ab.length != 0 and 
            cb.length != 0 and ac.length != 0):

            x = ab.length
            y = cb.length
            ca = Vector(a, c)

            alpha = Vector.angle(ab, ac)
            beta = Vector.angle(cb, ca)

            if(alpha != beta):
                A = 2/x
                B = 2/y
                F = A*numpy.cos(alpha) - B*numpy.cos(beta)
                G = A*numpy.sin(alpha) - B*numpy.sin(beta)

                phi = numpy.arctan(F/G)
                theta = numpy.pi - 2*phi

            # BD perfectly bisects AC
            else:
                ba = Vector(a, b)
                bc = Vector(c, b)
                gamma = Vector.angle(ba, bc)
                theta = 2*numpy.pi - (2*alpha + gamma)

            u = (ac.length/2) / numpy.tan(theta/2)
            r = round(math.sqrt((ac.length/2)**2 + u**2),5)

            P = abs(2 * r * numpy.cos(theta))

            # why is this not negative?
            h = -(x**2 - y**2 - P**2 - 2*P*y*numpy.cos(alpha)) / (2 * (P + (x+y)*numpy.cos(alpha)))

            m = Vector(ac * (h/ac.length)) + Vector(a)
            bm = Vector(m, b)
            d = Vector(bm * (r/bm.length)) + Vector(b)

            print("center at",d)
            
            return { "center": d, "radius": r } # curved path
        
        # COLINEAR FLAG
        else:
            return { "center": -1, "radius": -1 } # linear path

    # Gets handlebar for intracircular points
    #  returns vector H (handlebar)
    def interpolate(self, a, b, center, radius):
        ac = Vector(b, a)
        dc = Vector(b, center)
        da = Vector(a, center)

        d = ac.length
        r = radius

        m = Vector(ac * 0.5) + Vector(a)
        dm = Vector(m, center)

        theta = Vector.angle(dc, da)
        phi = (2*numpy.pi) - (numpy.pi + theta)

        x = math.sqrt(d**2 / (2-2*numpy.cos(phi)))
        u = x * numpy.cos(numpy.pi/4)
        v = math.sqrt(r**2 - (d/2)**2)

        h = Vector(dm * ((v+u)/dm.length)) + Vector(center)

        return h

    # Gets handlebar for extracircular points
    #  returns vector I0 and I0 (handlebars
    #  for final point, C, on F and first point, D, on G)
    def extrapolate(self, circle_f, circle_g):
        # ------------------- #
        # circle F
        # a = circle_f["a"]
        b = Vector(circle_f["b"])
        c = Vector(circle_f["c"])

        f = Vector(circle_f["center"])
        fr = circle_f["radius"]

        # vectors from center F to point A and C
        # fa = Vector(f, a) 
        # fc = Vector(f, c) 

        # h = Vector.cross(fa, fc) # normal of circle F

        # ------------------- #
        # circle G
        d = Vector(circle_g["a"])
        e = Vector(circle_g["b"])
        # m = circle_g["c"]

        g = Vector(circle_g["center"])
        gr = circle_g["radius"]

        # vectors from center G to point A and C
        # ga = Vector(g, a)
        # gc = Vector(g, c)

        # k = Vector.cross(ga, gc) # normal of circle G

        # ------------------- #

        # normal for plane P
        mid = Vector(Vector(g-f) * 0.5).v
        n = Vector(g,mid).v

        # unit vectors
        x_hat = Vector({"x":1,"y":0,"z":0})
        y_hat = Vector({"x":0,"y":1,"z":0})
        z_hat = Vector({"x":0,"y":0,"z":1})

        # tangents
        t0 = Vector(self.interpolate(c.v, b.v, f.v, fr))
        t1 = Vector(self.interpolate(d.v, e.v, g.v, gr))

        # SLOPES
        # tangent 0 from circle F
        # (x - t0_x) / l0 = (y - t0_y) / m0 = (z - t0_z) / n0
        l0 = (Vector.dot(t0, x_hat)) / (t0.length * x_hat.length)
        m0 = (Vector.dot(t0, y_hat)) / (t0.length * y_hat.length)
        n0 = (Vector.dot(t0, z_hat)) / (t0.length * z_hat.length)
        
        # tangent 1 from circle G
        # (x - t1_x) / l1 = (y - t1_y) / m1 = (z - t1_z) / n1
        l1 = (Vector.dot(t1, x_hat)) / (t1.length * x_hat.length)
        m1 = (Vector.dot(t1, y_hat)) / (t1.length * y_hat.length)
        n1 = (Vector.dot(t1, z_hat)) / (t1.length * z_hat.length)

        # INTERSECTIONS
        t0 = t0.v
        t1 = t1.v

        # intersection 0: tangent 0 with plane P
        x0 = (t0["x"] - n["x"]*l0*mid["x"]) / (1 - n["x"]*l0)
        y0 = (t0["y"] - n["y"]*m0*mid["y"]) / (1 - n["y"]*m0)
        z0 = (t0["z"] - n["z"]*n0*mid["z"]) / (1 - n["z"]*n0)

        i0 = {"x":x0,"y":y0,"z":z0}

        # intersection 1: tangent 1 with plane P
        x1 = (t1["x"] - n["x"]*l1*mid["x"]) / (1 - n["x"]*l1)
        y1 = (t1["y"] - n["y"]*m1*mid["y"]) / (1 - n["y"]*m1)
        z1 = (t1["z"] - n["z"]*n1*mid["z"]) / (1 - n["z"]*n1)

        i1 = {"x":x1,"y":y1,"z":z1}

        return [ i0, i1 ]

if __name__ == '__main__':
    points = [
        #{"x":0,  "y":10,  "z":0},
        #{"x":5,  "y":20, "z":0},
        #{"x":10, "y":10,  "z":0},
        #{"x":20,  "y":0,  "z":0},
        #{"x":25,  "y":10, "z":0},
        #{"x":30, "y":0,  "z":0}

        # {"x":10,  "y":10,  "z":0},
        # {"x":15,  "y":15, "z":0},
        # {"x":20, "y":10,  "z":0}
        # IMPORTANT: this is the largest the circle can be!
        #  Need to account for ellipses!!!!!!

        {"x":10,  "y":10,  "z":0},
        {"x":15,  "y":15, "z":0},
        {"x":20, "y":10,  "z":0}
    ]
    resolution = 3
    max_sample_dist = -1
    trajectory = ArmPath(points, resolution, max_sample_dist)

    print("Input",points)
    #print("Output")
    count = 0
    for point in trajectory.path:
        #print(str(count) + ": " + str(point))
        count += 1
    