# 9.19.21
# Path Planner for robotic arm with 6DoF
# Curvature Method
# Bezier Curves created based on triplets of points.
#  Handles for curves are automatically generated.

import numpy as np
import numpy.linalg as LA

class SmoothPath:
    def __init__(self, points, resolution, max_sample_dist):
        self.points = points
        self.resolution = resolution +1
        self.msd = max_sample_dist

        if(len(self.points) > 1):
            self.vectors = self.vectorize()
            self.curves = self.handle()
            self.path = self.draw()

    def vectorize(self):
        vectors = []

        for p in range(1, len(self.points)):
            vectors.append([
                self.points[p-1],
                self.points[p]
            ])

        return vectors

    def handle(self):

        vectors = self.vectors
        if(len(vectors) == 1): return vectors

        curves = []
        h = 0
        for v in range(len(vectors) -1):
            # set points
            p0 = np.array(vectors[v][0])
            p3 = np.array(vectors[v][1])
            p6 = np.array(vectors[v+1][1])

            # initialize endpoint handles
            p1 = np.array(p0)
            p5 = np.array(p6)

            if(v > 1 and v%2 == 1): 
                prev5 = np.array(curves[h-1][4])
                prev6 = np.array(curves[h-1][5])
                p1 = -(prev5 - prev6) + prev6

            # vectors from set points
            vec30 = p0 - p3
            vec36 = p6 - p3

            # angles for tangent
            inner = np.inner(vec30, vec36)
            norms = LA.norm(vec30) * LA.norm(vec36)
            cos = inner / norms
            phi = np.arccos(np.clip(cos, -1.0, 1.0))
            theta = (np.pi - phi) / 2

            # intended curvature
            offset = 0.5
            k = ((np.pi - phi) / np.pi) * offset

            # unit vector handles around p3
            # uses Rodrigues' rotation formula
            # https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
            norm = np.cross(vec30, vec36)
            nk = norm / LA.norm(norm)
            
            p2prime = vec30*np.cos(-theta) + np.cross(nk, vec30)*np.sin(-theta) + nk*np.dot(nk, vec30)*(1-np.cos(-theta))
            p4prime = vec36*np.cos( theta) + np.cross(nk, vec36)*np.sin( theta) + nk*np.dot(nk, vec36)*(1-np.cos( theta))

            # attempt to find a suitable l given a set of samples
            l_samples = [.5,.6,.7,.8,.9,1]
            l = 0
            estimated_k = 0
            prev_dist = 0
            skip_flag = False
            for i in range(len(l_samples)):
                if(skip_flag == True): continue

                l = l_samples[i]
                p2 = (p2prime * l) + p3
                p4 = (p4prime * l) + p3

                estimated_k = self.curvature([p0,p1,p2,p3,p4,p5,p6])
                dist = abs(k - estimated_k)

                if(i == 0): prev_dist = dist

                if(dist > prev_dist):
                    skip_flag = True
                    print("skip!")

                else: prev_dist = dist

            # choose closest l value for handles
            p2 = (p2prime * l) + p3
            p4 = (p4prime * l) + p3

            # modify endpoint handles
            if(v == 0): 
                vec03 = p3 - p0
                inner = vec03*np.cos(-theta) + np.cross(nk, vec03)*np.sin(-theta) + nk*np.dot(nk, vec03)*(1-np.cos(-theta))
                #p1 = inner / LA.norm(vec03) * l + p0

            if(v+1 == len(vectors)-1): 
                vec63 = p3 - p6
                inner = vec63*np.cos( theta) + np.cross(nk, vec63)*np.sin( theta) + nk*np.dot(nk, vec63)*(1-np.cos( theta))
                #p5 = inner / LA.norm(vec63) * l + p6

            curves.append([p0,p1,p2,p3,p4,p5,p6])
            h += 1

        return curves

    def curvature(self, points):
        p0 = points[0]
        p1 = points[1]
        p2 = points[2]
        p3 = points[3]
        p4 = points[4]
        p5 = points[5]
        p6 = points[6]

        estimate = (abs(((3*p0[1])/4 + (3*p1[1])/4 - (3*p2[1])/4 + (3*p4[1])/4 - (3*p5[1])/4 - (3*p6[1])/4)*((9*p1[0])/2 - 
        3*p0[0] + (3*p2[0])/2 - 6*p3[0] + (9*p4[0])/2 + (3*p5[0])/2 - 3*p6[0]) - ((3*p0[0])/4 + (3*p1[0])/4 - 
        (3*p2[0])/4 + (3*p4[0])/4 - (3*p5[0])/4 - (3*p6[0])/4)*((9*p1[1])/2 - 3*p0[1] + (3*p2[1])/2 - 6*p3[1] + 
        (9*p4[1])/2 + (3*p5[1])/2 - 3*p6[1]))**2 + abs(((3*p0[2])/4 + (3*p1[2])/4 - (3*p2[2])/4 + (3*p4[2])/4 - 
        (3*p5[2])/4 - (3*p6[2])/4)*((9*p1[0])/2 - 3*p0[0] + (3*p2[0])/2 - 6*p3[0] + (9*p4[0])/2 + (3*p5[0])/2 - 
        3*p6[0]) - ((3*p0[0])/4 + (3*p1[0])/4 - (3*p2[0])/4 + (3*p4[0])/4 - (3*p5[0])/4 - (3*p6[0])/4)*((9*p1[2])/2 - 
        3*p0[2] + (3*p2[2])/2 - 6*p3[2] + (9*p4[2])/2 + (3*p5[2])/2 - 3*p6[2]))**2 + abs(((3*p0[2])/4 + (3*p1[2])/4 - 
        (3*p2[2])/4 + (3*p4[2])/4 - (3*p5[2])/4 - (3*p6[2])/4)*((9*p1[1])/2 - 3*p0[1] + (3*p2[1])/2 - 6*p3[1] + 
        (9*p4[1])/2 + (3*p5[1])/2 - 3*p6[1]) - ((3*p0[1])/4 + (3*p1[1])/4 - (3*p2[1])/4 + (3*p4[1])/4 - (3*p5[1])/4 - 
        (3*p6[1])/4)*((9*p1[2])/2 - 3*p0[2] + (3*p2[2])/2 - 6*p3[2] + (9*p4[2])/2 + (3*p5[2])/2 - 
        3*p6[2]))**2)**(1/2)/(abs((3*p0[0])/4 + (3*p1[0])/4 - (3*p2[0])/4 + (3*p4[0])/4 - (3*p5[0])/4 - 
        (3*p6[0])/4)**2 + abs((3*p0[1])/4 + (3*p1[1])/4 - (3*p2[1])/4 + (3*p4[1])/4 - (3*p5[1])/4 - (3*p6[1])/4)**2 + 
        abs((3*p0[2])/4 + (3*p1[2])/4 - (3*p2[2])/4 + (3*p4[2])/4 - (3*p5[2])/4 - (3*p6[2])/4)**2)**(3/2)

        return estimate

    def cubic(self, points, t):
        p0 = points[0]
        p1 = points[1]
        p2 = points[2]
        p3 = points[3]

        point_on_curve = [
            p0[0]*(1-t)**3 + 3*t*p1[0]*(1-t)**2 + 3*(t**2)*p2[0]*(1-t) + (t**3)*p3[0],
            p0[1]*(1-t)**3 + 3*t*p1[1]*(1-t)**2 + 3*(t**2)*p2[1]*(1-t) + (t**3)*p3[1],
            p0[2]*(1-t)**3 + 3*t*p1[2]*(1-t)**2 + 3*(t**2)*p2[2]*(1-t) + (t**3)*p3[2]
        ]
        
        return point_on_curve

    def sample(self, handles):
        dist = self.msd # distance between samples
        n = self.resolution # resolution
        
        # calculate arc length
        arclength = 0
        for i in range(n):
            arclength += LA.norm(np.array(self.cubic(handles, i/n)) - np.array(self.cubic(handles, (i+1)/n)))

        # if dist = -1, then autocalculate sample distance
        if(dist == -1):
            dist = arclength/n

        samples = []
        for j in range(n):
            jt = j * (dist/arclength)

            # if the point is beyond the arclength
            if(jt > 1): break

            #print(jt)
            #print("handles",handles)
            samples.append(self.cubic(handles, jt))

        # include final point (t = 1)
        samples.append(self.cubic(handles, 1))

        return samples

    def draw(self):
        curves = []
        for h in self.curves:
            curves.append( self.sample([h[0],h[1],h[2],h[3]]) )
            curves.append( self.sample([h[3],h[4],h[5],h[6]]) )
        
        points = []
        for c in curves:
            for p in range(len(c)-1):
                # prune final curve points
                if(p == 0 or p%4 != 0):
                    points.append(c[p])

        # append final point
        points.append(curves[len(curves)-1][len(curves[len(curves)-1])-1])

        return points

if __name__ == '__main__':
    points = [
        #[0,  10, 0],
        #[5,  20, 0]
        #[10, 10, 0],
        #[20, 0,  0],
        #[25, 10, 0],
        #[30, 0,  0]

        [0,  10, 0],
        [5,  20, 0],
        [10, 10, 0],
        [15, 15, 0],
        [20, 10, 0],
        [25, 15, 0]
    ]
    resolution = 3
    max_sample_dist = -1
    trajectory = SmoothPath(points, resolution, max_sample_dist)

    print("Input",points)
    #print("Output")
    count = 0
    for point in trajectory.path:
        #print(str(count) + ": " + str(point))
        count += 1