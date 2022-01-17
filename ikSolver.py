
# 8.20.21
# Inverse Kinematics solver for robotic arm with 6DoF
# Based on Lectures from Angela Sodemann and Elias Brassitos
# Angela Sodemann - https://www.youtube.com/watch?v=KslFPohHkxA
# Elias Brassitos - https://www.youtube.com/watch?v=Is50EWYF99I

import numpy as np

class ikSolver:
    def __init__(self, l1=130, l2=130, d4=160, d6=140, zero_joint=0):
        self.L1=l1    # Limb 1 length
        self.L2=l2    # Limb 2 length
        self.D4=d4    # Upper wrist length
        self.D6=d6    # Lower wrist length

        self.zero_joint = zero_joint
        self.lengths = [
            l1,             # a1
            l2,             # a2
            zero_joint,     # a3
            d4-zero_joint,  # a4
            zero_joint,     # a5
            d6-zero_joint   # a6
        ]
        
        self.singularity = 9999 # singularity when equal to 0
        self.joint_angles = []

        # Default EE
        self.current_ee = [
            l2+d4+d6-10,
            0,
            l1
        ]
        self.orientation = [0,0,0]
        self.init_ee = self.current_ee

        # reserved for external manipulation.
        # this way can reset when joint limit exceeded.
        self.last_confirmed = self.current_ee

    # FIND_SINGULARITY
    # Determinant of Jacobian to find singularities.
    #  If this solution is 0 then there is a singularity,
    #  or in other words, the arm has gone somewhere inaccessible
    #  J = [Jw Jv]_6x6
    #  Jw = R0i[0;0;1] cross (p06-p0i), p0i from H0i -> 1:3 rows, 3rd col
    #  H[n-1][n]_4x4 = [R[n-1][n]_3x3,[r_n*cos(theta_n);r_n*sin(theta_n);d_n];[0,0,0,1]]
    #  d_n => distance through top of motor to next motor (motor1 = a1, motor4 = a4, motor6 = a6)
    #  Jv = R0i[0;0;1]
    #  0 <= i <= 5
    #  --- Check the notes or MATLAB file if confused
    def find_singularity(self, angles):
        a1 = self.lengths[0]
        a2 = self.lengths[1]
        a3 = self.lengths[2]
        a4 = self.lengths[3]
        a5 = self.lengths[4]
        a6 = self.lengths[5]

        # t1 = angles[0]
        t2 = angles[1]
        t3 = angles[2]
        t4 = angles[3]
        t5 = angles[4]
        # t6 = angles[5]
        
        sin2 = np.sin(t2)
        sin3 = np.sin(t3)
        sin4 = np.sin(t4)
        sin5 = np.sin(t5)

        cos2 = np.cos(t2)
        cos3 = np.cos(t3)
        cos4 = np.cos(t4)
        cos5 = np.cos(t5)

        self.singularity = a2*a3**2*sin2*sin5 - a2**3*cos2*sin4 + a2*a4**2*sin2*sin5 - 2*a2**2*a3*cos2*cos3*sin4 \
        - 2*a2**2*a4*cos2*cos3*sin4 - a1*a2**2*cos2*sin2*sin4 - a2**2*a3*cos2*sin3*sin5 \
        - a2**2*a4*cos2*sin3*sin5 + a2**2*a3*sin2*sin3*sin4 + a2**2*a4*sin2*sin3*sin4 \
        + a1*a2*a3*sin3*sin4 + a1*a2*a4*sin3*sin4 + 2*a2*a3*a4*sin2*sin5 - a2*a3**2*cos2*cos3**2*sin4 \
        - a2*a4**2*cos2*cos3**2*sin4 - a2*a3**2*cos3**2*sin2*sin5 - a2*a4**2*cos3**2*sin2*sin5 \
        + a2*a3**2*cos3*sin2*sin3*sin4 + a2*a4**2*cos3*sin2*sin3*sin4 - 2*a2*a3*a4*cos2*cos3**2*sin4 \
        - a1*a2*a3*cos2**2*sin3*sin4 - a1*a2*a4*cos2**2*sin3*sin4 - 2*a2*a3*a4*cos3**2*sin2*sin5 \
        - a2**2*a5*cos2*cos3*cos5*sin4 - a2**2*a6*cos2*cos3*cos5*sin4 - a2*a3**2*cos2*cos3*sin3*sin5 \
        - a2*a4**2*cos2*cos3*sin3*sin5 - a1*a2*a3*cos2*cos3*sin2*sin4 - a1*a2*a4*cos2*cos3*sin2*sin4 \
        - 2*a2*a3*a4*cos2*cos3*sin3*sin5 + 2*a2*a3*a4*cos3*sin2*sin3*sin4 - a2*a3*a5*cos2*cos3**2*cos5*sin4 \
        - a2*a3*a6*cos2*cos3**2*cos5*sin4 - a2*a4*a5*cos2*cos3**2*cos5*sin4 - a2*a4*a6*cos2*cos3**2*cos5*sin4 \
        + a2**2*a5*cos2*cos4*sin3*sin4*sin5 + a2**2*a6*cos2*cos4*sin3*sin4*sin5 \
        + a2*a3*a5*cos3*cos5*sin2*sin3*sin4 + a2*a3*a6*cos3*cos5*sin2*sin3*sin4 \
        + a2*a4*a5*cos3*cos5*sin2*sin3*sin4 + a2*a4*a6*cos3*cos5*sin2*sin3*sin4

        return self.singularity

    # GET_JOINT_POSITIONS
    # This is useful for simulations since 
    #  they run on point coordinates and not motor angles.
    # returns array [] of joint positions from 1 to 6
    #  - assume [0,0,0] as origin 
    #  - all points as [x, y, z]
    #  - IMPORTANT: need to transform coordinates to simulator compliant
    def get_joint_positions(self):
        a1 = self.lengths[0]
        a2 = self.lengths[1]
        a3 = self.lengths[2]
        a4 = self.lengths[3]
        a5 = self.lengths[4]
        a6 = self.lengths[5]

        t1 = self.joint_angles[0]
        t2 = self.joint_angles[1]
        t3 = self.joint_angles[2]
        t4 = self.joint_angles[3]
        t5 = self.joint_angles[4]
        
        sin1 = np.sin(t1)
        sin2 = np.sin(t2)
        sin3 = np.sin(t3)
        sin4 = np.sin(t4)
        sin5 = np.sin(t5)

        cos1 = np.cos(t1)
        cos2 = np.cos(t2)
        cos3 = np.cos(t3)
        cos4 = np.cos(t4)
        cos5 = np.cos(t5)

        # in MATLAB as H0i (H01, H02, H03, etc.), 1 <= i <= 6
        joints = [
            [0,0,0], # 1
            [ # 2
                a2*cos1*cos2,
                a2*cos2*sin1,
                a1 + a2*sin2
            ],
            [ # 3
                a2*cos1*cos2 + a3*cos1*cos2*cos3 - a3*cos1*sin2*sin3,
                a2*cos2*sin1 + a3*cos2*cos3*sin1 - a3*sin1*sin2*sin3,
                a1 + a2*sin2 + a3*cos2*sin3 + a3*cos3*sin2  
            ],
            [ # 4
                a2*cos1*cos2 - a4*(cos1*sin2*sin3 - cos1*cos2*cos3) \
                + a3*cos1*cos2*cos3 - a3*cos1*sin2*sin3,

                a2*cos2*sin1 - a4*(sin1*sin2*sin3 - cos2*cos3*sin1) \
                + a3*cos2*cos3*sin1 - a3*sin1*sin2*sin3,

                a1 + a4*(cos2*sin3 + cos3*sin2) + a2*sin2 + a3*cos2*sin3 + a3*cos3*sin2  
            ],
            [ # 5
                a2*cos1*cos2 - a4*(cos1*sin2*sin3 - cos1*cos2*cos3) + a5*sin5*(sin1*sin4 \
                - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - a5*cos5*(cos1*sin2*sin3 \
                - cos1*cos2*cos3) + a3*cos1*cos2*cos3 - a3*cos1*sin2*sin3,

                a2*cos2*sin1 - a5*cos5*(sin1*sin2*sin3 - cos2*cos3*sin1) - a4*(sin1*sin2*sin3 \
                - cos2*cos3*sin1) - a5*sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) \
                + a3*cos2*cos3*sin1 - a3*sin1*sin2*sin3,

                a1 + a4*(cos2*sin3 + cos3*sin2) + a2*sin2 + a5*cos5*(cos2*sin3 + cos3*sin2) \
                + a3*cos2*sin3 + a3*cos3*sin2 + a5*cos4*sin5*(cos2*cos3 - sin2*sin3)
            ],
            [ # 6 / EE
                a6*(sin5*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - cos5*(cos1*sin2*sin3 \
                - cos1*cos2*cos3)) - a4*(cos1*sin2*sin3 - cos1*cos2*cos3) + a2*cos1*cos2 \
                + a5*sin5*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) \
                - a5*cos5*(cos1*sin2*sin3 - cos1*cos2*cos3) + a3*cos1*cos2*cos3 - a3*cos1*sin2*sin3,

                a2*cos2*sin1 - a4*(sin1*sin2*sin3 - cos2*cos3*sin1) - a5*cos5*(sin1*sin2*sin3 
                - cos2*cos3*sin1) - a6*(sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) \
                + cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - a5*sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 \
                + cos3*sin1*sin2)) + a3*cos2*cos3*sin1 - a3*sin1*sin2*sin3,

                a1 + a4*(cos2*sin3 + cos3*sin2) + a2*sin2 + a6*(cos5*(cos2*sin3 + cos3*sin2) \
                + cos4*sin5*(cos2*cos3 - sin2*sin3)) + a5*cos5*(cos2*sin3 + cos3*sin2) \
                + a3*cos2*sin3 + a3*cos3*sin2 + a5*cos4*sin5*(cos2*cos3 - sin2*sin3)
            ]
        ]

        # x -> x is depth
        # y -> z is horizontal (positive is right)
        # z -> -y is vertical (negative is up)
        offset = self.lengths[0]
        joints = [
            {"x":0,           "z":0,           "y":0},
            {"x":joints[0][0],"z":joints[0][1],"y":joints[0][2]},
            {"x":joints[1][0],"z":joints[1][1],"y":joints[1][2]-offset},
            {"x":joints[2][0],"z":joints[2][1],"y":joints[2][2]-offset},
            {"x":joints[3][0],"z":joints[3][1],"y":joints[3][2]-offset},
            {"x":joints[4][0],"z":joints[4][1],"y":joints[4][2]-offset},
            {"x":joints[5][0],"z":joints[5][1],"y":joints[5][2]-offset}
        ]
        #{"x":joints[5][0],"z":joints[5][1],"y":-joints[5][2]}

        #print(joints)

        return joints

    # FIND_EE
    # This is useful for determining the position of the End Effector
    #  during startup of a physical arm. The alternative would be to
    #  set an EE position -- though this maybe dangerous.
    # returns a calculated EE position based on provided angles
    #  - should provide encoder read angles in an array []
    #  - angles in RADIANS
    def find_ee(self, angles):
        a1 = self.lengths[0]
        a2 = self.lengths[1]
        a3 = self.lengths[2]
        a4 = self.lengths[3]
        a5 = self.lengths[4]
        a6 = self.lengths[5]

        # angles in RADIANS
        t1 = angles[0]
        t2 = angles[1]
        t3 = angles[2]
        t4 = angles[3]
        t5 = angles[4]
        
        sin1 = np.sin(t1)
        sin2 = np.sin(t2)
        sin3 = np.sin(t3)
        sin4 = np.sin(t4)
        sin5 = np.sin(t5)

        cos1 = np.cos(t1)
        cos2 = np.cos(t2)
        cos3 = np.cos(t3)
        cos4 = np.cos(t4)
        cos5 = np.cos(t5)

        ee = [ # 6 / EE
            a6*(sin5*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - cos5*(cos1*sin2*sin3 \
            - cos1*cos2*cos3)) - a4*(cos1*sin2*sin3 - cos1*cos2*cos3) + a2*cos1*cos2 \
            + a5*sin5*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) \
            - a5*cos5*(cos1*sin2*sin3 - cos1*cos2*cos3) + a3*cos1*cos2*cos3 - a3*cos1*sin2*sin3,

            a2*cos2*sin1 - a4*(sin1*sin2*sin3 - cos2*cos3*sin1) - a5*cos5*(sin1*sin2*sin3 
            - cos2*cos3*sin1) - a6*(sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) \
            + cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - a5*sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 \
            + cos3*sin1*sin2)) + a3*cos2*cos3*sin1 - a3*sin1*sin2*sin3,

            a1 + a4*(cos2*sin3 + cos3*sin2) + a2*sin2 + a6*(cos5*(cos2*sin3 + cos3*sin2) \
            + cos4*sin5*(cos2*cos3 - sin2*sin3)) + a5*cos5*(cos2*sin3 + cos3*sin2) \
            + a3*cos2*sin3 + a3*cos3*sin2 + a5*cos4*sin5*(cos2*cos3 - sin2*sin3)
        ]

        return ee

    def move(self, dir, amount):

        v = dir
        n = amount
        u = self.current_ee

        # direction
        x1 = v[0]
        y1 = v[1]
        z1 = v[2]

        # current position
        x0 = u[0]
        y0 = u[1]
        z0 = u[2]

        # magnitude
        m = (x1**2 + y1**2 + z1**2)**0.5

        # v/mag_v * amount + u
        # (step in the direction)
        new_ee = [
            n/m * x1 + x0,
            n/m * y1 + y0,
            n/m * z1 + z0
        ]

        """ print("============")
        print("new_ee",new_ee)
        show_angles(self.joint_angles) """
        
        self.solve(new_ee)
    
    def show_angles(self):
        angles = self.joint_angles
        t1 = angles[0]
        t2 = angles[1]
        t3 = angles[2]
        t4 = angles[3]
        t5 = angles[4]
        t6 = angles[5]
        print('Theta 1: ',round(t1*180/np.pi,5))
        print('Theta 2: ',round(t2*180/np.pi,5))
        print('Theta 3: ',round(t3*180/np.pi,5))
        print('Theta 4: ',round(t4*180/np.pi,5))
        print('Theta 5: ',round(t5*180/np.pi,5))
        print('Theta 6: ',round(t6*180/np.pi,5))

    # SOLVE
    # Accepts a position (optional) and rotation (optional) of the End Effector
    #  - if no position is given, a default will be provided
    # Calculates angles of Joints 1-6
    # (WIP) Checks for singularities before returning a solution
    # returns joint angles in RADIANS
    def solve(self, pos=None, rot=None):
        # GIVEN Position (mm)
        if pos == None: 
            pos = self.current_ee

        ee_x = pos[0]
        ee_y = pos[1]
        ee_z = pos[2]

        # GIVEN Rotation Angles (degrees)
        if rot == None: 
            rot = self.orientation

        tr = rot[0] # Theta roll
        ty = rot[1] # Theta yaw
        tp = rot[2] # Theta pitch

        tr = tr * np.pi / 180 # (radians)
        ty = ty * np.pi / 180
        tp = tp * np.pi / 180

        # R0_6choose
        R0_6choose = [
            [
                np.cos(tr)*np.sin(tp) + np.cos(tp)*np.sin(tr)*np.sin(ty),
                np.cos(ty)*np.sin(tr),
                np.cos(tp)*np.cos(tr) - np.sin(tp)*np.sin(tr)*np.sin(ty)
            ],
            [
                np.sin(tp)*np.sin(tr) - np.cos(tp)*np.cos(tr)*np.sin(ty),
                -np.cos(tr)*np.cos(ty),
                np.cos(tp)*np.sin(tr) + np.cos(tr)*np.sin(tp)*np.sin(ty)
            ],
            [
                np.cos(tp)*np.cos(ty),
                -np.sin(ty),
                -np.cos(ty)*np.sin(tp)
            ]
        ]
        
        # Direction of Limb D6
        Z_frame = np.array([[R0_6choose[2][0]],[R0_6choose[2][1]],[R0_6choose[2][2]]])

        # EE position in mm
        p_e=np.array([
            [ee_x],
            [ee_y],
            [ee_z]
        ])
        p_c=np.subtract(p_e, self.D6 * Z_frame) # wrist center in mm

        # Diagonal length to P_c in XY-plane
        p_cd=np.sqrt(
            p_c[0]**2 + p_c[1]**2
        )[0]

        # Height from P_1 to P_c
        height_p1c=p_c[2] - self.L1 

        # Hypotenuse length from P_1 to P_c
        w=np.sqrt(
            p_cd**2 + height_p1c**2
        )[0]

        # Angle between X_0 and w
        gamma=np.arctan2(height_p1c, p_cd)[0]

        # Angle between w and L2
        alpha=np.arccos(
            (self.D4**2 - (self.L2**2 + w**2)) / 
            (-2*self.L2*w)
        )

        # P_2 position in arm plane (X_0, Y_0)
        p_2=[
            self.L2*np.cos(alpha + gamma), # p_2d
            self.L1 + self.L2*np.sin(alpha + gamma)  # p_2z
        ]

        # Relative X and Y components of Limbs 2 and D4
        LA=p_2[0]
        LB=p_c[2] - p_2[1]

        LC=p_cd - p_2[0]
        LD=-p_2[1] + p_c[2]

        # Theta 1 in radians
        if(p_c[0]==0 and p_c[1]>0): # rotated 90 degrees left
            t1=np.pi
        elif(p_c[0] and p_c[1]<0): # rotated 90 degrees right
            t1=-np.pi
        elif(p_c[0]==0 and p_c[1]==0): # in vertical position (this depends on ee position actually)
            t1=0
        else:
            t1=np.arctan2(p_c[1], p_c[0])[0]

        # Theta 2 in radians
        if(LA==0):
            t2=0
        else:
            t2=-np.arctan2(LB, LA)[0]

        # Theta 3 in radians
        if(LC==0): 
            t3=-t2
        else: 
            t3=np.arctan2(LD, LC)[0] - t2

        # R0_3 Actual
        R0_3 = [
            [
                -np.cos(t1)*np.cos(t2)*np.sin(t3) - np.cos(t1)*np.cos(t3)*np.sin(t2),
                np.sin(t1),
                np.cos(t1)*np.cos(t2)*np.cos(t3) - np.cos(t1)*np.sin(t2)*np.sin(t3)
            ],
            [
                -np.cos(t2)*np.sin(t1)*np.sin(t3) - np.cos(t3)*np.sin(t1)*np.sin(t2),
                -np.cos(t1),
                np.cos(t2)*np.cos(t3)*np.sin(t1) - np.sin(t1)*np.sin(t2)*np.sin(t3)
            ],
            [
                np.cos(t2)*np.cos(t3) - np.sin(t2)*np.sin(t3),
                0,
                np.cos(t2)*np.sin(t3) + np.cos(t3)*np.sin(t2)
            ]
        ]

        # R3_6 Actual
        R3_6 = np.dot(np.linalg.inv(R0_3), R0_6choose)

        # Theta 5
        t5 = np.arccos(R3_6[2][2])

        # Theta 6 and 4
        if(t5 != 0):
            t6 = np.arcsin(R3_6[2][1]/np.sin(t5))
            t4 = np.arcsin(R3_6[1][2]/np.sin(t5))
        else:
            t6 = 0
            t4 = 0

        singular = self.find_singularity([t1,t2,t3,t4,t5,t6])

        if singular != 0 and np.isnan(singular) == False: # this way you can check if the move is valid
            self.joint_angles = [t1,t2,t3,t4,t5,t6]
            self.current_ee = pos
            self.orientation = rot
        else:
            all_joints = self.get_joint_positions()
            print("------")
            print("Singularity found!", singular)
            print("Cur EE",self.current_ee)
            print("Request",pos)
            print("Calc EE",all_joints[6])
            print("All joints",all_joints)
            self.show_angles()

        return self.joint_angles

if __name__ == '__main__':

    arm = ikSolver()
    #config = arm.solve(pos=[275,10,60], rot=[0,0,0])
    arm.solve()

    # Output
    arm.show_angles()
    print("EE:",arm.current_ee)
    print("Positions:",arm.get_joint_positions())