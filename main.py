# pyPS4Controller added who knows when
from pyPS4Controller.controller import Controller

# 8/18/21
# LX-16A Servo test for 6DoF Arm, based on code by Sujit Vasanth

# simplified lx16a servo communication.. altered by Sujit Vasanth
# modified from danjperron's LX16A library https://github.com/danjperron/LX16A/blob/master/LX16A.py
# see youtube videos at https://www.youtube.com/watch?v=isUQWM0gjo4 and https://www.youtube.com/watch?v=XBxGyKV4W7U0000
from time import sleep
import serial
import struct
from ikSolver import ikSolver as IK
import math
import numpy as np

#from PyQt5.QtCore import Qt
# pip3 install pynput
from pynput import keyboard

class MyController(Controller):

        def __init__(self, **kwargs):
            Controller.__init__(self, **kwargs)

        def on_x_press(self):
            print("Lift")
            arm.move(up, step)
            update_arm()

        def on_x_release(self):
            print("Lift halt")

        def on_circle_press(self):
            print("Lower")
            arm.move(down, step)
            update_arm()

        def on_circle_release(self):
            print("Lower halt")

        def on_L3_up(self, value):
            print("EXTEND")
            arm.move(forward, step)
            update_arm()

        def on_L3_down(self, value):
            print("CONTRACT")
            arm.move(backward, step)
            update_arm()

        def on_L3_y_at_rest(self):
            print("HALT")

        def on_L2_left(self, value):
            print("LEFT")
            arm.move(left, step)
            update_arm()

        def on_L2_right(self, value):
            print("RIGHT")
            arm.move(right, step)
            update_arm()

        def on_L2_x_at_rest(self):
            print("HALT")

def on_press(key):
    try:
        #print('Alphanumeric key pressed: {0} '.format(key.char))

        if key.char == 'r':
            reset_arm()

        else:
            direction = []
            if key.char == 'a':
                #print("Strafe left")
                direction = left

            if key.char == 'd':
                #print("Strafe right")
                direction = right

            if key.char == 'w':
                #print("Go forward")
                direction = forward

            if key.char == 's':
                #print("Go backward")
                direction = backward

            if key.char == 'f':
                #print("Go up")
                direction = up

            if key.char == 'c':
                #print("Go down")
                direction = down

            arm.move(direction, step)
            update_arm()
            
    except AttributeError:
        print('special key pressed: {0}'.format(key))

def update_arm():
    angles = arm.joint_angles
    t1 = math.floor(angles[0]  * (180/math.pi) * (1000/270) + 765)
    t2 = math.floor(-angles[1] * (180/math.pi) * (1000/270) + 759)
    t3 = math.floor(angles[2]  * (180/math.pi) * (1000/270) + 1000)
    t4 = math.floor(angles[3]  * (180/math.pi) * (1000/270) + 450)
    t5 = math.floor(angles[4]  * (180/math.pi) * (1000/270) + 360)
    t6 = math.floor(angles[5]  * (180/math.pi) * (1000/270) + 500)

    """ show_info(
        readPosition(1),
        readPosition(2),
        readPosition(3),
        readPosition(4),
        readPosition(5),
        readPosition(6)
    ) """

    arm.last_confirmed = arm.current_ee
    arm_position(200, t1, t2, t3, t4, t5, t6, n_pos[6])

    # joint limits. protect the arm!
    if  (t1 > 1000 or t1 < 530):
        print("JOINT 1: Out of range!",readPosition(1))
        arm.solve(arm.last_confirmed)
        #reset_arm()
    elif(t2 > 800  or t2 < 200  ):
        print("JOINT 2: Out of range!",readPosition(2))
        arm.solve(arm.last_confirmed)
        #reset_arm()
    elif(t3 > 1000 or t3 < 200):
        print("JOINT 3: Out of range!",readPosition(3))
        arm.solve(arm.last_confirmed)
        #reset_arm()
    elif(t4 > 900  or t4 < 0  ):
        print("JOINT 4: Out of range!",readPosition(4))
        arm.solve(arm.last_confirmed)
        #reset_arm()
    elif(t5 > 800  or t5 < 50 ):
        print("JOINT 5: Out of range!",readPosition(5))
        arm.solve(arm.last_confirmed)
        #reset_arm()
    elif(t6 > 1000 or t6 < 0  ):
        print("JOINT 6: Out of range!",readPosition(6))
        arm.solve(arm.last_confirmed)
        #reset_arm() 

    else: 
        # SUCCESSFUL CALCULATION! Within limits
        # towards (right), up, up, right, up, right, close
        #moveservos(200, -100, -200, 100, 100, 100, 100, -100)
        #arm_position(200, 765, 662, 759, 450, 266, 500, n_pos[6])
        arm.last_confirmed = arm.current_ee
        arm_position(250, t1, t2, t3, t4, t5, t6, n_pos[6])

def show_info(t1,t2,t3,t4,t5,t6):
    print("=====")
    print(t1)
    print(t2)
    print(t3)
    print(t4)
    print(t5)
    print(t6)

def on_release(key):
    #print('Key released: {0}'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def reset_arm():
    print("Resetting servos...")
    arm.solve(arm.init_ee)
    arm_position(200, n_pos[0],n_pos[1],n_pos[2],n_pos[3],n_pos[4],n_pos[5],n_pos[6])
    sleep(2)
    print("Complete!")

def sendPacket(packet):
    packet1=bytearray(packet);sum=0
    for a in packet1: sum=sum+a
    fullPacket = bytearray(struct.pack("<BB",0x55,0x55) + packet + struct.pack("<B",(~sum) & 0xff))
    serial.write(fullPacket); sleep(0.00002)

def moveServo(id,position,rate):# Move servo 0-1000, rate(ms) 0-30000(slow)
    message = struct.pack("<BBBHH",id,7,1,position,rate)
    #print(message)
    sendPacket(message)

def moveservos(speed,s1,s2,s3,s4,s5,s6,s7):
    s=(s1,s2,s3,s4,s5,s6,s7)
    for a in range(7): moveServo(a+1,n_pos[a]+s[a],speed)

""" def arm_position(speed,s1,s2,s3,s4,s5,s6,s7):
    s=(s1,s2,s3,s4,s5,s6,s7)
    for a in range(7): moveServo(a+1,s[a],speed) """
def arm_position(speed,s1,s2,s3,s4,s5,s6,s7):
    moveServo(1,s1,speed)
    moveServo(2,s2,speed)
    moveServo(3,s3,speed)
    moveServo(4,s4,speed)
    moveServo(5,s5,speed)
    print("Servo 5",s5)
    moveServo(6,s6,speed)
    moveServo(7,s7,speed)

def sendReceivePacket(packet,receiveSize):
    t_id = packet[0];t_command = packet[2]
    serial.flushInput();serial.timeout=0.1;sendPacket(packet)
    r_packet = serial.read(receiveSize+3); return r_packet

def readPosition(id):
    rpacket = sendReceivePacket(struct.pack("<BBB",id,3,28),5)
    s = struct.unpack("<BBBBBhB",rpacket);return s[5]

def init_arm():
    l1=130
    l2=130
    d4=160
    d6=140
    zero_joint=0

    init = IK(l1, l2, d4, d6, zero_joint)
    # get initial angles of joints
    t1 = math.floor(( n_pos[0] - 765 ) * (270/1000) * (np.pi/180))
    t2 = math.floor((-n_pos[1] + 759 ) * (270/1000) * (np.pi/180))
    t3 = math.floor(( n_pos[2] - 1000) * (270/1000) * (np.pi/180))
    t4 = math.floor(( n_pos[3] - 450 ) * (270/1000) * (np.pi/180))
    t5 = math.floor(( n_pos[4] + 360 ) * (270/1000) * (np.pi/180))
    t6 = math.floor(( n_pos[5] - 500 ) * (270/1000) * (np.pi/180))

    # IK solve
    init_ee = init.find_ee([t1,t2,t3,t4,t5,t6])
    init.solve(init_ee)
    return init

# GLOBAL VARIABLES
serial = serial.Serial("COM3", baudrate=115200, timeout=0.001)
serial.setDTR(1)

# neutral position of arm (6) and EE (7th)
n_pos = [900,700,200,450,700,500,300]
arm = init_arm()
step = 5

# UP/DOWN
up = [0,0,1]
down = [0,0,-1]

# FORWARD/BACK
forward = [1,0,0]
backward = [-1,0,0]

# LEFT/RIGHT
left = [0,-1,0]
right = [0,1,0]

if __name__ == '__main__':
    try:
        #controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        #controller.listen()

        reset_arm()

        ee = arm.init_ee
        rot = [1,0,0]
        delay = 0.6
        while True:
            print("Position 1")
            arm.solve([ee[0]-150,ee[1],ee[2]+50],rot)
            update_arm()
            arm.show_angles()
            sleep(delay)
            print("Position 2")
            arm.solve([ee[0]-200,ee[1]+100,ee[2]+50],rot)
            update_arm()
            arm.show_angles()
            sleep(delay)
            
            print("Position 3")
            arm.solve([ee[0]-200,ee[1]+100,ee[2]+100],rot)
            update_arm()
            arm.show_angles()
            sleep(delay)
            print("Position 4")
            arm.solve([ee[0]-150,ee[1],ee[2]+100],rot)
            update_arm()
            arm.show_angles()
            sleep(delay)

        """ # KEYBOARD LISTENER FOR TESTING
        # Collect events until released
        with keyboard.Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            listener.join() """

    except KeyboardInterrupt:
        reset_arm()
        serial.close()
        print("Connection closed.")