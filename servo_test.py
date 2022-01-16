
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

serial = serial.Serial("COM3", baudrate=115200, timeout=0.001)
serial.setDTR(1)

# neutral position of arm (6) and EE (7th)
#n_pos = [915, 550, 300, 455, 800, 500, 570]
n_pos = [900,700,200,450,800,500,300]

def sendPacket(packet):
    packet1=bytearray(packet);sum=0
    for a in packet1: sum=sum+a
    fullPacket = bytearray(struct.pack("<BB",0x55,0x55) + packet + struct.pack("<B",(~sum) & 0xff))
    serial.write(fullPacket); sleep(0.00002)

def moveServo(id,position,rate):# Move servo 0-1000, rate(ms) 0-30000(slow)
    sendPacket(struct.pack("<BBBHH",id,7,1,position,rate))

def moveservos(speed,s1,s2,s3,s4,s5,s6,s7):
    s=(s1,s2,s3,s4,s5,s6,s7)
    for a in range(7): moveServo(a+1,n_pos[a]+s[a],speed)

""" def arm_position(speed,s1,s2,s3,s4,s5,s6,s7):
    s=(s1,s2,s3,s4,s5,s6,s7)
    for a in range(7): moveServo(a+1,s[a],speed) """
def arm_position(speed,s1,s2,s3,s4,s5,s6,s7):
    moveServo(7,s7,speed)
    moveServo(6,s6,speed)
    moveServo(5,s5,speed)
    moveServo(4,s4,speed)
    moveServo(3,s3,speed)
    moveServo(2,s2,speed)
    moveServo(1,s1,speed)

def sendReceivePacket(packet,receiveSize):
    t_id = packet[0];t_command = packet[2]
    serial.flushInput();serial.timeout=0.1;sendPacket(packet)
    r_packet = serial.read(receiveSize+3); return r_packet

def readPosition(id):
    rpacket = sendReceivePacket(struct.pack("<BBB",id,3,28),5)
    s = struct.unpack("<BBBBBhB",rpacket);return s[5]

def scan():
    for i in range(7): print(readPosition(i+1))

if __name__ == '__main__':
    try:
        print("Resetting servos...")
        arm_position(200, n_pos[0],n_pos[1],n_pos[2],n_pos[3],n_pos[4],n_pos[5],n_pos[6])

        l1=130
        l2=130
        d4=160
        d6=140
        zero_joint=0

        arm = IK(l1, l2, d4, d6, zero_joint)
        arm.solve()

        angles = arm.joint_angles
        t1 = math.floor(angles[0]  * (180/math.pi) * (1000/270) + 765)
        t2 = math.floor(-angles[1] * (180/math.pi) * (1000/270) + 759)
        t3 = math.floor(angles[2]  * (180/math.pi) * (1000/270) + 1000)
        t4 = math.floor(angles[3]  * (180/math.pi) * (1000/270) + 450)
        t5 = math.floor(-angles[4] * (180/math.pi) * (1000/270) + 410)
        t6 = math.floor(angles[5]  * (180/math.pi) * (1000/270) + 500)

        if(
            t1 > 1000 or t1 < 530 or 
            t2 > 700  or t2 < 0   or 
            t3 > 1000 or t3 < 200 or 
            t4 > 900  or t4 < 0   or 
            t5 > 850  or t5 < 0   or 
            t6 > 1000 or t6 < 0   
        ):
            print("out of range!")

        else: 
            sleep(3)
            print("Moving servos...")
            # towards (right), up, up, right, up, right, close
            #moveservos(200, -100, -200, 100, 100, 100, 100, -100)
            #arm_position(200, 765, 662, 759, 450, 266, 500, n_pos[6])
            #print(t1)
            #print(t2)
            #print(t3)
            #print(t4)
            #print(t5)
            #print(t6)
            arm_position(200, t1, t2, t3, t4, t5, t6, n_pos[6])

        sleep(3)
        print("Resetting servos...")
        arm_position(200, n_pos[0],n_pos[1],n_pos[2],n_pos[3],n_pos[4],n_pos[5],n_pos[6])
    except KeyboardInterrupt:
        serial.close()
        print("Connection closed.")
