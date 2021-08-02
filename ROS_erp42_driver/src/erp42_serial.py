#!/usr/bin/env python

import rospy
import serial
# from multiprocessing import Queue
import geometry_msgs.msg
from std_msgs.msg import Float32
from erp42_core.msg import DriveCmd
from erp42_core.msg import SerialFeedBack
import math
import time
import signal
import threading

exitThread = 0
wheelbase = 1.040
wheeltrack = 0.985
TICK2RAD = 0.06283185307
wheel_radius = 0.265

class control():
    def __init__(self, port_num):

        S = 83
        T = 84
        X = 88
        AorM = 1
        ESTOP = 0
        ETX0 = 13
        ETX1 = 10
        ALIVE = 0

        self.last_encoder = 0
        self.feedback = SerialFeedBack()

        self.port_num = port_num

        self.DATA = bytearray(14)

        self.DATA[0] = S
        self.DATA[1] = T
        self.DATA[2] = X
        self.DATA[3] = AorM
        self.DATA[4] = ESTOP
        self.DATA[12] = ETX0
        self.DATA[13] = ETX1

        self.ALIVE = ALIVE
        self.ser = serial.Serial(
            port=self.port_num,
            baudrate=115200,
            # timeout=0
        )
        
        thread = threading.Thread(target=self.receive_data)
        thread.deamon = True
        thread.start()
        

    def send_data(self, SPEED, STEER, BRAKE, GEAR):
        if STEER >= 0:
            self.DATA[8] = STEER // 256
            self.DATA[9] = STEER % 256
        else:
            STEER = -STEER
            self.DATA[8] = 255 - STEER // 256
            self.DATA[9] = 255 - STEER % 256

        self.DATA[5] = GEAR
        self.DATA[6] = SPEED // 256
        self.DATA[7] = SPEED % 256
        self.DATA[10] = BRAKE
        self.DATA[11] = self.ALIVE

        self.ser.write(bytes(self.DATA))
        print(self.ALIVE)
        self.ALIVE = self.ALIVE + 1
        if self.ALIVE is 256:
            self.ALIVE = 0

    def receive_data(self):
        line = []
        while not exitThread:

            for i in self.ser.read():
                line.append(i)
                if ord(i) == 83:
                    del line[:]
                    line.append(i)
                elif ord(i) == 10 and ord(line[-1]) == 10 and len(line) == 18:
                    self.handle_data(line)
                    del line[:]
                    break
                if len(line) >= 18:
                    del line[:]
                    break

            
    def handle_data(self, line):
        self.feedback.AorM = ord(line[4])
        self.feedback.Gear = ord(line[5])

        self.feedback.KPH = (ord(line[6]) + ord(line[7]) * 256)/10  #nominal speed

        self.feedback.Deg = ord(line[8]) + ord(line[9]) * 256
        
        if self.feedback.Deg >= 23768:
            self.feedback.Deg -= 65536

        if self.feedback.Deg != 0 :
            self.feedback.Deg /= 71.0

        self.feedback.Brake = ord(line[10])

        self.feedback.Encoder = (ord(line[11]) + ord(line[12]) * 256 + ord(line[13]) * 256 * 256 + ord(line[14]) * 256 * 256 * 256)
        if self.feedback.Encoder >= 2147483648: 
            self.feedback.Encoder -= 4294967296

        pub.publish(self.feedback)
        print(self.feedback)

    def callback_cmd(self, msg):

        Speed = msg.KPH * 10
        if Speed > 200:
            Speed = 200
        elif Speed < 0:
            Speed = 0

        Steer = msg.Deg * 71
        if Steer > 1999:
            Steer = 1999
        if Steer < -1999:
            Steer = -1999
            
        Brake = msg.Brake
        if msg.Brake > 200:
            Brake = 200

        Gear = msg.Gear

        
        self.send_data(int(Speed), int(Steer), Brake, Gear)

    

if __name__ == "__main__":
    rospy.init_node('erp42_serial')

    port = rospy.get_param('~port','/dev/ttyUSB0')

    pub = rospy.Publisher('/erp42_feedback', SerialFeedBack, queue_size=1)
    mycar = control(port)
    rospy.Subscriber('/erp42_command', DriveCmd, mycar.callback_cmd)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        
        rate.sleep()

    exitThread = 1
