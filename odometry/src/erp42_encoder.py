#!/usr/bin/env python

import rospy
import serial
import threading
import math as m
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from path_planner.msg import stanleyMsg
from odometry.msg import encoderMsg

# Params

exitThread = 0
wheelbase = 1.040
wheeltrack = 0.985
TICK2RAD = 0.06283185307
wheel_radius = 0.265

p_gain = rospy.get_param("p_gain", 0.0)


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

        self.doPIControl = True

        self.stanley_control = stanleyMsg()
        self.feedbackMsg = encoderMsg()
        self.control_msg = stanleyMsg()

        self.feedback_encoder = 0

        """
            Encoder variable
        """

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
            port=port_num,
            baudrate=115200,
        )

    """ Util """

    def kph2mps(self, value):
        return value * 0.277778

    def mps2kph(self, value):
        return value * 3.6

    """ Serial Read """

    def receive_data(self):
        line = []
        while not exitThread:
            try:
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

            except Exception as ex:
                print(ex)

    def handle_data(self, line):
        """
            Transfer packet data to AorM / Gear / Speed(m/s) / Steer(rad)
        """

        # AorM - 0: M / 1: A
        feedback_AorM = ord(line[4])

        # Gear - 0: B / 1: N / 2: D
        feedback_gear = ord(line[5])

        # Speed
        feedback_KPH = (ord(line[6]) + ord(line[7]) * 256) / 10
        feedback_speed = self.kph2mps(value=feedback_KPH)

        # Steer
        feedback_DEG = ord(line[8]) + ord(line[9]) * 256

        if feedback_DEG >= 23768:
            feedback_DEG -= 65536

        if feedback_DEG != 0:
            feedback_DEG /= 71.0

        feedback_steer = m.radians(feedback_DEG)

        # Brake
        feedback_brake = ord(line[10])

        # Encoder
        self.feedback_encoder = (ord(line[11]) + ord(line[12]) * 256 + ord(
            line[13]) * 256 * 256 + ord(line[14]) * 256 * 256 * 256)

        if self.feedback_encoder >= 2147483648:
            self.feedback_encoder -= 4294967296

        # print(self.feedback_encoder)

        data = encoderMsg()

        data.speed = feedback_speed
        data.steer = feedback_steer
        data.steer = feedback_DEG
        data.brake = feedback_brake
        data.gear = feedback_gear
        data.AorM = feedback_AorM
        data.Encoder = self.feedback_encoder

        self.feedbackMsg = data
        encoder_pub.publish(data)

    """ Serial Write"""

    def cmd_vel_callback(self, msg):
        self.control_msg = msg

    def PIControl(self, currentSpeed, desiredSpeed):
        p = p_gain
        err = desiredSpeed - currentSpeed

        res = desiredSpeed + p * err

        if res < 0.0:
            return 0.0

        return res

    def send_data(self, SPEED, STEER, BRAKE, GEAR):
        """
            Function to send serial to ERP42 with
            speed(KPH), steer(Deg), Brake(1~200), Gear(2: drive)
        """

        if self.doPIControl is True:

            current_speed = self.mps2kph(self.feedbackMsg.speed)    # kph
            desired_speed = SPEED  # kph

            SPEED = self.PIControl(
                currentSpeed=current_speed, desiredSpeed=desired_speed)

        GEAR = 2 if SPEED >= 0.0 else 0

        SPEED = abs(SPEED) * 10
        if SPEED > 200:
            SPEED = 200
        elif SPEED < 0:
            SPEED = 0

        STEER = STEER * 71
        if STEER > 1999:
            STEER = 1999
        if STEER < -1999:
            STEER = -1999

        try:

            if STEER >= 0:
                self.DATA[8] = int(STEER // 256)
                self.DATA[9] = int(STEER % 256)
            else:
                STEER = -STEER
                self.DATA[8] = int(255 - STEER // 256)
                self.DATA[9] = int(255 - STEER % 256)

            self.DATA[5] = GEAR    # GEAR
            self.DATA[6] = int(SPEED // 256)
            self.DATA[7] = int(SPEED % 256)
            self.DATA[10] = BRAKE   # BREAK
            self.DATA[11] = self.ALIVE

            self.ser.write((self.DATA))

            self.ALIVE = self.ALIVE + 1
            if self.ALIVE == 256:
                self.ALIVE = 0

        except Exception as ex:
            print(ex)

    def distanceCallback(self, msg):
        distance = msg.data

        if distance < 3.0:
            self.distance_ratio = (1 / 3) * distance
        else:
            self.distance_ratio = 1.0

        if distance < 1.0:
            brake = int(-200.0 * distance + 200.0)

            if brake < 1:
                brake = 1

        else:
            brake = 1

        self.control_brake = brake


if __name__ == "__main__":
    rospy.init_node('erp42_encoder')

    """ Param """
    port = rospy.get_param('/erp_port', '/dev/ttyUSB0')

    """ Object """
    mycar = control(port_num=port)

    thread = threading.Thread(target=mycar.receive_data)
    thread.deamon = True
    thread.start()

    """ Publisher """
    encoder_pub = rospy.Publisher(
        '/erp42_encoder', encoderMsg, queue_size=1)

    """ Subscriber"""
    rospy.Subscriber("/Control_msg", stanleyMsg, mycar.cmd_vel_callback)
    # rospy.Subscriber("/laser_distance", Float32, mycar.distanceCallback)

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():

        sp = mycar.control_msg.speed
        st = m.degrees(mycar.control_msg.steer)
        br = int(mycar.control_msg.brake)

        print(sp, st, br)

        mycar.send_data(SPEED=(sp), STEER=(st), BRAKE=(br), GEAR=2)

        rate.sleep()

    exitThread = 1
