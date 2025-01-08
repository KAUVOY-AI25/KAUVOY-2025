#! /usr/bin/env python3

import sys
import select
import time
sys.path.append(r'/home/kauvoy/anaconda3/lib/python3.8/site-packages')

import rospy
from erp_driver.msg import erpCmdMsg, erpStatusMsg  # Import the erpStatusMsg
from std_msgs.msg import Float64
import serial
import struct
import numpy as np
from ByteHandler import ErpMsg2Packet

# Fixed variable values
SPEED = 60 #IMPORTANT! TrackRACE = 7 km/s
STEER = 0
BRAKE = 1
ESTOP = False
GEAR = 0

class ERPHandler:
    def __init__(self, port, baudrate) -> None:
        #self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.alive = 0
        self.cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)
        self.status_sub = rospy.Subscriber('/erp42_status', erpStatusMsg, self.status_callback)  # Subscribe to the ERP status message
        self.steer_sub = rospy.Subscriber('/steering_angle', Float64, self.steer_callback)  # Subscribe to the cmd_degree topic
        self.packet = erpCmdMsg()
        self.update_cmd('e_stop', ESTOP)
        self.update_cmd('gear', GEAR)
        self.update_cmd('speed', SPEED)
        self.update_cmd('steer', STEER)
        self.update_cmd('brake', BRAKE)

        # Initialize steering
        self.packet.steer = 0

    def send_packet(self) -> None:
        packet = ErpMsg2Packet(self.packet, self.alive)
        self.serial.write(packet)
        self.alive = (self.alive + 1) % 256

    def publish_cmd(self) -> None:
        self.cmd_pub.publish(self.packet)

    def update_cmd(self, field, value):
        if field == 'e_stop':
            self.packet.e_stop = bool(int(value))
        elif field == 'gear':
            self.packet.gear = int(value)
        elif field == 'speed':
            self.packet.speed = int(value)
        elif field == 'steer':
            self.packet.steer = int(value)
        elif field == 'brake':
            self.packet.brake = int(value)

    def status_callback(self, msg):
        self.current = msg.steer
        print("current_steer: ", self.current)

    def steer_callback(self, msg):
        if msg.data >= 28:
            new_steer_value = 2000
        else:
            new_steer_value = (msg.data-0.5)*(71)
             
        self.update_cmd('steer', new_steer_value) #pure pursuit
        print("pure pursuit steer :", msg.data, "cmd_steer :", new_steer_value)


if __name__ == "__main__":
    rospy.init_node("erp_cmd_sender")
    # port = rospy.get_param("/erp_base/port")
    # baudrate = rospy.get_param("/erp_base/baudrate")
    port, baudrate = "/dev/ttyERP", 115200
    
    erp_handler = ERPHandler(port, baudrate)
    
    while not rospy.is_shutdown():
        erp_handler.publish_cmd()  
        erp_handler.send_packet()  
        rospy.sleep(0.1)

