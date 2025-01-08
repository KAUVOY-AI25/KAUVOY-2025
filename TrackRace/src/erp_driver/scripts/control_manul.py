#! /usr/bin/env python3

import sys
import select
sys.path.append(r'/home/kauvoy/anaconda3/lib/python3.8/site-packages')

import rospy
from erp_driver.msg import erpCmdMsg
import serial
import struct
import numpy as np
from ByteHandler import ErpMsg2Packet

class ERPHandler:
    def __init__(self, port, baudrate) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.alive = 0
        self.cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)
        self.packet = erpCmdMsg()
        self.packet.e_stop = False
        self.packet.gear = 0
        self.packet.speed = 0
        self.packet.steer = 0
        self.packet.brake = 1

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

def get_user_input() -> (str, str):
    field = input("Enter field to update (e_stop, gear, speed, steer, brake): ")
    value = input(f"Enter new value for {field}: ")
    return field, value

if __name__ == "__main__":
    rospy.init_node("erp_cmd_sender")
    port = rospy.get_param("/erp_base/port")
    baudrate = rospy.get_param("/erp_base/baudrate")
    
    erp_handler = ERPHandler(port, baudrate)
    
    while not rospy.is_shutdown():
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            field, value = get_user_input()
            erp_handler.update_cmd(field, value)
            erp_handler.publish_cmd()  # Publish the updated command
        
        erp_handler.send_packet()  # Send the packet via serial
        rospy.sleep(0.1)
