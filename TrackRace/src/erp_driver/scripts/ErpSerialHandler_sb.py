#!/usr/bin/env python3

from struct import pack
import rospy
from erp_driver.msg import erpStatusMsg, erpCmdMsg
from std_msgs.msg import Int8, Bool, UInt8, Int32

import serial
import numpy as np

from ByteHandler import ErpMsg2Packet, Packet2ErpMsg

START_BITS = "535458"

class ERPHandler:
    def __init__(self) -> None:
        rospy.init_node("erp_base")
        _port = rospy.get_param("/erp_base/port")
        _baudrate = rospy.get_param("/erp_base/baudrate")
        rospy.loginfo("erp_base::Uart Port : %s", _port)
        rospy.loginfo("erp_base::Baudrate  : %s", _baudrate)

        self.serial = serial.Serial(port=_port, baudrate=_baudrate)
        rospy.loginfo("Serial %s Connected", _port)
        self.alive = 0
        self.packet = erpCmdMsg()
        self.packet.gear = 0
        self.packet.e_stop = False
        self.packet.brake = 1

        # Publishers for individual factors
        self.control_mode_pub = rospy.Publisher("/erp42_control_mode", Int8, queue_size=3)
        self.e_stop_pub = rospy.Publisher("/erp42_e_stop", Bool, queue_size=3)
        self.gear_pub = rospy.Publisher("/erp42_gear", UInt8, queue_size=3)
        self.speed_pub = rospy.Publisher("/erp42_speed", UInt8, queue_size=3)
        self.steer_pub = rospy.Publisher("/erp42_steer", Int32, queue_size=3)
        self.brake_pub = rospy.Publisher("/erp42_brake", UInt8, queue_size=3)
        self.encoder_pub = rospy.Publisher("/erp42_encoder", Int32, queue_size=3)
        self.alive_pub = rospy.Publisher("/erp42_alive", UInt8, queue_size=3)

        self.erpCmdMsg_sub = rospy.Subscriber(
            "/erp42_ctrl_cmd", erpCmdMsg, self.sendPacket
        )

    def recvPacket(self) -> None:
        packet = self.serial.read(18)
        if not packet.hex().find(START_BITS) == 0:
            end, data = packet.hex().split(START_BITS)
            packet = bytes.fromhex(START_BITS + data + end)
            status_msg = Packet2ErpMsg(packet)
            
            # Publish each factor individually
            self.control_mode_pub.publish(status_msg.control_mode)
            self.e_stop_pub.publish(status_msg.e_stop)
            self.gear_pub.publish(status_msg.gear)
            self.speed_pub.publish(status_msg.speed)
            self.steer_pub.publish(status_msg.steer)
            self.brake_pub.publish(status_msg.brake)
            self.encoder_pub.publish(status_msg.encoder)
            self.alive_pub.publish(status_msg.alive)

        else:
            status_msg = Packet2ErpMsg(packet)
            
            # Publish each factor individually
            self.control_mode_pub.publish(status_msg.control_mode)
            self.e_stop_pub.publish(status_msg.e_stop)
            self.gear_pub.publish(status_msg.gear)
            self.speed_pub.publish(status_msg.speed)
            self.steer_pub.publish(status_msg.steer)
            self.brake_pub.publish(status_msg.brake)
            self.encoder_pub.publish(status_msg.encoder)
            self.alive_pub.publish(status_msg.alive)

    def sendPacket(self, _data: erpCmdMsg) -> None:
        self.packet = _data

    def serialSend(self) -> None:
        packet = ErpMsg2Packet(self.packet, self.alive)
        self.serial.write(packet)
        self.alive += 1
        if self.alive == 256:
            self.alive = 0


if __name__ == "__main__":
    ehandler = ERPHandler()
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        ehandler.recvPacket()
        ehandler.serialSend()
        rate.sleep()