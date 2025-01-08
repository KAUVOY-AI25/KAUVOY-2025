import sys
import select
import time
sys.path.append(r'/home/kauvoy/anaconda3/lib/python3.8/site-packages')

import rospy
from erp_driver.msg import erpCmdMsg, erpStatusMsg  # Import the erpStatusMsg
import serial
import struct
import numpy as np
from ByteHandler import ErpMsg2Packet
from PID import PIDController
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Fixed variable values
SPEED = 0
STEER = 0
BRAKE = 1
ESTOP = False
GEAR = 0

class ERPHandler:
    def __init__(self, port, baudrate) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.alive = 0
        self.cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)
        self.status_sub = rospy.Subscriber('/erp42_status', erpStatusMsg, self.status_callback)  # Subscribe to the ERP status message
        self.packet = erpCmdMsg()
        self.update_cmd('e_stop', ESTOP)
        self.update_cmd('gear', GEAR)
        self.update_cmd('speed', SPEED)
        self.update_cmd('steer', STEER)
        self.update_cmd('brake', BRAKE)

        # Initialize PID controller for speed
        self.speed_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, initial_value=SPEED)
        self.target_speed = 0

        # Initialize plotting
        self.speed_data = []
        self.target_speed_data = []
        self.time_data = []
        self.start_time = time.time()

        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000)
        plt.ion()

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
        current_time = time.time() - self.start_time

        # Update the current speed in the PID controller
        self.speed_pid.current = msg.speed
        # Calculate the new speed value using PID
        new_speed_value = self.speed_pid.compute(self.target_speed)
        # Update the command with the new speed value
        self.update_cmd('speed', new_speed_value)

        # Log data for plotting
        self.speed_data.append((current_time, msg.speed))
        self.target_speed_data.append((current_time, self.target_speed))
        self.time_data.append(current_time)

    def update_speed(self):
        new_speed_value = self.speed_pid.compute(self.target_speed)
        self.update_cmd('speed', new_speed_value)

    def update_plot(self, frame):
        if not self.time_data:
            return

        times = self.time_data

        self.ax.clear()
        if len(times) == len(self.speed_data):
            self.ax.plot(times, [data[1] for data in self.speed_data], label="Current Speed")
        if len(times) == len(self.target_speed_data):
            self.ax.plot(times, [data[1] for data in self.target_speed_data], label="Target Speed", linestyle='dashed')
        self.ax.set_ylabel("Speed")
        self.ax.set_xlabel("Time")
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)

    def run(self):
        plt.show(block=False)

if __name__ == "__main__":
    rospy.init_node("erp_cmd_sender")
    port = rospy.get_param("/erp_base/port")
    baudrate = rospy.get_param("/erp_base/baudrate")
    
    erp_handler = ERPHandler(port, baudrate)
    
    erp_handler.run()
    while not rospy.is_shutdown():
        erp_handler.update_speed()  # Update the speed value using PID
        erp_handler.publish_cmd()  # Publish the fixed command
        erp_handler.send_packet()  # Send the packet via serial

        # Check if there's input from the terminal
        if select.select([sys.stdin], [], [], 0)[0]:
            try:
                new_target_speed = float(input("Enter new target speed value: "))
                erp_handler.target_speed = new_target_speed
                print(f"Updated target speed to: {new_target_speed}")
            except ValueError:
                print("Invalid input. Please enter a numerical value.")

        rospy.sleep(0.1)
