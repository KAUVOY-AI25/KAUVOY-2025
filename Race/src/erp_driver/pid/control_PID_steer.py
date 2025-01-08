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


# Fixed variable values
SPEED = 0
STEER = 0
BRAKE = 1
ESTOP = False
GEAR = 2

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

        # Initialize PID controller for steering
        self.steer_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, initial_value=STEER)
        #self.steer_pid = PIDController(kp=0, ki=0, kd=0, initial_value=STEER)
        self.target_steer = 0

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
        # Update the current steering value in the PID controller
        self.steer_pid.current = msg.steer
        # Calculate the new steering value using PID
        new_steer_value = self.steer_pid.compute(self.target_steer)
        # Update the command with the new steering value
        self.update_cmd('steer', new_steer_value)

    def update_steer(self):
        new_steer_value = self.steer_pid.compute(self.target_steer)
        self.update_cmd('steer', new_steer_value)

if __name__ == "__main__":
    rospy.init_node("erp_cmd_sender")
    port = rospy.get_param("/erp_base/port")
    baudrate = rospy.get_param("/erp_base/baudrate")
    
    erp_handler = ERPHandler(port, baudrate)
    
    while not rospy.is_shutdown():
        erp_handler.update_steer()  # Update the steering value using PID
        erp_handler.publish_cmd()  # Publish the fixed command
        erp_handler.send_packet()  # Send the packet via serial

        # Check if there's input from the terminal
        if select.select([sys.stdin], [], [], 0)[0]:
            try:
                new_target_steer = float(input("Enter new target steer value: "))
                erp_handler.target_steer = new_target_steer
                print(f"Updated target steer to: {new_target_steer}")
            except ValueError:
                print("Invalid input. Please enter a numerical value.")

        rospy.sleep(0.1)

# import sys
# import rospy
# from erp_driver.msg import erpCmdMsg
# import serial
# from ByteHandler import ErpMsg2Packet
# import tkinter as tk
# from tkinter import ttk
# from PID import PIDController

# class ERPHandler:
#     def __init__(self, port, baudrate) -> None:
#         self.serial = serial.Serial(port=port, baudrate=baudrate)
#         self.alive = 0
#         self.cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)
#         self.packet = erpCmdMsg()
#         self.packet.e_stop = False
#         self.packet.gear = 0
#         self.packet.speed = 0
#         self.packet.steer = 0
#         self.packet.brake = 1

#         # Initialize PID controller for steering
#         self.steer_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, initial_value=0)
#         self.target_steer = 0

#     def send_packet(self) -> None:
#         packet = ErpMsg2Packet(self.packet, self.alive)
#         self.serial.write(packet)
#         self.alive = (self.alive + 1) % 256

#     def publish_cmd(self) -> None:
#         self.cmd_pub.publish(self.packet)

#     def update_cmd(self, field, value):
#         if field == 'e_stop':
#             self.packet.e_stop = bool(int(value))
#         elif field == 'gear':
#             self.packet.gear = int(value)
#         elif field == 'speed':
#             self.packet.speed = int(value)
#         elif field == 'steer':
#             self.packet.steer = int(value)
#         elif field == 'brake':
#             self.packet.brake = int(value)

#     def update_steer(self):
#         new_steer_value = self.steer_pid.compute(self.target_steer)
#         self.update_cmd('steer', new_steer_value)

#     def set_quit_values(self):
#         """Sets the values to be used when quitting."""
#         self.update_cmd('speed', '0')
#         self.update_cmd('steer', '0')
#         self.update_cmd('brake', '33')
#         self.publish_cmd()

# class ERPControlApp:
#     def __init__(self, master, erp_handler):
#         self.master = master
#         self.erp_handler = erp_handler
#         self.master.title("ERP42 Control Panel")
#         self.master.geometry("1200x800")  # Set the size of the window to 1200x800

#         self.style = ttk.Style()
#         self.style.configure('TLabel', font=('Arial', 18, 'bold'))
#         self.style.configure('TEntry', font=('Arial', 18, 'bold'), padding=10)
#         self.style.configure('TButton', font=('Arial', 18, 'bold'), padding=20)

#         self.fields = ['e_stop', 'gear', 'speed', 'steer', 'brake']
#         self.entries = {}

#         for idx, field in enumerate(self.fields):
#             ttk.Label(master, text=field).grid(row=idx, column=0, padx=20, pady=20, sticky='E')
#             self.entries[field] = ttk.Entry(master)
#             self.entries[field].grid(row=idx, column=1, padx=20, pady=20, sticky='W')
#             self.entries[field].insert(0, '0')  # Set default value to 0

#         # Add target steer input
#         ttk.Label(master, text='target_steer').grid(row=len(self.fields), column=0, padx=20, pady=20, sticky='E')
#         self.target_steer_entry = ttk.Entry(master)
#         self.target_steer_entry.grid(row=len(self.fields), column=1, padx=20, pady=20, sticky='W')
#         self.target_steer_entry.insert(0, '0')  # Set default value to 0

#         button_frame = tk.Frame(master)
#         button_frame.grid(row=len(self.fields)+1, columnspan=2, pady=20)

#         self.update_button = ttk.Button(button_frame, text="Update Command", command=self.update_command)
#         self.update_button.grid(row=0, column=0, padx=10, ipadx=40, ipady=20)

#         self.quit_button = ttk.Button(button_frame, text="Quit", command=self.quit_app)
#         self.quit_button.grid(row=0, column=1, padx=10, ipadx=40, ipady=20)

#         # Configure button colors and sizes
#         self.update_button.config(style='TButton')
#         self.quit_button.config(style='TButton')

#         self.style.configure('TButton', foreground='white')
#         self.style.map('TButton', background=[('!active', 'blue'), ('pressed', 'blue'), ('active', 'blue')], foreground=[('!active', 'white'), ('pressed', 'white'), ('active', 'white')])

#         self.quit_button.config(style='Red.TButton')
#         self.style.configure('Red.TButton', foreground='white')
#         self.style.map('Red.TButton', background=[('!active', 'red'), ('pressed', 'red'), ('active', 'red')], foreground=[('!active', 'white'), ('pressed', 'white'), ('active', 'white')])

#         self.master.bind('<Return>', lambda event: self.update_command())

#         self.update_serial()

#     def update_command(self):
#         for field in self.fields:
#             value = self.entries[field].get()
#             self.erp_handler.update_cmd(field, value)
#         self.erp_handler.publish_cmd()

#         # Update target steer value
#         target_steer_value = self.target_steer_entry.get()
#         try:
#             self.erp_handler.target_steer = float(target_steer_value)
#             print(f"Updated target steer to: {target_steer_value}")
#         except ValueError:
#             print("Invalid input for target steer. Please enter a numerical value.")
#         self.erp_handler.update_steer()

#     def quit_app(self):
#         """Sets the values to be used when quitting and then quits the application."""
#         self.erp_handler.set_quit_values()
#         self.master.quit()

#     def update_serial(self):
#         self.erp_handler.send_packet()
#         self.master.after(100, self.update_serial)

# if __name__ == "__main__":
#     rospy.init_node("erp_cmd_sender")
#     port = rospy.get_param("/erp_base/port")
#     baudrate = rospy.get_param("/erp_base/baudrate")
    
#     erp_handler = ERPHandler(port, baudrate)
    
#     root = tk.Tk()
#     app = ERPControlApp(root, erp_handler)

#     try:
#         root.mainloop()
#     except KeyboardInterrupt:
#         # Handle KeyboardInterrupt
#         app.quit_app()
#         sys.exit()
