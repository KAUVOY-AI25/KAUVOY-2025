#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg, erpCmdMsg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv

class ERPStatusPlotter:
    def __init__(self):
        rospy.init_node("erp_status_plotter")

        # Status data
        self.gear_data = []
        self.speed_data = []
        self.steer_data = []
        self.brake_data = []
        self.enc_data = []
        self.time_data = []

        # Command data
        self.cmd_gear_data = []
        self.cmd_speed_data = []
        self.cmd_steer_data = []
        self.cmd_brake_data = []
        self.cmd_time_data = []

        self.start_time = time.time()

        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/erp42_ctrl_cmd", erpCmdMsg, self.cmd_callback)

        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4, self.ax5) = plt.subplots(5, 1, figsize=(10, 8))
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000)

    def status_callback(self, msg):
        current_time = time.time() - self.start_time

        self.gear_data.append((current_time, msg.gear))
        self.speed_data.append((current_time, msg.speed))
        self.steer_data.append((current_time, msg.steer))
        self.brake_data.append((current_time, msg.brake))
        self.enc_data.append((current_time, msg.encoder))
        self.time_data.append(current_time)

    def cmd_callback(self, msg):
        current_time = time.time() - self.start_time

        self.cmd_gear_data.append((current_time, msg.gear))
        self.cmd_speed_data.append((current_time, msg.speed))
        self.cmd_steer_data.append((current_time, msg.steer))
        self.cmd_brake_data.append((current_time, msg.brake))
        self.cmd_time_data.append(current_time)

    def update_plot(self, frame):
        if not self.time_data:
            return

        times = self.time_data
        cmd_times = self.cmd_time_data

        # Plot gear data
        self.ax1.clear()
        if len(times) == len(self.gear_data):
            self.ax1.plot(times, [data[1] for data in self.gear_data], label="Status Gear")
        if len(cmd_times) == len(self.cmd_gear_data):
            self.ax1.plot(cmd_times, [data[1] for data in self.cmd_gear_data], label="Cmd Gear", linestyle='dashed')
        self.ax1.set_ylabel("Gear")
        self.ax1.set_ylim(0, 2)
        self.ax1.set_yticks([0, 1, 2])
        self.ax1.legend()

        # Plot speed data
        self.ax2.clear()
        if len(times) == len(self.speed_data):
            self.ax2.plot(times, [data[1] for data in self.speed_data], label="Status Speed")
        if len(cmd_times) == len(self.cmd_speed_data):
            self.ax2.plot(cmd_times, [data[1] for data in self.cmd_speed_data], label="Cmd Speed", linestyle='dashed')
        self.ax2.set_ylabel("Speed")
        self.ax2.set_ylim(0, 200)
        self.ax2.legend()

        # Plot steer data
        self.ax3.clear()
        if len(times) == len(self.steer_data):
            self.ax3.plot(times, [data[1] for data in self.steer_data], label="Status Steer")
        if len(cmd_times) == len(self.cmd_steer_data):
            self.ax3.plot(cmd_times, [data[1] for data in self.cmd_steer_data], label="Cmd Steer", linestyle='dashed')
        self.ax3.set_ylabel("Steer")
        self.ax3.set_ylim(-2000, 2000)
        self.ax3.legend()

        # Plot brake data
        self.ax4.clear()
        if len(times) == len(self.brake_data):
            self.ax4.plot(times, [data[1] for data in self.brake_data], label="Status Brake")
        if len(cmd_times) == len(self.cmd_brake_data):
            self.ax4.plot(cmd_times, [data[1] for data in self.cmd_brake_data], label="Cmd Brake", linestyle='dashed')
        self.ax4.set_ylabel("Brake")
        self.ax4.set_ylim(1, 33)
        self.ax4.legend()

        # Plot encoder data
        self.ax5.clear()
        if len(times) == len(self.enc_data):
            self.ax5.plot(times, [data[1] for data in self.enc_data], label="Encoder")
        self.ax5.set_ylabel("Encoder")
        self.ax5.set_xlabel("Time")
        self.ax5.legend()

    def save_data_to_csv(self):
            with open('erp_data.csv', 'w', newline='') as csvfile:
                fieldnames = ['Time', 'status_Gear', 'status_Speed', 'status_Steer', 'status_Brake']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                for i in range(len(self.time_data)):
                    writer.writerow({
                        'Time': self.time_data[i],
                        'Cmd_Gear': self.gear_data[i][1],
                        'Cmd_Speed': self.speed_data[i][1],
                        'Cmd_Steer': self.steer_data[i][1],
                        'Cmd_Brake': self.brake_data[i][1]
                    })

    def run(self):
        plt.show()

if __name__ == "__main__":
    plotter = ERPStatusPlotter()
    plotter.run()
