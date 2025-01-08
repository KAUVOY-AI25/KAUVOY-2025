import rospy
from tkinter import *
from erp_driver.msg import erpCmdMsg

class ERPControlUI:
    def __init__(self, master):
        self.master = master
        master.title("ERP Control")

        self.gear_var = IntVar()
        self.e_stop_var = BooleanVar()
        self.speed_var = IntVar()
        self.steer_var = IntVar()
        self.brake_var = IntVar()

        self.create_widgets()

        # Initialize ROS publisher without initializing a node
        self.pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=10)

    def create_widgets(self):
        Label(self.master, text="Gear (0 or 1):").grid(row=0, column=0, sticky=W)
        Entry(self.master, textvariable=self.gear_var).grid(row=0, column=1)

        Label(self.master, text="Emergency Stop (True or False):").grid(row=1, column=0, sticky=W)
        Checkbutton(self.master, variable=self.e_stop_var).grid(row=1, column=1)

        Label(self.master, text="Speed (0-200):").grid(row=2, column=0, sticky=W)
        Entry(self.master, textvariable=self.speed_var).grid(row=2, column=1)

        Label(self.master, text="Steer (-2000 to 2000):").grid(row=3, column=0, sticky=W)
        Entry(self.master, textvariable=self.steer_var).grid(row=3, column=1)

        Label(self.master, text="Brake (1-33):").grid(row=4, column=0, sticky=W)
        Entry(self.master, textvariable=self.brake_var).grid(row=4, column=1)

        Button(self.master, text="Send Command", command=self.send_command).grid(row=5, columnspan=2)

    def send_command(self):
        gear = self.gear_var.get()
        e_stop = self.e_stop_var.get()
        speed = self.speed_var.get()
        steer = self.steer_var.get()
        brake = self.brake_var.get()

        cmd_msg = erpCmdMsg()
        cmd_msg.gear = gear
        cmd_msg.e_stop = e_stop
        cmd_msg.speed = speed
        cmd_msg.steer = steer
        cmd_msg.brake = brake

        self.pub.publish(cmd_msg)
        rospy.loginfo("Command sent: %s", cmd_msg)

if __name__ == "__main__":
    rospy.init_node("erp_control_ui_node", anonymous=True)
    root = Tk()
    app = ERPControlUI(root)
    rospy.loginfo("ERP Control UI started.")
    root.mainloop()
    rospy.spin()
