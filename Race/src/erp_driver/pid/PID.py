import time

class PIDController:
    def __init__(self, kp, ki, kd, initial_value=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.current = initial_value
        self.error_p = 0
        self.error_i = 0
        self.error_d = 0
        self.prev_error = 0
        self.prev_time = time.time()
        self.pid_out = 0

    def compute(self, target):
        # Update current time and calculate time interval
        self.cur_time = time.time()
        self.interval_time = self.cur_time - self.prev_time

        # Calculate proportional error
        self.error_p = target - self.current
        # Update integral error
        self.error_i += self.error_p * self.interval_time
        # Calculate derivative error
        self.error_d = (self.error_p - self.prev_error) / self.interval_time

        # Calculate PID output
        self.pid_out = self.current + self.kp * self.error_p + self.ki * self.error_i + self.kd * self.error_d
        
        # Clamp PID output to bounds
        self.pid_out = max(min(self.pid_out, 2000), -2000)
        #print("Target Steer:", target, "Current:", self.current, "PID_value:", self.pid_out)

        # Update previous error and time
        self.prev_error = self.error_p
        self.prev_time = self.cur_time

        # Return the computed PID output
        return self.pid_out