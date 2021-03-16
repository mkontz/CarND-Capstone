import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from math import pi

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yawController = YawController(wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        kp = 0.5
        ki = 0.02
        kd = 0.05
        mn = 0.
        mx = 1.0
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        f = 0.5 # cutoff ferequency (Hz)
        tau = 1. / (2. * pi * f)  # time constant
        ts = 0.02 # sample time 0.02s (50Hz)
        self.vel_filt = LowPassFilter(tau, ts)
        self.last_vel_cmd = 0.

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, linear_vel, angular_vel, dbw_enabled, stop_cmd):

        # rospy.loginfo('Controller: current_vel: %f, linear_vel: %f, angular_vel: %f, dbw_enabled: %u, stop_cmd: %u', current_vel, linear_vel, angular_vel, dbw_enabled, stop_cmd)

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.last_vel_cmd = 0.
            return 0., 0., 0.

        # Time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Velocity
        current_vel = self.vel_filt.filt(current_vel)
        vel_cmd = min(linear_vel, self.last_vel_cmd + sample_time * self.accel_limit)
        self.last_vel_cmd = vel_cmd
        vel_error = vel_cmd - current_vel

        # commands (no brakes)
        steer = self.yawController.get_steering(linear_vel, angular_vel, current_vel)
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.

        if stop_cmd:
            throttle = 0.
            brake = abs(self.decel_limit) * self.vehicle_mass * self.wheel_radius # torque in N*m   
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0.
            brake = 700.
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # torque in N*m

        return throttle, brake, steer
