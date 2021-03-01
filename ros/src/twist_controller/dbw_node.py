#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Subscribe to topics
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        # Controller
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        # Input variables
        self.dbw_enabled = None
        self.current_velocity = None
        self.current_ang_vel = None
        self.current_velocity_time = None
        self.linear_vel = None
        self.angular_vel = None

        # Output variables
        self.throttle = self.steer = self.brake = 0.

        rospy.loginfo('init complete')

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        self.throttle = 0.
        self.steer = 0.
        self.brake = 700.0 # enough to prevent car from moving

        rospy.loginfo('starting to loop')


        while not rospy.is_shutdown():
            self.throttle = 0.
            self.brake = 700.0 # enough to prevent car from moving

            if (self.dbw_enabled is not None and
                self.current_velocity_time is not None and
                self.current_velocity is not None and
                self.linear_vel is not None and
                self.angular_vel is not None):

                stop_cmd = (0.2 < (rospy.get_time() - self.current_velocity_time))

                # rospy.loginfo('loop - controller input: dbw_enabled: %u, stop_cmd: %u', self.dbw_enabled, stop_cmd)
                # rospy.loginfo('loop - controller: current_velocity: %f, linear_vel: %f, angular_vel: %f', self.current_velocity, self.linear_vel, self.angular_vel)

                # You should only publish the control commands if dbw is enabled
                self.throttle, self.brake, self.steer = self.controller.control(self.current_velocity,
                                                                                self.linear_vel,
                                                                                self.angular_vel,
                                                                                self.dbw_enabled,
                                                                                stop_cmd)

                # rospy.loginfo('loop - controller: throttle: %f, brake: %f, steer: %f', self.throttle, self.brake, self.steer)

            if (self.dbw_enabled is not None and self.dbw_enabled):
                # rospy.loginfo('loop - publish: throttle: %f, brake: %f, steer: %f', self.throttle, self.brake, self.steer)
                self.publish(self.throttle, self.brake, self.steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

        # rospy.loginfo('dbw_enabled_cb: dbw_enabled: %u', self.dbw_enabled)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        self.current_velocity_time = rospy.get_time()

        # rospy.loginfo('current_velocity_cb: current_velocity: %f', self.current_velocity)

    def twist_cmd_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

        # rospy.loginfo('twist_cmd_cb: linear_vel: %f, angular_vel: %f', self.linear_vel, self.angular_vel)


if __name__ == '__main__':
    DBWNode()
