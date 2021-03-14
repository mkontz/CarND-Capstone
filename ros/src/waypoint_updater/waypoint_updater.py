#!/usr/bin/env python

import rospy

# messages
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

# Other
import math
import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', ??, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.target_decel = rospy.get_param('~target_decel')
        self.max_decel = rospy.get_param('~max_decel')

        # Member variables
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.i_stop = -1

        rospy.loginfo('WaypointUpdater: done initializing')

        self.loop()

    def loop(self):
        rate = rospy.Rate(1)
        rospy.loginfo('WaypointUpdater: starting loop')
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

    def get_next_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        next_idx = self.waypoint_tree.query([x,y], 1)[1]

        pos = np.array([x, y])
        closest_pos = np.array(self.waypoints_2d[next_idx])
        prev_pos = np.array(self.waypoints_2d[next_idx-1])

        prod = np.dot(closest_pos-prev_pos, pos-closest_pos)

        if (0 < prod):
            next_idx = (next_idx + 1) % len(self.waypoints_2d)

        return next_idx

    def publish_waypoints(self):
        if (self.pose is not None and self.base_waypoints is not None and
            self.waypoints_2d is not None and self.waypoint_tree is not None):

            next_idx = self.get_next_waypoint_idx()

            lane = Lane()
            lane.header = self.base_waypoints.header

            # if (self.i_stop < 0)

            wrap_around = LOOKAHEAD_WPS - (len(self.base_waypoints.waypoints) - next_idx)
            if 0 < wrap_around:
                lane.waypoints = self.base_waypoints.waypoints[next_idx : ] + self.base_waypoints.waypoints[ : wrap_around]
            else:
                lane.waypoints =self.base_waypoints.waypoints[next_idx : next_idx + LOOKAHEAD_WPS]

            # # decel at end of points incase communication is lost.
            # lane.waypoints = self.decelerate(lane.waypoints, self.target_decel)

            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, pose_msg):
        self.pose = pose_msg

    def waypoints_cb(self, waypoints):
        if self.waypoints_2d is None:
            self.base_waypoints = waypoints
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

            # remove decel ramp at end of waypoints
            target_spd = self.base_waypoints.waypoints[0].twist.twist.linear.x
            i = len(self.base_waypoints.waypoints) - 1
            while (0 < i) and (self.base_waypoints.waypoints[i].twist.twist.linear.x < target_spd):
                rospy.loginfo('i: %u, spd: %f, target: %f', i, self.base_waypoints.waypoints[i].twist.twist.linear.x, target_spd)
                self.base_waypoints.waypoints[i].twist.twist.linear.x = target_spd
                i -= 1

    def traffic_cb(self, msg):
        if self.i_stop != msg.data:
            self.i_stop = msg.data

            # publish new waypoints
            # self.publish_waypoints()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2):
            dist += dl(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist

    def distanceL2(self, waypoints, wp1, wp2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(waypoints[wp1].pose.pose.position, waypoints[wp2].pose.pose.position)

    def decelerate(self, waypoints, decel):
        waypoints[-1].twist.twist.linear.x = 0.
        i_last = len(waypoints) - 1
        i = i_last - 1
        while (0 <= i):
            vel = math.sqrt(2 * decel * self.distance(waypoints, i, i_last))
            if (vel < waypoints[i].twist.twist.linear.x):
                rospy.loginfo("Setting vel %u = %f", i, vel)
                waypoints[i].twist.twist.linear.x = vel
                i -= 1
            else:
                break

        return waypoints


if __name__ == '__main__':
    try:
        waypoint_updater = WaypointUpdater()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
