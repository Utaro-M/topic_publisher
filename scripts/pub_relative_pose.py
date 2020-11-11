#!/usr/bin/env python
# -*- coding: utf-8 -*

import numpy as np
import rospy
from skrobot.coordinates import Coordinates
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from skrobot.coordinates.math import rpy2quaternion

from topic_publisher.cfg import RelativePoseParamsConfig


class RelativePosePublisher():
    def __init__(self):
        self.pub = rospy.Publisher('~output', PoseStamped, queue_size=1)
        self.pose_stamped = PoseStamped()
        self.subscribe()

        self.srv = Server(RelativePoseParamsConfig,
                          self.dynamic_reconfigure_callback)
        self.x = rospy.get_param('~x', 0.)
        self.y = rospy.get_param('~y', 0.)
        self.z = rospy.get_param('~z', 0.)
        self.roll = np.deg2rad(rospy.get_param('~roll', 0.))
        self.pitch = np.deg2rad(rospy.get_param('~pitch', 0.))
        self.yaw = np.deg2rad(rospy.get_param('~yaw', 0.))
        self.q = rpy2quaternion([self.yaw, self.pitch, self.roll])

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input_pose', PoseStamped, self.callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.x = config['x']
        self.y = config['y']
        self.z = config['z']
        self.roll = np.deg2rad(config['roll'])
        self.pitch = np.deg2rad(config['pitch'])
        self.yaw = np.deg2rad(config['yaw'])
        self.q = rpy2quaternion([self.yaw, self.pitch, self.roll])
        return config

    def callback(self, msg):
        source_coords = Coordinates(
            pos=[msg.pose.position.x, msg.pose.position.y,
                 msg.pose.position.z],
            rot=[msg.pose.orientation.w, msg.pose.orientation.x,
                 msg.pose.orientation.y, msg.pose.orientation.z])

        source2dist_coords = Coordinates(
            pos=[self.x, self.y, self.z], rot=self.q)

        dist_coords \
            = source_coords.copy_worldcoords().transform(
                source2dist_coords)

        self.pose_stamped.header = msg.header
        self.pose_stamped.pose.position.x = dist_coords.translation[0]
        self.pose_stamped.pose.position.y = dist_coords.translation[1]
        self.pose_stamped.pose.position.z = dist_coords.translation[2]
        self.pose_stamped.pose.orientation.w = dist_coords.quaternion[0]
        self.pose_stamped.pose.orientation.x = dist_coords.quaternion[1]
        self.pose_stamped.pose.orientation.y = dist_coords.quaternion[2]
        self.pose_stamped.pose.orientation.z = dist_coords.quaternion[3]

        self.pub.publish(self.pose_stamped)


if __name__ == '__main__':
    rospy.init_node('relative_pose_publisher', anonymous=False)
    RelativePosePublisher()
    rospy.spin()
