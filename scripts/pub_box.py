#!/usr/bin/env python
# -*- coding: utf-8 -*

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import BoundingBox
from skrobot.coordinates.math import rpy2quaternion

from topic_publisher.cfg import BoxParamsConfig


class BoxPublisher():
    def __init__(self):
        self.pub = rospy.Publisher("~output", BoundingBox, queue_size=1)
        self.box = BoundingBox()
        self.frame = rospy.get_param("~frame", "/base_link")
        self.box.header.frame_id = self.frame
        self.box.header.seq = 0

        self.srv = Server(BoxParamsConfig,
                          self.dynamic_reconfigure_callback)
        self.x = rospy.get_param("~x", 0.)
        self.y = rospy.get_param("~y", 0.)
        self.z = rospy.get_param("~z", 0.)
        self.w = rospy.get_param("~w", 0.2)
        self.d = rospy.get_param("~d", 0.2)
        self.h = rospy.get_param("~h", 0.2)
        self.roll = np.deg2rad(rospy.get_param('~roll', 0.))
        self.pitch = np.deg2rad(rospy.get_param('~pitch', 0.))
        self.yaw = np.deg2rad(rospy.get_param('~yaw', 0.))
        self.q = rpy2quaternion([self.yaw, self.pitch, self.roll])

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.x = config["x"]
        self.y = config["y"]
        self.z = config["z"]
        self.w = config["w"]
        self.d = config["d"]
        self.h = config["h"]
        self.roll = np.deg2rad(config['roll'])
        self.pitch = np.deg2rad(config['pitch'])
        self.yaw = np.deg2rad(config['yaw'])
        self.q = rpy2quaternion([self.yaw, self.pitch, self.roll])

        return config

    def timer_callback(self, event):
        rospy.loginfo("pub box")
        self.box.header.stamp = event.current_real
        self.box.pose.position.x = self.x
        self.box.pose.position.y = self.y
        self.box.pose.position.z = self.z
        self.box.dimensions.x = self.w
        self.box.dimensions.y = self.d
        self.box.dimensions.z = self.h
        self.box.pose.orientation.w = self.q[0]
        self.box.pose.orientation.x = self.q[1]
        self.box.pose.orientation.y = self.q[2]
        self.box.pose.orientation.z = self.q[3]

        self.pub.publish(self.box)


if __name__ == '__main__':
    rospy.init_node("box_publisher", anonymous=False)
    BoxPublisher()
    rospy.spin()
