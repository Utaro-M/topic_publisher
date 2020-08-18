#!/usr/bin/env python
# -*- coding: utf-8 -*

from dynamic_reconfigure.server import Server
from box_publisher.cfg import DynamicParamsConfig
import rospy
from jsk_recognition_msgs.msg import BoundingBox
import sys


class Boxpublisher():
    def __init__(self):
        self.srv = Server(DynamicParamsConfig,
                          self.dynamic_reconfigure_callback)
        self.pub = rospy.Publisher("~output", BoundingBox, queue_size=1)
        self.box = BoundingBox()
        self.frame = rospy.get_param("~frame", "/base_link")
        self.box.header.frame_id = self.frame
        self.x = rospy.get_param("~x", 0.6)
        self.y = rospy.get_param("~y", 0.0)
        self.z = rospy.get_param("~z", 0.2)
        self.w = rospy.get_param("~w", 0.6)
        self.d = rospy.get_param("~d", 0.6)
        self.h = rospy.get_param("~h", 0.2)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.x = config["x"]
        self.y = config["y"]
        self.z = config["z"]
        self.w = config["w"]
        self.d = config["d"]
        self.h = config["h"]
        return config

    def timer_callback(self, timer):
        rospy.loginfo("pub box")
        self.box.header.stamp = rospy.Time.now()
        self.box.pose.position.x = self.x
        self.box.pose.position.y = self.y
        self.box.pose.position.z = self.z
        self.box.dimensions.x = self.w
        self.box.dimensions.y = self.d
        self.box.dimensions.z = self.h
        self.box.pose.orientation.x = 0.
        self.box.pose.orientation.y = 0.
        self.box.pose.orientation.z = 0.
        self.box.pose.orientation.w = 1.
        self.pub.publish(self.box)


def main(args):
    rospy.init_node("box_publisher", anonymous=False)
    bp = Boxpublisher()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

