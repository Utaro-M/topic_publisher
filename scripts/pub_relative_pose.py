#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from skrobot import coordinates
from geometry_msgs.msg import PoseStamped


class RelativePosePublisher():
    def __init__(self):
        self.pub = rospy.Publisher('~output', PoseStamped, queue_size=1)
        self.x = rospy.get_param('~x', 0.)
        self.y = rospy.get_param('~y', 0.)
        self.z = rospy.get_param('~z', 0.)
        self.qx = rospy.get_param('~qx', 0.)
        self.qy = rospy.get_param('~qy', 0.)
        self.qz = rospy.get_param('~qz', 0.)
        self.qw = rospy.get_param('~qw', 1.)
        self.pose_stamped = PoseStamped()
        self.subscribe()
        print('hoge')

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input_pose', PoseStamped, self.callback)

    def callback(self, msg):
        print('called')
        source_coords = coordinates.Coordinates(
            pos=[msg.pose.position.x, msg.pose.position.y,
                 msg.pose.position.z],
            rot=[msg.pose.orientation.w, msg.pose.orientation.x,
                 msg.pose.orientation.y, msg.pose.orientation.z])

        source2dist_coords = coordinates.Coordinates(
            pos=[self.x, self.y, self.z],
            rot=[self.qw, self.qx, self.qy, self.qz])

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
