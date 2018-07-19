#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf

def callback (data):
    br = tf.TransformBroadcaster()
    br.sendTransform (  (data.pose.pose.position.x, data.pose.pose.position.y, 0.0),
                        (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                        data.header.stamp,
                        "odom",
                        "base_link")

if __name__ == '__main__':
    rospy.init_node ("baselink2odom", anonymous=True)
    rospy.Subscriber ("odom", Odometry, callback)
    rospy.spin()
