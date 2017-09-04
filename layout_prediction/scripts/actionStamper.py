#! /usr/bin/env python

import rospy
from pr2_mechanism_controllers.msg import BaseOdometryState
from nav_msgs.msg import Odometry

pub = rospy.Publisher ('/myslam/action', Odometry, queue_size = 10)

def callback (data):
    odometry = Odometry ()
    odometry.header.stamp = rospy.Time.now () 
    odometry.pose.pose.position.x = data.velocity.linear.x
    odometry.pose.pose.position.y = data.velocity.linear.y
    odometry.pose.pose.orientation.z = data.velocity.angular.z
    pub.publish (odometry)

def actionStamper ():
    rospy.init_node ("actionStamper", anonymous = True)
    rospy.Subscriber ("/base_odometry/state", BaseOdometryState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        actionStamper ()
    except rospy.ROSInterruptException:
        pass
