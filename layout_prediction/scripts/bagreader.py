#! /usr/bin/env python

import rospy
import rosbag

def readbag():
    bag = rosbag.Bag('/home/ism/data/dataset/cv/mitset/2012-04-06-11-15-29.bag')
    for topic, msg, t in bag.read_messages (topics=['/base_odometry/odom']):
        print t
    bag.close()

if __name__ == '__main__':
    try:
        readbag()
    except rospy.ROSInterruptException:
        pass
