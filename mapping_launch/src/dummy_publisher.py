#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def dummy_publisher():
    rospy.init_node('rplidarNode', anonymous=True)
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.loginfo("Dummy rplidarNode started")
    rospy.spin()  # Keep the node running without publishing anything

if __name__ == '__main__':
    try:
        dummy_publisher()
    except rospy.ROSInterruptException:
        pass
