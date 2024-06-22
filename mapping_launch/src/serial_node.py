#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    # Dummy function to process the cmd_vel message
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

def serial_node():
    rospy.init_node('serial_node', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Serial Node is now subscribed to /cmd_vel")
    rospy.spin()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
