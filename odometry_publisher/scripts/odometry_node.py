#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, TransformStamped
import tf.transformations as tf
import tf2_ros
import math

class OdometryNode:
    def __init__(self):
        rospy.init_node('odom_node')
        self.odom_data_pub = rospy.Publisher('odom_data_euler', Odometry, queue_size=100)
        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=100)
        self.odomNew = Odometry()
        self.odomOld = Odometry()

        # Initial pose
        self.initialX = 0.0
        self.initialY = 0.0
        self.initialTheta = 0.00000000001
        self.PI = 3.141592

        # Robot physical constants
        self.TICKS_PER_REVOLUTION = 10200
        self.WHEEL_RADIUS = 0.224 / 2
        self.WHEEL_BASE = 0.71
        self.TICKS_PER_METER = 14494

        # Distance both wheels have traveled
        self.distanceLeft = 0
        self.distanceRight = 0

        rospy.Subscriber("right_ticks", Int16, self.calc_right)
        rospy.Subscriber("left_ticks", Int16, self.calc_left)

        self.tfb_ = tf2_ros.TransformBroadcaster()

    def calc_left(self, leftCount):
        if leftCount.data != 0 and self.lastCountL != 0:
            leftTicks = leftCount.data - self.lastCountL

            if leftTicks > 10000:
                    leftTicks = 0 - (65535 - leftTicks)
            elif leftTicks < -10000:
                    leftTicks = 65535 - leftTicks

            self.distanceLeft = leftTicks / self.TICKS_PER_METER

        self.lastCountL = leftCount.data

    def calc_right(self, rightCount):
        if rightCount.data != 0 and self.lastCountR != 0:
            rightTicks = rightCount.data - self.lastCountR

            if rightTicks > 10000:
                self.distanceRight = (0 - (65535 - self.distanceRight)) / self.TICKS_PER_METER
            elif rightTicks < -10000:
                rightTicks = 65535 - rightTicks

            self.distanceRight = rightTicks / self.TICKS_PER_METER

        self.lastCountR = rightCount.data

        lastCountR = rightCount.data

    def update_odom(self):
        # Calculate the average distance
        cycleDistance = (self.distanceRight + self.distanceLeft) / 2

        # Calculate the number of radians the robot has turned since the last cycle
        #cycleAngle = math.asin((self.distanceRight - self.distanceLeft) / self.WHEEL_BASE)
        # Using math.atan2() will give you the correct angle regardless of the sign or magnitude of the input
        cycleAngle = math.atan2((self.distanceRight - self.distanceLeft), self.WHEEL_BASE)

        # Average angle during the last cycle
        avgAngle = cycleAngle / 2 + self.odomOld.pose.pose.orientation.z

        if avgAngle > self.PI:
            avgAngle -= 2 * self.PI
        elif avgAngle < -self.PI:
            avgAngle += 2 * self.PI

        # Calculate the new pose (x, y, and theta)
        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        # Prevent lockup from a single bad cycle
        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y) or math.isnan(
                self.odomNew.pose.pose.position.z):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if self.odomNew.pose.pose.orientation.z > self.PI:
            self.odomNew.pose.pose.orientation.z -= 2 * self.PI
        elif self.odomNew.pose.pose.orientation.z < -self.PI:
            self.odomNew.pose.pose.orientation.z += 2 * self.PI

        # Compute the velocity
        current_time = rospy.Time.now()
        delta_time = current_time.to_sec() - self.odomOld.header.stamp.to_sec()
        self.odomNew.header.stamp = current_time
        self.odomNew.twist.twist.linear.x = cycleDistance / delta_time
        self.odomNew.twist.twist.angular.z = cycleAngle / delta_time

        # Save the pose data for the next cycle
        self.odomOld.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.odomOld.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.odomOld.pose.pose.orientation.z = self.odomNew.pose.pose.orientation.z
        self.odomOld.header.stamp = current_time

        # Publish the odometry message
        self.odom_data_pub.publish(self.odomNew)

    def publish_transform(self):
        tfs = TransformStamped()
        tfs.header.stamp = rospy.Time.now()
        tfs.header.frame_id = "odom"
        tfs.child_frame_id = "base_footprint"
        tfs.transform.translation.x = self.odomNew.pose.pose.position.x
        tfs.transform.translation.y = self.odomNew.pose.pose.position.y
        tfs.transform.translation.z = 0.0
        r = tf.quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)
        tfs.transform.rotation.x = r[0]
        tfs.transform.rotation.y = r[1]
        tfs.transform.rotation.z = r[2]
        tfs.transform.rotation.w = r[3]

        tf_msg = TFMessage([tfs])
        self.pub_tf.publish(tf_msg)

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            self.update_odom()
            self.publish_transform()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = OdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
