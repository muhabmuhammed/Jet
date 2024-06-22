#!/usr/bin/env python3

# Author: Automatic Addison
# Date: May 30, 2021
# ROS Version: ROS 1 - Melodic
# Website: https://automaticaddison.com
# This ROS node sends the robot goals to move to a particular location on 
# a map. I have configured this program to the map of my own apartment.
#
# 1 = Bathroom
# 2 = Bedroom
# 3 = Front Door
# 4 = Living Room
# 5 = Home Office
# 6 = Kitchen (Default)

import rospy
import actionlib
from std_msgs.msg import Int16
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main(msg):
    # Initialize the move_base action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Mapping from user choice to goal location
    locations = {
        1: ('Bathroom', (10.0, 3.7, 0.0)),
        2: ('Bedroom', (8.1, 4.3, 0.0)),
        3: ('Front Door', (10.5, 2.0, 0.0)),
        4: ('Living Room', (5.3, 2.7, 0.0)),
        5: ('Home Office', (2.5, 2.0, 0.0)),
        6: ('Kitchen', (3.0, 6.0, 0.0))
    }

    run = True
    while run:
        
        # Set up the goal location
        if msg.data in locations:
            location_name, (x, y, w) = locations[msg.data]
            print("\nGoal Location: {}\n".format(location_name))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = w
            
            # Send the goal
            rospy.loginfo("Sending goal")
            client.send_goal(goal)
            client.wait_for_result()
            
            # Check if the robot reached the goal
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("The robot has arrived at the goal location")
            else:
                rospy.loginfo("The robot failed to reach the goal location for some reason")
        else:
            print("\nInvalid selection. Please try again.\n")
            continue

if __name__ == '__main__':
    rospy.init_node('simple_navigation_goals')
    rospy.Subscriber("sub", Int16, main )
    rospy.spin()

    
