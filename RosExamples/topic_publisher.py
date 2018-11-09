#!/usr/bin/env python

import rospy

# In this case, we’re going to use a 32-bit integer, defined in the ROS
# standard message package, std_msgs.
# For the import to work as expected, we need to import from
# <package name>.msg, since this is where the package definitions are stored
from std_msgs.msg import Int32

# Creates the node which is publishing to the topic
rospy.init_node('topic_publisher')

# This gives the topic a name (counter) and specifies the type of message
# that will be sent over it (Int32) - Intialize the publisher 
pub = rospy.Publisher('counter', Int32)
rate = rospy.Rate(2)
count = 0

while not rospy.is_shutdown():
  pub.publish(count)
  count += 1
  rate.sleep()
