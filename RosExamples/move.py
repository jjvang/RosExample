#!/usr/bin/env python
# Line above makes sure your script is excuted as a python script
# Every python controller needs these lines
import rospy
# import rospy is used when writing a ROS node
# The velocity command message
# std_msgs.msg used so we can reuse the string type for publishing
# Twist belongs to the geometry_msgs, we import it from that message 
from geometry_msgs.msg import Twist
if __name__ == '__main__':
    # This section of code defines the talker's interface to the rest of ROS.
    # rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master
    rospy.init_node('move')
    # A publisher for the move data
    # pub = rospy.Publisher("chatter", String, queue_size=10) declares that your node is publishing to the chatter topic using the message type String
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    # Drive forward at a given speed.  The robot points up the x-axis.
    command = Twist()
    # move forward
    command.linear.x = 0.1
    # alone does nothing?
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.1
    command.angular.y = 0.0
    command.angular.z = 0.0
    # Loop at 10Hz, publishing movement commands until we shut down.
    rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(command)
    rate.sleep()
    # This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise)
    # Questions: twist? laserscan? etc.

    # [geometry_msgs/Twist]:
    # geometry_msgs/Vector3 linear
    #   float64 x
    #   float64 y
    #   float64 z
    # geometry_msgs/Vector3 angular
    #   float64 x
    #   float64 y
    #   float64 z
