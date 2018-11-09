#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) #[1]
rospy.init_node('red_light_green_light')
red_light_twist = Twist() #[2]
green_light_twist = Twist()
green_light_twist.linear.x = 0.5 #[3]
driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist) #[4]
    else:
        cmd_vel_pub.publish(red_light_twist)
    if light_change_time > rospy.Time.now(): #[5]
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)

    rate.sleep() #[6]

# [1]The queue_size=1 argument tells rospy to only buffer a single outbound message.
# In case the node sending the messages is transmitting at a higher rate than
# the receiving node(s) can receive them, rospy will simply drop any messages
# beyond the queue_size.
#
# [2]The message constructors set all fields to zero. Therefore, the red_light_twist
# message tells a robot to stop, since all of its velocity subcomponents are zero.
#
# [3]The x component of the linear velocity in a Twist message is, by convention,
# aligned in the direction the robot is facing, so this line means “drive straight
# ahead at 0.5 meters per second.”
#
# [4]We need to continually publish a stream of velocity command messages, since
# most mobile base drivers will time out and stop the robot if they don’t receive at
# least several messages per second.
#
# [5]This branch checks the system time and toggles the red/green light periodically.
#
# [6]Without this call to rospy.sleep() the code would still run, but it would send far
# too many messages, and take up an entire CPU core!
