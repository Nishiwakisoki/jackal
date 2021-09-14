#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


def imageCb(msg):
 err = int(msg.data) - 320
 twist.linear.x = 0.1
 twist.angular.z = -float(err) / 500
 pub.publish(twist)


if __name__=='__main__':
 rospy.init_node('yello_follow')
 twist = Twist()
 pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
 sub = rospy.Subscriber('/center_line', Int32, imageCb)
 rospy.spin()
