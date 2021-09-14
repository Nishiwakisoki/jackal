#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

count=0
count_go=0

def imageCb(msg):
 count += 1
 if (count == 1 ):
  for count_go in range(30):
   twist.linear.x = 0.1
   twist.angular.z = 0.0
   pub.publish(twist)
   count_go +=1
  
  twist.linear.x = 0.0
  twist.angular.z = 0.0
  pub.publish(twist)


if __name__=='__main__':
 rospy.init_node('red_stop')
 twist = Twist()
 pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
 sub = rospy.Subscriber('/red_detect', Int32, imageCb)
 rospy.spin()
