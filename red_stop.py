#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

count = 0
count_t = 0

def imageCb(msg):
 global count
 global count_t
 if (count == 0 ):
  count = 1
  twist.linear.x = 0.0
  twist.angular.z = 0.0
  pub.publish(twist)
  time.sleep(2)
  while (count_t < 30):
   twist.linear.x = 0.1
   twist.angular.z = 0.0
   pub.publish(twist)
   time.sleep(0.1)
   count_t += 1
   rospy.loginfo("count:%d",count)
   rospy.loginfo("count_t:%d",count_t)



if __name__=='__main__':
 rospy.init_node('red_stop')
 twist = Twist()
 pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
 sub = rospy.Subscriber('/red_detect', Int32, imageCb)
 rospy.spin()
