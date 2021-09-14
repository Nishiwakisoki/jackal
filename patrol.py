#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class patrol:
 def __init__(self):
  self.bridge = cv_bridge.CvBridge()
  cv2.namedWindow("window", 1)
  self.twist = Twist()
  self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
  self.image_pub = rospy.Publisher('/findline_img', Image, queue_size=1)
  self.cent_pub = rospy.Publisher('/center_line',Int32, queue_size=1)
  self.red_pub = rospy.Publisher('/red_detect',Int32, queue_size=1)
  self.green_pub = rospy.Publisher('/green_detect',Int32, queue_size=1)
  self.blue_pub = rospy.Publisher('/blue_detect',Int32, queue_size=1)
  
 def image_callback(self, msg):
  #while not rospy.is_shutdown():
  #rate = rospy.Rate(2)
  kernel = numpy.ones((3,3),numpy.uint8)
  image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  lower_yellow = numpy.array([20, 10, 100])
  upper_yellow = numpy.array([40, 255, 255])
  mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
  # BEGIN FILTER
  lower_red = numpy.array([0, 180, 100])
  upper_red = numpy.array([10, 255, 255])
  mask2 = cv2.inRange(hsv, lower_red, upper_red)

  lower_green = numpy.array([37, 100, 100])
  upper_green = numpy.array([80, 255, 255])
  mask3 = cv2.inRange(hsv, lower_green, upper_green)

  lower_blue = numpy.array([85, 100, 60])
  upper_blue = numpy.array([120, 255, 255])
  mask4 = cv2.inRange(hsv, lower_blue, upper_blue)

  #lower_blue = numpy.array([0, 100, 100])
  #upper_blue = numpy.array([10, 255, 255])
  #mask4 = cv2.inRange(hsv, lower_blue, upper_blue)
  # BEGIN CROP
  h, w, d = image.shape
  search_top = 3*h/4
  search_bot = 3*h/4 + 100
  serch_left = 1*w/5
  serch_right = 4*w/5
  
  serch_red = 1*w/2
	
  mask1[0:search_top, 0:w] = 0
  mask1[search_bot:h, 0:w] = 0
  mask1[search_top:search_bot, 0:serch_left] = 0
  mask1[search_top:search_bot, serch_right:w] = 0
  mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)
  # END CROP
  mask2[0:search_top, 0:w] = 0
  mask2[search_bot:h, 0:w] = 0
  mask2[search_top:search_bot, 0:serch_red] = 0
  mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)

  mask3[0:search_top, 0:w] = 0
  mask3[search_bot:h, 0:w] = 0
  mask3[search_top:search_bot, 0:serch_red] = 0
  mask3 = cv2.morphologyEx(mask3, cv2.MORPH_OPEN, kernel)

  mask4[0:search_top, 0:w] = 0
  mask4[search_bot:h, 0:w] = 0
  mask4[search_top:search_bot, 0:serch_red] = 0
  mask4 = cv2.morphologyEx(mask4, cv2.MORPH_OPEN, kernel)
   
  M1 = cv2.moments(mask1)
  M2 = cv2.moments(mask2)
  M3 = cv2.moments(mask3)
  M4 = cv2.moments(mask4)

  if (M1['m00'] > 200000 and M2['m00'] < 200000):
   cx = int(M1['m10']/M1['m00'])
   cy = int(M1['m01']/M1['m00'])
   cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
   self.cent_pub.publish(cx)

  if (M2['m00'] >= 200000):
   red = 1
   self.red_pub.publish(red)

  if (M3['m00'] >= 200000):
   green = 1
   self.green_pub.publish(green)

  if (M4['m00'] >= 200000):
   blue = 1
   self.blue_pub.publish(blue)

  findline_img = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
  self.image_pub.publish(findline_img)

  # END CONTROL
  cv2.imshow("window", image)
  cv2.imshow("window2", mask1)
  cv2.imshow("window3", mask2)
  cv2.imshow("window2", mask3)
  cv2.imshow("window3", mask4)
  cv2.waitKey(3)
  #rate.sleep()

if __name__ == '__main__':
 rospy.init_node('patrol')
 patrol = patrol()
 rospy.spin()
 # END ALL
