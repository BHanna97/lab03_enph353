#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
from geometry_msgs.msg import Twist
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class line_follower:

  def __init__(self):

#intialize movement of the robot
    self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
   
    
#initialize bridge and subscribe to the image stream
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    
#convert to cv image.
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      print(e)
   
    
    cv2.waitKey(3)
    threshold = 105
#convert frames to grayscale
    grey_vid = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Image window", grey_vid)
#use only the bottom 100 rows
    print(np.size(grey_vid))
    lower_section = grey_vid[699:799, :]
#average the values in each columns
    pixel_average = np.mean(lower_section, axis = 0)
    
#set road values to 100 while setting back to 0
    pixel_average[pixel_average <= threshold] = 100
    pixel_average[pixel_average > threshold] = 0
#find index of first instance of change in value from both ends
    first_instance = (pixel_average != 0).argmax()
    last_instance = (np.flip(pixel_average) != 0).argmax()
#robot moves so that background space is equal on both sides of road in order to stay centered
    move = Twist()
    move.linear.x = 0.5
    if first_instance == last_instance == 0:
      move.angular.z = 6.0
      move.linear.x = 0
    if first_instance > last_instance:
      move.angular.z = -0.75
    elif last_instance > first_instance:
      move.angular.z = 0.75
    elif last_instance == first_instance != 0:
      move.angular.z = 0

    self.move_pub.publish(move)


def main(args):
  ic = line_follower()
  rospy.init_node('follow_line', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    print("in progress")
