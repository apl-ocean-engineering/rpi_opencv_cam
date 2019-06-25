#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rpi_opencv_cam')
import sys
import rospy
import ConfigParser
import cv2 as cv
import numpy as np
import argparse
import random as rng
rng.seed(12345)
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# first version of color masking node for raspicam, no bounding boxes

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("red_mask",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("raspicam_node/image",Image,self.callback)

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower_red0 = np.array([0,50,50])
    upper_red0 = np.array([10,255,255])
    lower_red1 = np.array([170,50,50])
    upper_red1 = np.array([180,255,255])

    lower_purple = np.array([264,64,0])
    upper_purple= np.array([264,64,100])

    mask0 = cv.inRange(hsv, lower_red0, upper_red0)
    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask = mask0 + mask1

#    mask = cv.inRange(hsv, lower_red, upper_red)
#    mask = cv.inRange(hsv, lower_purple, upper_purple)
    res = cv.bitwise_and(frame, frame, mask = mask)

#    cv.imshow('image', frame)
#    cv.imshow('mask', mask)
#    cv.imshow('result', res)
#    cv.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  # pretty sure you must init a node in order to recieve data from a topic
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


