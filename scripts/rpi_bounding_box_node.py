#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rpi_opencv_cam')
import sys
import rospy
import cv2 as cv
import numpy as np
import argparse
import random as rng
rng.seed(12345)
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# mainly adapting online tutorial for bounding boxes and contours to work with ROS
# and raspicam node

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("bounding_boxes",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("raspicam_node/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    cv_image = cv.blur(cv_image, (3,3))
    threshold = 125
    canny_output = cv.Canny(cv_image, threshold, threshold * 2)
    contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
  
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    for i, c in enumerate(contours):
      contours_poly[i] = cv.approxPolyDP(c, 3, True)
      boundRect[i] = cv.boundingRect(contours_poly[i])


    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
      color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
      cv.drawContours(drawing, contours_poly, i, color)
      cv.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
        (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)

    cv_image = drawing



    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

