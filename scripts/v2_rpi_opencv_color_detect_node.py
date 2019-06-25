#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rpi_opencv_cam')
import sys
import rospy
#import ConfigParser
import cv2 as cv
import numpy as np
import argparse
import random as rng
rng.seed(12345)
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# uses raspicam node and command line arguments specifying desired colors
# masks and draws bounding boxes around objects of each desired color
# publishes results to individual ROS topics

# numpy arrays corresponding to upper/lower bounds of desired colors
# later used to make color masks
lower_red = np.array([160,100,50])
upper_red = np.array([180,255,255])
lower_blue = np.array([100,150,0])
upper_blue = np.array([140,255,255])

# all available colors
colors = ['red','blue']

# dictionary of topic publishers, used to publish result for each desired color
# updated in __init__
color_topics = {}

class image_converter:

  def __init__(self):
  # use command line arguments to specify which colors to look for
    try:
        for color in sys.argv[1:]:
          if color in colors:
            color_topics[color] = rospy.Publisher(color + "_mask", Image, queue_size=10)
          else:
            print (color + " is not a valid color")
    except IndexError:
      # unsure why this line doesn't work
      print ("No color given")
      rospy.signal_shutdown("No color given")

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("raspicam_node/image",Image,self.callback)

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # use this if switching from command line to an input / config file
#    config = ConfigParser.ConfigParser()

   # dictionary of colors that map to opencv masks
   # might be better to have this in a separate file?
   # maybe some way to automate the generation of this?
   # since we have a list of colors, and each color has an upper_ and lower_ defined,
   # could you loop thru the color list and build up the color_filter dictionary?
    color_filters = {
    'red': cv.inRange(hsv, lower_red, upper_red),
    'blue': cv.inRange(hsv, lower_blue, upper_blue)
    }
    try:
      for color in sys.argv[1:]:
        try:
          # do color filtering
          mask = color_filters[color]
          result = cv.bitwise_and(frame, frame, mask = mask)
          # trying to get rid of static / issues
          kernel = np.ones((5,5), np.uint8)
          result = cv.dilate(result, kernel, iterations=2)
          result = cv.morphologyEx(result, cv.MORPH_CLOSE, kernel)
          # thresholding for bounding box
          threshold = 130
          blur = cv.blur(result, (3,3))
          canny_output = cv.Canny(blur, threshold, threshold * 2)
          # in opencv 4, did not need "im2"
          im2, contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
          drawing = result
          # draw only biggest contour (used while working with 1 object)
          if len(contours) != 0:
            # draw in blue the contours that were found
            cv.drawContours(drawing, contours, -1, 255, 3)
            # find the biggest area
            c = max(contours, key = cv.contourArea)
            x,y,w,h = cv.boundingRect(c)
            # draw the biggest bounding box (in green)
            cv.rectangle(drawing,(x,y),(x+w,y+h),(0,255,0),2)

# old code for drawing all found bounding boxes / contours
#          contours_poly = [None]*len(contours)
#          boundRect = [None]*len(contours)
#          for i, c in enumerate(contours):
#            contours_poly[i] = cv.approxPolyDP(c, 3, True)
#            boundRect[i] = cv.boundingRect(contours_poly[i])
#
#          drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
#          for i in range(len(contours)):
#            randColor = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
#            cv.drawContours(drawing, contours_poly, i, randColor)
#            cv.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
#            (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), randColor, 2)
          
          try:
            # publish result to topic that "color" maps to
            color_topics[color].publish(self.bridge.cv2_to_imgmsg(drawing, "bgr8"))
          except CvBridgeError as e:
            print(e)

# old code for displaying results locally
#          cv.imshow("bounding boxes", drawing)
#          cv.imshow(color, result)
#          cv.waitKey(1)

        except KeyError:
          pass
    except IndexError:
        print ("No color given")
        rospy.signal_shutdown("No color given")

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


