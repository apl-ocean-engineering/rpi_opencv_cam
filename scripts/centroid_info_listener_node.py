#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rpi_opencv_cam.msg import centroid

# subscribes to centroid_data topic and prints out info

def callback(data):
#  rospy.loginfo("object found, center at " + data.x + ", " + data.y + ", color is " + data.color)
  rospy.loginfo("object found, center at %d, %d and color is %s" % (data.x, data.y, data.color))

def listener():
  rospy.init_node('centroid_listener', anonymous=True)
  rospy.Subscriber("centroid_data", centroid, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
