#!/usr/bin/env python

import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

from line_follower.srv import DriveToTarget
from line_follower.srv import DriveToTargetRequest


bridge = CvBridge()

def drive_bot(lin_x, ang_z):
    
    rospy.wait_for_service('/line_follower/command_robot')
    drive_bot = rospy.ServiceProxy('/line_follower/command_robot', DriveToTarget)
    
    msg = drive_bot(lin_x, ang_z)
    print(msg)

def process_image_callback(ros_image):
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  ####################################################################
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  lower_yellow = numpy.array([ 10,  10,  10])
  upper_yellow = numpy.array([255, 255, 250])
  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
  
  h, w, d = cv_image.shape
  search_top = int(3*h/4)
  search_bot = int(3*h/4 + 20)
  
  mask[0:search_top, 0:w] = 0
  mask[search_bot:h, 0:w] = 0
  M = cv2.moments(mask)
  if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      linear_x = 0.6
      angular_z = -float(err) / 100
      drive_bot(linear_x, angular_z)
      # END CONTROL
  
  ####################################################################    
  
  cv2.imshow("Image window", cv_image)
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('process_image')
  image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, process_image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
