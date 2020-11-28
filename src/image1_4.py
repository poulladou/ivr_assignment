#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from image_processing_4 import get_joints


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize publishers to send joint position data to topics
    self.circle1_pub1 = rospy.Publisher("circle1_yz", Float64MultiArray, queue_size=10)
    self.circle2_pub1 = rospy.Publisher("circle2_yz", Float64MultiArray, queue_size=10)
    self.circle3_pub1 = rospy.Publisher("circle3_yz", Float64MultiArray, queue_size=10)
    self.circle4_pub1 = rospy.Publisher("circle4_yz", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
  # Calculate the conversion from pixel to meter
  def pixel2meter(self, image):
    # Obtain the centre of each coloured blob
    circle1, circle2, circle3, circle4 = get_joints(image)
    # find the distance between two circles
    dist = np.sum((circle4 - circle3)**2)
    return 2.5 / np.sqrt(dist) 
    
   
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', self.cv_image1)
    

      
    #print(self.pixel2meter(self.cv_image1))
    
    circle1, circle2, circle3, circle4 = get_joints(self.cv_image1)
    

    #print("circle1: ",circle1)
    #print("circle2: ", circle2)
    #print("circle3: ", circle3) 
    #print("circle4: ", circle4)
    
    #shows circles detected
    img = self.cv_image1
    im_b=cv2.inRange(img, (0,0,0), (15,15,15))
    circles = cv2.HoughCircles(im_b,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=100,param2=7,minRadius=1, maxRadius=17) 
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
      cv2.circle(img, (i[0], i[1]), i[2], (0,255,0),2)
      cv2.circle(img, (i[0], i[1]),2, (0,0,255),3)
    cv2.imshow('detected circles1', img)

    #im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    
    self.circle1_1=Float64MultiArray()
    self.circle1_1.data=circle1
    
    self.circle2_1=Float64MultiArray()
    self.circle2_1.data=circle2
    
    self.circle3_1=Float64MultiArray()
    self.circle3_1.data=circle3
    
    self.circle4_1=Float64MultiArray()
    self.circle4_1.data=circle4
    
    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.circle1_pub1.publish(self.circle1_1)      
      self.circle2_pub1.publish(self.circle2_1)
      self.circle3_pub1.publish(self.circle3_1)
      self.circle4_pub1.publish(self.circle4_1)    
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


