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
from image_processing import joint_wrt_base


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
    self.yellowPos_pub1 = rospy.Publisher("yellowPos_yz", Float64MultiArray, queue_size=10)
    self.bluePos_pub1 = rospy.Publisher("bluePos_yz", Float64MultiArray, queue_size=10)
    self.greenPos_pub1 = rospy.Publisher("greenPos_yz", Float64MultiArray, queue_size=10)
    self.redPos_pub1 = rospy.Publisher("redPos_yz", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge() 
    
   
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', self.cv_image1)
    
    yellowPos,bluePos,greenPos,redPos=joint_wrt_base(self.cv_image1)
    
    
    #print("yellow: ",yellowPos)
    #print("blue: ", bluePos)
    #print("green: ", greenPos) 
    #print("red: ", redPos)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    
    self.yellowPos1=Float64MultiArray()
    self.yellowPos1.data=yellowPos
    
    self.bluePos1=Float64MultiArray()
    self.bluePos1.data=bluePos
    
    self.greenPos1=Float64MultiArray()
    self.greenPos1.data=greenPos
    
    self.redPos1=Float64MultiArray()
    self.redPos1.data=redPos
    
    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.yellowPos_pub1.publish(self.yellowPos1)      
      self.bluePos_pub1.publish(self.bluePos1)
      self.greenPos_pub1.publish(self.greenPos1)
      self.redPos_pub1.publish(self.redPos1)     
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


