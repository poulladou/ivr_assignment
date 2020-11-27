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
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize publishers to send joint position data to topics
    self.yellowPos_pub2 = rospy.Publisher("yellowPos_xz", Float64MultiArray, queue_size=10)    
    self.bluePos_pub2 = rospy.Publisher("bluePos_xz", Float64MultiArray, queue_size=10)
    self.greenPos_pub2 = rospy.Publisher("greenPos_xz", Float64MultiArray, queue_size=10)
    self.redPos_pub2 = rospy.Publisher("redPos_xz", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
    yellowPos,bluePos,greenPos,redPos=joint_wrt_base(self.cv_image2)
    
    
    #print("yellow: ",yellowPos)
    #print("blue: ", bluePos)
    #print("green: ", greenPos) 
    #print("red: ", redPos)
    
    
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    
    self.yellowPos2=Float64MultiArray()
    self.yellowPos2.data=yellowPos
    
    self.bluePos2=Float64MultiArray()
    self.bluePos2.data=bluePos
    
    self.greenPos2=Float64MultiArray()
    self.greenPos2.data=greenPos
    
    self.redPos2=Float64MultiArray()
    self.redPos2.data=redPos

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.yellowPos_pub2.publish(self.yellowPos2)
      self.bluePos_pub2.publish(self.bluePos2)
      self.greenPos_pub2.publish(self.greenPos2)
      self.redPos_pub2.publish(self.redPos2)  
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


