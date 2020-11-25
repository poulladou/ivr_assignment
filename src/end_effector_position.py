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

from image_processing import to3d

class Server:
  def __init__(self):
    self.redPos1=None

    self.redPos2=None
   
    
  def callback1_redPos(self,msg):
    self.redPos1=msg
    self.compute_position()   
   
    
  def callback2_redPos(self,msg):
    self.redPos2=msg
    self.compute_position()    
    
    
  def compute_position(self):
    if self.redPos1 is not None and  self.redPos2 is not None:
      
      redPos=Float64MultiArray()
      
      
      redPos.data = to3d(self.redPos1.data,self.redPos2.data)
      
      print(redPos)
          
    
if __name__=="__main__":
 
  server=Server()
  
  rospy.init_node('end_effector_position')
  
  rospy.Subscriber("/redPos_yz", Float64MultiArray, server.callback1_redPos)
  rospy.Subscriber("/redPos_xz", Float64MultiArray, server.callback2_redPos)
  
  rospy.spin()
  

  
