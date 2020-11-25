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

from image_processing import joint_angles

class Server:
  def __init__(self):
    self.yellowPos1=None  
    self.bluePos1=None
    self.greenPos1=None
    self.redPos1=None
    
    self.yellowPos2=None    
    self.bluePos2=None
    self.greenPos2=None
    self.redPos2=None
    
    #self.joint_angles=rospy.Publisher("joint_angles",Float64MultiArray, queue_size=10)
    self.joint2_est= rospy.Publisher("joint2_est", Float64, queue_size=10)
    self.joint3_est= rospy.Publisher("joint3_est", Float64, queue_size=10)
    self.joint4_est= rospy.Publisher("joint4_est", Float64, queue_size=10)
    
  def callback1_yellowPos(self,msg):
    self.yellowPos1=msg
    self.compute_angles()
    
  def callback1_bluePos(self,msg):
    self.bluePos1=msg
    self.compute_angles()
    
  def callback1_greenPos(self,msg):
    self.greenPos1=msg
    self.compute_angles()   
    
  def callback1_redPos(self,msg):
    self.redPos1=msg
    self.compute_angles()   
    
  def callback2_yellowPos(self,msg):
    self.yellowPos2=msg
    self.compute_angles()  
    
  def callback2_bluePos(self,msg):
    self.bluePos2=msg
    self.compute_angles()    
    
  def callback2_greenPos(self,msg):
    self.greenPos2=msg
    self.compute_angles()    
    
  def callback2_redPos(self,msg):
    self.redPos2=msg
    self.compute_angles()    
    
    
  def compute_angles(self):
    if self.yellowPos1 is not None and self.bluePos1 is not None and self.greenPos1 is not None and self.redPos1 is not None and self.yellowPos2 is not None and self.bluePos2 is not None and self.greenPos2 is not None and self.redPos2 is not None:
      
      #angles=Float64MultiArray()
      
      j2=Float64()
      j3=Float64()
      j4=Float64()
      
      j2.data, j3.data, j4.data =joint_angles(self.yellowPos1.data, self.bluePos1.data, self.greenPos1.data, self.redPos1.data, self.yellowPos2.data, self.bluePos2.data, self.greenPos2.data, self.redPos2.data)
      
      #print(angles)
      
      #self.joint_angles.publish(angles)
      self.joint2_est.publish(j2)
      self.joint3_est.publish(j3)
      self.joint4_est.publish(j4)
      
    
if __name__=="__main__":
 
  server=Server()
  
  rospy.init_node('angles')
  
  rospy.Subscriber("/yellowPos_yz", Float64MultiArray, server.callback1_yellowPos)  
  rospy.Subscriber("/bluePos_yz", Float64MultiArray, server.callback1_bluePos)
  rospy.Subscriber("/greenPos_yz", Float64MultiArray, server.callback1_greenPos)
  rospy.Subscriber("/redPos_yz", Float64MultiArray, server.callback1_redPos)
  rospy.Subscriber("/yellowPos_xz", Float64MultiArray, server.callback2_yellowPos)
  rospy.Subscriber("/bluePos_xz", Float64MultiArray, server.callback2_bluePos)
  rospy.Subscriber("/greenPos_xz", Float64MultiArray, server.callback2_greenPos)
  rospy.Subscriber("/redPos_xz", Float64MultiArray, server.callback2_redPos)
  
  rospy.spin()
  

  
