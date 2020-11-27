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

from image_processing_4 import identify_joints, joint_angles

class Server:
  def __init__(self):
    self.circle1_1=None  
    self.circle2_1=None
    self.circle3_1=None
    self.circle4_1=None
    self.num_circles_1=None
    
    self.circle1_2=None    
    self.circle2_2=None
    self.circle3_2=None
    self.circle4_2=None
    self.num_circles_2=None
    
    self.joint2_est= rospy.Publisher("joint2_est", Float64, queue_size=10)
    self.joint3_est= rospy.Publisher("joint3_est", Float64, queue_size=10)
    self.joint4_est= rospy.Publisher("joint4_est", Float64, queue_size=10)
    
  def callback1_circle1(self,msg):
    self.circle1_1=msg
    self.compute_angles()
    
  def callback1_circle2(self,msg):
    self.circle2_1=msg
    self.compute_angles()
    
  def callback1_circle3(self,msg):
    self.circle3_1=msg
    self.compute_angles()   
    
  def callback1_circle4(self,msg):
    self.circle4_1=msg
    self.compute_angles() 
    
  def callback1_num_circles(self,msg):
    self.num_circles_1=msg
    self.compute_angles()  
    
  def callback2_circle1(self,msg):
    self.circle1_2=msg
    self.compute_angles()  
    
  def callback2_circle2(self,msg):
    self.circle2_2=msg
    self.compute_angles()    
    
  def callback2_circle3(self,msg):
    self.circle3_2=msg
    self.compute_angles()    
    
  def callback2_circle4(self,msg):
    self.circle4_2=msg
    self.compute_angles() 
    
  def callback2_num_circles(self,msg):
    self.num_circles_2=msg
    self.compute_angles()    
    
    
  def compute_angles(self):
    if self.circle1_1 is not None and self.circle2_1 is not None and self.circle3_1 is not None and self.circle4_1 is not None and self.num_circles_1 is not None and self.circle1_2 is not None and self.circle2_2 is not None and self.circle3_2 is not None and self.circle4_2 is not None and self.num_circles_2 is not None:
      
      
      j2=Float64()
      j3=Float64()
      j4=Float64()
      
      joints1 = (self.circle1_1.data, self.circle2_1.data, self.circle3_1.data, self.circle4_1.data)
      
      joints2 = (self.circle1_2.data, self.circle2_2.data, self.circle3_2.data, self.circle4_2.data)
      
      redPos, greenPos, bluePos, yellowPos = identify_joints(joints1, self.num_circles_1.data, joints2, self.num_circles_2.data)
      
      j2.data, j3.data, j4.data =joint_angles(redPos, greenPos, bluePos, yellowPos)
      
      self.joint2_est.publish(j2)
      self.joint3_est.publish(j3)
      self.joint4_est.publish(j4)
      
    
if __name__=="__main__":
 
  server=Server()
  
  rospy.init_node('angles', anonymous=True)
  
  rospy.Subscriber("/circle1_yz", Float64MultiArray, server.callback1_circle1)  
  rospy.Subscriber("/circle2_yz", Float64MultiArray, server.callback1_circle2)
  rospy.Subscriber("/circle3_yz", Float64MultiArray, server.callback1_circle3)
  rospy.Subscriber("/circle4_yz", Float64MultiArray, server.callback1_circle4)
  rospy.Subscriber("/num_circles_yz", Float64, server.callback1_num_circles)  
  
  rospy.Subscriber("/circle1_xz", Float64MultiArray, server.callback2_circle1)
  rospy.Subscriber("/circle2_xz", Float64MultiArray, server.callback2_circle2)
  rospy.Subscriber("/circle3_xz", Float64MultiArray, server.callback2_circle3)
  rospy.Subscriber("/circle4_xz", Float64MultiArray, server.callback2_circle4)
  rospy.Subscriber("/num_circles_xz", Float64, server.callback2_num_circles)  
  
  
  rospy.spin()
  

  
