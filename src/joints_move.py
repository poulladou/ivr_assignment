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

class joint_movement:
  # defines publisher
  def __init__(self):
    rospy.init_node('joint_movement', anonymous=True)
    
    self.joint2_actual = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_actual = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_actual = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    self.rate = rospy.Rate(10)
    self.time = rospy.get_time()


  def callback(self):
    self.j2 = Float64()
    self.j3 = Float64()
    self.j4 = Float64()
    
    # publish the results
    while not rospy.is_shutdown():
      t = rospy.get_time() - self.time
      
      self.j2.data = (np.pi/2)*np.sin((t*np.pi)/15)
      self.joint2_actual.publish(self.j2.data)
      
      self.j3.data = (np.pi/2)*np.sin((t*np.pi)/18)
      self.joint3_actual.publish(self.j3.data)
      
      self.j4.data = (np.pi/2)*np.sin((t*np.pi)/20)
      self.joint4_actual.publish(self.j4.data)

      self.rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
  try:
    jm = joint_movement()
    jm.callback()
  except rospy.ROSInterruptException:
    pass
    
