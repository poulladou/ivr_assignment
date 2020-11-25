#!/usr/bin/env python3

import cv2
import numpy as np


def blob_mask(image, LB_thresh, UB_thresh):
  #Isolate colour bounded by values LB_thresh and UB_thresh as a binary image
  mask = cv2.inRange(image, LB_thresh, UB_thresh)
  # This applies a dilate that makes the binary region larger 
  #(the more iterations the larger it becomes)
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations=3)
  return mask
  
def detect_orange(image):
  orange_mask = blob_mask(image, (5,100,100), (100,255,255))
  M = cv2.moments(orange_mask)
  #Calculate pixel coordinates for the centre of the blob
  cx = int(M['m10'] / M['m00'])
  cy = int(M['m01'] / M['m00'])
  return np.array([cx, cy])
  
def detect_red(image):
  red_mask = blob_mask(image, (0,0,100), (40,40,255))
  M = cv2.moments(red_mask)
  #Calculate pixel coordinates for the centre of the blob
  try:
    cx = int(M['m10'] / M['m00'])
  except ZeroDivisionError:
    return detect_green(image)
  try:
    cy = int(M['m01'] / M['m00'])
  except ZeroDivisionError:
    return detect_green(image)
  return np.array([cx, cy])
	
def detect_green(image):
  green_mask = blob_mask(image, (0,100,0), (40,255,40))
  M = cv2.moments(green_mask)
  #Calculate pixel coordinates for the centre of the blob
  try:
    cx = int(M['m10'] / M['m00'])
  except ZeroDivisionError:
    return detect_blue(image)
  try:
    cy = int(M['m01'] / M['m00'])
  except ZeroDivisionError:
    return detect_blue(image)
  return np.array([cx, cy])
	
def detect_blue(image):
  blue_mask = blob_mask(image, (100,0,0), (255,40,40))
  M = cv2.moments(blue_mask)
  #Calculate pixel coordinates for the centre of the blob
  try:
    cx = int(M['m10'] / M['m00'])
  except ZeroDivisionError:
    return detect_green(image)
  try:
    cy = int(M['m01'] / M['m00'])
  except ZeroDivisionError:
    return detect_green(image)
  return np.array([cx, cy])
	
def detect_yellow(image):
  yellow_mask = blob_mask(image, (0,100,100), (0,255,255))
  M = cv2.moments(yellow_mask)
  #Calculate pixel coordinates for the centre of the blob
  cx = int(M['m10'] / M['m00'])
  cy = int(M['m01'] / M['m00'])
  return np.array([cx, cy])
  
# Calculate the conversion from pixel to meter
def pixel2meter(image):
    # Obtain the centre of each coloured blob
    circle1Pos = detect_yellow(image)
    circle2Pos = detect_blue(image)
    # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    return 2.5 / np.sqrt(dist)

# Detects joint positions in specified image	
def joint_pos(image):
  yellowPos= detect_yellow(image)
  
  bluePos= detect_blue(image)
  
  greenPos= detect_green(image)
  
  redPos= detect_red(image)
  return yellowPos,bluePos,greenPos,redPos

#  Converts specified joint position to be with respect to fixed frame and in meters 
def wrt_base(basePos, jointPos, image):
  a = pixel2meter(image)
  jointPos[0] = jointPos[0]-basePos[0]
  jointPos[1] = -(jointPos[1]-basePos[1])
  return a*jointPos

# Converts joint positions to be with respect to fixed frame and in meters 
def joint_wrt_base(image):
  yellowPos,bluePos,greenPos,redPos = joint_pos(image)
  
  bluePos = wrt_base(yellowPos, bluePos, image)
  greenPos = wrt_base(yellowPos, greenPos, image) 
  redPos = wrt_base(yellowPos, redPos, image)
  yellowPos = wrt_base(yellowPos, yellowPos, image)
  return yellowPos,bluePos,greenPos,redPos

# Converts 2d coordinates to 3d	
def to3d(posImage1, posImage2):
  pos_z = (posImage1[1] + posImage2[1])/2.
  pos_y = posImage1[0]
  pos_x = posImage2[0]
  return np.array([pos_x, pos_y, pos_z])
 
# Defines the joint positions in 3d   		
def joint_pos_3d(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2):
  yellowPos = to3d(yellowPos1, yellowPos2)
  
  bluePos = to3d(bluePos1, bluePos2)
  
  greenPos = to3d(greenPos1, greenPos2)
  
  redPos = to3d(redPos1, redPos2)

  return yellowPos,bluePos,greenPos,redPos

# Calculates the joint angles
def joint_angles(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2):
  yellowPos,bluePos,greenPos,redPos= joint_pos_3d(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2)
  
  greenWRTblue= greenPos-bluePos
  
  #j2=np.arctan2(greenWRTblue[2], greenWRTblue[0]) 
  j2=-np.arctan2(-greenWRTblue[1], greenWRTblue[2]) 
  
  #j3=np.arctan2(greenWRTblue[1], greenWRTblue[2]*3.5*(-3.5)*np.sin(j2))
  j3=np.arctan2(-greenWRTblue[0]*np.sin(j2), greenWRTblue[1])
  #j3=j3 % np.pi
  
  redWRTgreen = redPos-greenPos

  #j4=np.arctan2(redWRTgreen[0]*np.sin(j3) + redWRTgreen[1]*np.cos(j2)*np.cos(j3), redWRTgreen[1]*np.sin(j2))
  j4=np.arctan2(-redWRTgreen[2]*np.sin(j3) + redWRTgreen[0]*np.cos(j2)*np.cos(j3), redWRTgreen[0]*np.sin(j2))  


  return j2,j3,j4

    		


    		
    		


