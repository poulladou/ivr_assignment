#!/usr/bin/env python3

import cv2
import numpy as np

def detect_blob(image, LB_thresh, UB_thresh):
  #Isolate colour bounded by values LB_thresh and UB_thresh as a binary image
  mask = cv2.inRange(image, LB_thresh, UB_thresh)
  # This applies a dilate that makes the binary region larger 
  #(the more iterations the larger it becomes)
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations=3)
  M = cv2.moments(mask)
  #Calculate pixel coordinates for the centre of the blob
  try:
    cx = int(M['m10'] / M['m00'])
  except ZeroDivisionError:
    return detect_orange(image)
  try:
    cy = int(M['m01'] / M['m00'])
  except ZeroDivisionError:
    return detect_orange(image)
  return np.array([cx, cy])
  
def detect_orange(image):
  orange_centre = detect_blob(image, (5,100,100), (100,255,255))

def detect_red(image):
  red_centre = detect_blob(image, (0,0,100), (40,40,255))
  return red_centre
	
def detect_green(image):
  green_centre = detect_blob(image, (0,100,0), (40,255,40))
  return green_centre
	
def detect_blue(image):
  blue_centre = detect_blob(image, (100,0,0), (255,40,40))
  return blue_centre
	
def detect_yellow(image):
  yellow_centre = detect_blob(image, (0,100,100), (0,255,255))
  return yellow_centre
  
# Calculate the conversion from pixel to meter
def pixel2meter(image):
    # Obtain the centre of each coloured blob
    circle1Pos = detect_yellow(image)
    circle2Pos = detect_blue(image)
    # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    return 2.5 / np.sqrt(dist)
	
def joint_pos(image):
  a=pixel2meter(image)
  yellowPos= a*detect_yellow(image)
  bluePos= a*detect_blue(image)
  greenPos= a*detect_green(image)
  redPos= a*detect_red(image)
  return yellowPos,bluePos,greenPos,redPos
	

    		
def joint_pos_combined(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2):
  yellowPos_z=(yellowPos1[1]+yellowPos2[1])/2.
  yellowPos1=list(yellowPos1)
  yellowPos2=list(yellowPos2)
  yellowPos1[1] =yellowPos_z
  yellowPos2[1]=yellowPos_z
  
  bluePos_z=(bluePos1[1]+bluePos2[1])/2.
  bluePos1=list(bluePos1)
  bluePos2=list(bluePos2)
  bluePos1[1] =bluePos_z
  bluePos2[1]=bluePos_z
  
  greenPos_z=(greenPos1[1]+greenPos2[1])/2.
  greenPos1=list(greenPos1)
  greenPos2=list(greenPos2)
  greenPos1[1] =greenPos_z
  greenPos2[1]=greenPos_z
  
  redPos_z=(redPos1[1]+redPos2[1])/2.
  redPos1=list(redPos1)  
  redPos1[1] =redPos_z

  return yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2
    
def joint_angles(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2):
  yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2= joint_pos_combined(yellowPos1,bluePos1,greenPos1,redPos1,yellowPos2,bluePos2,greenPos2,redPos2)
  j1=0
  
  l1_1=np.arctan2(yellowPos1[0]-bluePos1[0],yellowPos1[1]-bluePos1[1])
  j2=np.arctan2(bluePos1[0]-greenPos1[0],bluePos1[1]-greenPos1[1]) #-l1_1
  j4=np.arctan2(greenPos1[0]-redPos1[0],greenPos1[1]-redPos1[1])-j2 #-l1_1
  
  l1_2=-np.arctan2(yellowPos2[0]-bluePos2[0],yellowPos2[1]-bluePos2[1])
  j3=-(np.arctan2(bluePos2[0]-greenPos2[0],bluePos2[1]-greenPos2[1]) )#-l1_2)
  return np.array([j1,j2,j3,j4])
    		


    		
    		


