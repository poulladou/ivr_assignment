#!/usr/bin/env python3

import cv2
import numpy as np
from operator import itemgetter

# Isolates circles 
def get_circles(image):
  #Isolates black colour
  mask=cv2.inRange(image, (0,0,0), (15,15,15))
  #Gets circles centres and radius
  circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=100,param2=7,minRadius=1, maxRadius=17)
  if circles is not None: 
    circles = np.uint16(np.around(circles))
  return circles
 
# Gets centres of joints when only 2 circles are isolated
def two_circles(circles):
  circle1 = circles[0,0,:2]
  circle2 = circles[0,1,:2]
  if circle1[1]<circle2[1]:
    redPos = circle1
    greenPos = circle1
    bluePos = circle1
    yellowPos = circle2
  else:
    redPos = circle2
    greenPos = circle2
    bluePos = circle2
    yellowPos = circle1
  return redPos, greenPos, bluePos, yellowPos

# Gets centres of joints when only 3 circles are isolated  
def three_circles(circles):
  circle1 = circles[0,0,:2]
  circle2 = circles[0,1,:2]
  circle3 = circles[0,2,:2]
  circles = [circle1, circle2, circle3]
  sorted_circles = sorted(circles, key=itemgetter(1))
  link_ratio = sorted_circles[1][1]-sorted_circles[0][1]/sorted_circles[2][1]-sorted_circles[1][1]
  if link_ratio<1.3:
    redPos = sorted_circles[0]
    greenPos = sorted_circles[1]
    bluePos = sorted_circles[1]
    yellowPos = sorted_circles[2]
  else:
    redPos = sorted_circles[0]
    greenPos = sorted_circles[1]
    bluePos = sorted_circles[2]
    yellowPos = sorted_circles[2]
  return redPos, greenPos, bluePos, yellowPos

# Gets centres of joints when only 4 circles are isolated 
def four_circles(circles):
  circle1 = circles[0,0,:2]
  circle2 = circles[0,1,:2]
  circle3 = circles[0,2,:2]
  circle4 = circles[0,3,:2]
  circles = [circle1, circle2, circle3, circle4]
  sorted_circles = sorted(circles, key=itemgetter(1))
  circle1 = sorted_circles[0]
  circle2 = sorted_circles[1]
  circle3 = sorted_circles[2]
  circle4 = sorted_circles[3]
  return circle1, circle2, circle3, circle4
  
# Gets centres of joints when more circles are isolated
def other_circles(circles):
  num_circles = len(circles[0])
  if num_circles>4:
    circle1, circle2, circle3, circle4 = four_circles(circles)
  else:
    circle1 = circles[0,0,:2]
    circle2 = circles[0,0,:2]
    circle3 = circles[0,0,:2]
    circle4 = circles[0,0,:2]
  return circle1, circle2, circle3, circle4

# Gets centres of joints  
def get_joints(image):
  circles = get_circles(image)
  num_circles = len(circles[0])
  if num_circles==2:
    circle1, circle2, circle3, circle4 = two_circles(circles)
  elif num_circles==3:
    circle1, circle2, circle3, circle4 = three_circles(circles)
  elif num_circles==4:
    circle1, circle2, circle3, circle4 = four_circles(circles)
  else:
    circle1, circle2, circle3, circle4 = other_circles(circles) 
    print('Too many or too few circles detected')
  return circle1, circle2, circle3, circle4 , num_circles
  
# Converts 2d coordinates to 3d	
def to3d(posImage1, posImage2):
  pos_z = (posImage1[1] + posImage2[1])/2.
  pos_y = posImage1[0]
  pos_x = posImage2[0]
  return np.array([pos_x, pos_y, pos_z])
  
# Defines the joint positions in 3d   		
def joint_pos_3d(redPos1, greenPos1, bluePos1, yellowPos1,redPos2, greenPos2, bluePos2, yellowPos2):
  yellowPos = to3d(yellowPos1, yellowPos2)
  
  bluePos = to3d(bluePos1, bluePos2)
  
  greenPos = to3d(greenPos1, greenPos2)
  
  redPos = to3d(redPos1, redPos2)

  return redPos, greenPos, bluePos, yellowPos

# Conversion from pixel to meter
def pixel2meter():
  a = 0.03787878787878788
  return a

#  Converts specified joint position to be with respect to fixed frame and in meters 
def wrt_base(basePos, jointPos):
  a = pixel2meter()
  jointPos[0] = jointPos[0]-basePos[0]
  jointPos[1] = jointPos[1]-basePos[1]
  jointPos[2] = -(jointPos[2]-basePos[2])
  return a*jointPos
  
# Converts joint positions to be with respect to fixed frame 
def joint_wrt_base(redPos, greenPos, bluePos, yellowPos): 
  bluePos = wrt_base(yellowPos, bluePos)
  greenPos = wrt_base(yellowPos, greenPos) 
  redPos = wrt_base(yellowPos, redPos)
  yellowPos = wrt_base(yellowPos, yellowPos)
  return redPos, greenPos, bluePos, yellowPos

# Finds euclidean distance between two circles  
def euclid_dist(circle1, circle2):
  dist = np.sum((circle1 - circle2)**2)
  return np.sqrt(dist)

# Groups joints based on the link they're attached to  
def group_joints(circle1, circle2, circle3, circle4):
  a = pixel2meter()
  circles12 = a*euclid_dist(circle1, circle2)
  print(circles12)
  circles13 = a*euclid_dist(circle1, circle3)
  print(circles13)
  circles14 = a*euclid_dist(circle1, circle4)
  print(circles14)
  circles23 = a*euclid_dist(circle2, circle3)
  print(circles23)
  circles24 = a*euclid_dist(circle2, circle4)
  print(circles24)
  circles34 = a*euclid_dist(circle3, circle4)
  print(circles34)
  print()
  if 2.25<= circles12 and circles12<2.75:
    link1=[circle1, circle2]
    
  elif 2.25<= circles13 and circles13<2.75:
    link1=[circle1, circle3]
    
  elif 2.25<= circles14 and circles14<2.75:
    link1=[circle1, circle4]
    
  elif 2.25<= circles23 and circles23<2.75:
    link1=[circle2, circle3]
    
  elif 2.25<= circles24 and circles24<2.75:
    link1=[circle2, circle4]
      
  elif 2.25<= circles34 and circles34<2.75:
    link1=[circle3, circle4]
  else:
    link1 = [circle4, circle3]
    
    
  if 2.75<= circles12 and circles12<3.25:
    link3=[circle1, circle2]
    
  elif 2.75<= circles13 and circles13<3.25:
    link3=[circle1, circle3]
    
  elif 2.75<= circles14 and circles14<3.25:
    link3=[circle1, circle4]
    
  elif 2.75<= circles23 and circles23<3.25:
    link3=[circle2, circle3]
    
  elif 2.75<= circles24 and circles24<3.25:
    link3=[circle2, circle4]
      
  elif 2.75<= circles34 and circles34<3.25:
    link3=[circle3, circle4] 
  else:
    link3 = [circle2, circle1]
    
    
  if 3.25<= circles12 and circles12<3.75:
    link2=[circle1, circle2]
    
  elif 3.25<= circles13 and circles13<3.75:
    link2=[circle1, circle3]
    
  elif 3.25<= circles14 and circles14<3.75:
    link2=[circle1, circle4]
    
  elif 3.25<= circles23 and circles23<3.75:
    link2=[circle2, circle3]
    
  elif 3.25<= circles24 and circles24<3.75:
    link2=[circle2, circle4]
      
  elif 3.25<= circles34 and circles34<3.75:
    link2=[circle3, circle4]
  else:
    link2 = [circle3, circle2]
    
  return link1, link2, link3

# Identifies each joint as red, green, blue and yellow
def compare_joints(circle1, circle2, circle3, circle4):
  link1, link2, link3 = group_joints(circle1, circle2, circle3, circle4) 
  link1 = list(link1)
  link2 = list(link2)
  link3 = list(link3)
  if (link1[0] == link2[0]).all():
    bluePos = link1[0]
    yellowPos = link1[1]
    greenPos = link2[1]
    if (link2[1] == link3[0]).all():
      redPos = link3[1]
    else:
      redPos = link3[0]
  elif (link1[0] == link2[1]).all():
    bluePos = link1[0]
    yellowPos = link1[1]
    greenPos = link2[0]
    if (link2[0] == link3[0]).all():
      redPos = link3[1]
    else:
      redPos = link3[0]
  elif (link1[1] == link2[0]).all():
    bluePos = link1[1]
    yellowPos = link1[0] 
    greenPos = link2[1] 
    if (link2[1] == link3[0]).all():
      redPos = link3[1]
    else:
      redPos = link3[0]  
  elif (link1[1] == link2[1]).all():
    bluePos = link1[1]
    yellowPos = link1[0] 
    greenPos = link2[0]
    if (link2[0] == link3[0]).all():
      redPos = link3[1]
    else:
      redPos = link3[0]
  else:
    redPos = link3[1]
    greenPos = link3[0]
    bluePos = link1[1]
    yellowPos = link1[0]
      
  return redPos, greenPos, bluePos, yellowPos
  
# Identifies each joint as red, green, blue and yellow
def identify_joints(joints1, num_circles1, joints2, num_circles2):
  circle1_1, circle2_1, circle3_1, circle4_1 = joints1
  circle1_2, circle2_2, circle3_2, circle4_2 = joints2
  # joints from two images combined
  circle1, circle2, circle3, circle4 = joint_pos_3d(circle1_1, circle2_1, circle3_1, circle4_1,circle1_2, circle2_2, circle3_2, circle4_2)
  # joints identified, converted to be with respect to fixed frame and in meter
  redPos, greenPos, bluePos, yellowPos = compare_joints(circle1, circle2, circle3, circle4)
  redPos, greenPos, bluePos, yellowPos = joint_wrt_base(redPos, greenPos, bluePos, yellowPos)

  return redPos, greenPos, bluePos, yellowPos
  
# Calculates the joint angles
def joint_angles(redPos, greenPos, bluePos, yellowPos):
  greenWRTblue= greenPos-bluePos

  j2=-np.arctan2(-greenWRTblue[1], greenWRTblue[2]) 
  
  j3=np.arctan2(-greenWRTblue[0]*np.sin(j2), greenWRTblue[1])
  
  redWRTgreen = redPos-greenPos

  j4=np.arctan2(-redWRTgreen[2]*np.sin(j3) + redWRTgreen[0]*np.cos(j2)*np.cos(j3), redWRTgreen[0]*np.sin(j2))  

  return j2,j3,j4 
  
  
  
  
