#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math

# isolate targets
def detect_orange(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
	return mask

# isolate blue for conversion calculation in conjuction with yellow joint
def detect_blue(image):
	mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.dilate(mask, kernel, iterations=3)
	M = cv2.moments(mask)
	cx = int(M['m10'] / M['m00'])
	cy = int(M['m01'] / M['m00'])
	return np.array([cx, cy])

# isolate yellow for conversion calculation in conjuction with blue joint
def detect_yellow(image):
	mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.dilate(mask, kernel, iterations=3)
	M = cv2.moments(mask)
	cx = int(M['m10'] / M['m00'])
	cy = int(M['m01'] / M['m00'])
	return np.array([cx, cy])

# isolate yellow and then calculate circle position with respect to top left of image
def detect_yellow_forbase(image):
	mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
	circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 500, param1=100, param2=1, minRadius=1, maxRadius=15)
	circles = np.round(circles[0, :].astype("int"))
	(h, z, r) = circles[0]
	return np.array([y, h])

# detect the spherical target out of both targets
def detect_circle_target(image):
	mask = detect_orange(image)
	circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 500, param1=100, param2=10, minRadius=1, maxRadius=15)
	if circles is not None:
		circles = np.round(circles[0, :].astype("int"))
		(h, z, r) = circles[0]
	else:
		h = 0
		z = -1

	return np.array([h, z])

# convert pixel to metres
def pixel2metre(image):
	centre1 = detect_yellow(image)
	centre2 = detect_blue(image)
	distance = np.sqrt(np.sum((centre2 - centre1) ** 2))
	return 2.5 / distance

# find position of targets using both image 1 and 2
def find_targ_pos(image1, image2):
	coords_xz = detect_circle_target(image2)
	coords_yz = detect_circle_target(image1)
	coords_base2 = detect_yellow(image2)
	coords_base1 = detect_yellow(image1)
	converter = pixel2metre(image1)
	z_converted = False
	if coords_xz[1] != -1:
		target_wrt_base = coords_xz - coords_base2
		targetx = converter * target_wrt_base[0]
		targetz = converter * target_wrt_base[1] * -1
		z_converted = True
	else:
		targetx = 0

	if coords_yz[1] != -1:
		target_wrt_base = coords_yz - coords_base1
		targety = converter * target_wrt_base[0]
		if z_converted == False:
			targetz = converter * target_wrt_base[1] * -1
	else:
		targety = 0
	
	return np.array([targetx, targety, targetz])