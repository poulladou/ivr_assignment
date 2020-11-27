#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import sympy as sym
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math
from image_processing import *
from target_detection import find_targ_pos
from transformation_matrices import forward_kinematics

class closed_loop_control:

    def __init__(self):
        rospy.init_node('closed_loop_control', anonymous=True)
        self.end_effector_pub = rospy.Publisher('end_effector_position', Float64MultiArray, queue_size=10)
        self.target_pub = rospy.Publisher('target', Float64MultiArray, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)
        self.bridge = CvBridge()
        self.image1_sub = rospy.Subscriber('/camera1/robot/image_raw', Image, self.callback1)
        self.image2_sub = rospy.Subscriber('/camera2/robot/image_raw', Image, self.callback2)
        self.cv_image1 = None
        self.cv_image2 = None
        self.time_target = rospy.get_time()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0,0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64')

    def detect_end_effector(self, im1, im2):
        yel1, blu1, gre1, red1 = joint_wrt_base(im1)
        yel2, blu2, gre2, red2 = joint_wrt_base(im2)
        return to3d(red1, red2)

    def get_joint_angles(self, im1, im2):
        yel1, blu1, gre1, red1 = joint_wrt_base(im1)
        yel2, blu2, gre2, red2 = joint_wrt_base(im2)
        return joint_angles(yel1, blu1, gre1, red1, yel2, blu2, gre2, red2)

    def calc_jacobian(self, q):
        fk = sym.Matrix(forward_kinematics())
        theta1, theta2, theta3, theta4 = sym.symbols('theta1, theta2, theta3, theta4')
        joints = sym.Array([theta1, theta2, theta3, theta4])
        jaco = fk.jacobian(joints)
        jaco = jaco.subs(theta1, q[0])
        jaco = jaco.subs(theta2, q[1])
        jaco = jaco.subs(theta3, q[2])
        jaco = jaco.subs(theta4, q[3])
        jaco = np.array(jaco, dtype='float64')
        return jaco

    def control_closed(self, im1, im2):
        K_p = np.array([[2, 0, 0], [0, 2, 0], [0, 0, 2]])
        K_d = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        current_time = np.array([rospy.get_time()])
        dt = current_time - self.time_previous_step
        self.time_previous_step = current_time
        pos = self.detect_end_effector(im1, im2)
        pos_d = find_targ_pos(im1, im2)
        self.error_d = ((pos_d - pos) - self.error)/dt
        self.error = pos_d - pos
        j2, j3, j4 = self.get_joint_angles(im1, im2)
        q = np.array([0, j2, j3, j4])
        J_inv = np.linalg.pinv(self.calc_jacobian(q))
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))
        q_d = q + (dt * dq_d)
        return q_d

    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        q_d = self.control_closed(self.cv_image1, self.cv_image2)
        self.joint2 = Float64()
        self.joint2.data = q_d[1]
        self.joint3 = Float64()
        self.joint3.data = q_d[2]
        self.joint4 = Float64()
        self.joint4.data = q_d[3]
        cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(3)

        try:
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

def main(args):
    cc = closed_loop_control()
    try:
        print('Starting')
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)