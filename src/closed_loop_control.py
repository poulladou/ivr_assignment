#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import sympy as sym
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math
from image_processing import *
from target_detection import find_targ_pos
from transformation_matrices import forward_kinematics

class closed_loop_control:

    def __init__(self):
        rospy.init_node('closed_loop_control', anonymous=True)
        self.end_effector_pubx = rospy.Publisher('end_effector_positionx', Float64, queue_size=10)
        self.end_effector_puby = rospy.Publisher('end_effector_positiony', Float64, queue_size=10)
        self.end_effector_pubz = rospy.Publisher('end_effector_positionz', Float64, queue_size=10)
        self.target_pubx = rospy.Publisher('targetx', Float64, queue_size=10)
        self.target_puby = rospy.Publisher('targety', Float64, queue_size=10)
        self.target_pubz = rospy.Publisher('targetz', Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)
        self.bridge = CvBridge()
        self.joints = rospy.Subscriber('/robot/joint_states', JointState, self.callback)
        self.joints_actual = None
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
        pg = 1
        dg = 0.5
        K_p = np.array([[pg, 0, 0], [0, pg, 0], [0, 0, pg]])
        K_d = np.array([[dg, 0, 0], [0, dg, 0], [0, 0, dg]])
        current_time = np.array([rospy.get_time()])
        dt = current_time - self.time_previous_step
        self.time_previous_step = current_time
        pos = self.detect_end_effector(im1, im2)
        pos_d = find_targ_pos(im1, im2)
        self.error_d = ((pos_d - pos) - self.error)/dt
        self.error = pos_d - pos
        q = np.array([self.joints_actual[0], self.joints_actual[1], self.joints_actual[2], self.joints_actual[3]])
        J_inv = np.linalg.pinv(self.calc_jacobian(q))
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))
        q_d = q + (dt * dq_d)
        return q_d

    def callback(self, data):
        self.joints_actual = data.position

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

        self.end_effector_posx = Float64()
        self.end_effector_posx.data = self.detect_end_effector(self.cv_image1, self.cv_image2)[0]
        self.end_effector_posy = Float64()
        self.end_effector_posy.data = self.detect_end_effector(self.cv_image1, self.cv_image2)[1]
        self.end_effector_posz = Float64()
        self.end_effector_posz.data = self.detect_end_effector(self.cv_image1, self.cv_image2)[2]
        self.target_posx = Float64()
        self.target_posx.data = find_targ_pos(self.cv_image1, self.cv_image2)[0]
        self.target_posy = Float64()
        self.target_posy.data = find_targ_pos(self.cv_image1, self.cv_image2)[1]
        self.target_posz = Float64()
        self.target_posz.data = find_targ_pos(self.cv_image1, self.cv_image2)[2]

        try:
            self.end_effector_pubx.publish(self.end_effector_posx)
            self.end_effector_puby.publish(self.end_effector_posy)
            self.end_effector_pubz.publish(self.end_effector_posz)
            self.target_pubx.publish(self.target_posx)
            self.target_puby.publish(self.target_posy)
            self.target_pubz.publish(self.target_posz)
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
