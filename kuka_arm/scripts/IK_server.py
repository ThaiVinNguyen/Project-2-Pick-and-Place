#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import radians
from sympy import *

## Create symbols for DH param
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')     # Create joint angles theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')     # Create link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')     # Create link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # Create joint twist angles

## Create Modified DH parameters
DH = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
    alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
    alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
    alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:        q4,
    alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
    alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
    alpha6:      0, a6:      0, d7: 0.303, q7:         0}
## Function to return homogeneous transform matrix
def TF_Mat(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])
    return TF

# Create function to calculate theta1, theta2, theta3
def Calculate_theta_1_2_3(WC):
    global DH
    WCx, WCy, WCz = WC[0], WC[1], WC[2]
	
    # Calculate theta1
    theta1 = atan2(WCy,WCx)

    WCz_j2 = WCz - DH[d1]                           # WC z-component from j2
    WCx_j2 = sqrt(WCx**2 + WCy**2) - DH[a1]         # WC x-component from j2
    a = round(sqrt(DH[d4]**2 + DH[a3]**2), 7)       # line segment: j3-WC
    b = sqrt(WCx_j2**2 + WCz_j2**2)                 # line segment: j2-WC
    c = DH[a2]                                      # link length:  j2-j3

    A = acos((b**2 + c**2 - a**2) / (2*b*c))        # angle A
    B = acos((a**2 + c**2 - b**2) / (2*a*c))        # angle B
    C = acos((a**2 + b**2 - c**2) / (2*a*b))        # angle C

    # The sag between joint-3 and WC is due to a3 and its angle is formed
    # between y3-axis and side_a
    S = round(atan2(abs(DH[a3]),DH[d4]), 7) # angle Sag

    # Calculate theta2, theta3
    theta2 = pi/2 - A - atan2(WCz_j2, WCx_j2)
    theta3 = pi/2 - (B + S)

    return theta1, theta2, theta3

def Calculate_theta_4_5_6(T0_1,T1_2,T2_3,ROT_EE,theta1,theta2,theta3,q1,q2,q3):
    # Respective individual link Transforms
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]

    # Extract rotation matrix R0_3 from transformation matrix T0_3
    R0_3 = R0_1*R1_2*R2_3
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})
                        
    # Get rotation matrix R3_6 from (transpose of R0_3 * ROT_EE)
    R3_6 = R0_3.transpose() * ROT_EE    
    """
    R3_6 is the composite rotation matrix formed from an extrinsic:
    R3_6[1, 0]  # sin(theta5)*cos(theta6)
    R3_6[1, 1]  # -sin(theta6)*sin(theta6)
    R3_6[0, 2]  # -sin(theta5)*cos(theta4)
    R3_6[1, 2]  # cos(theta5)
    R3_6[2, 2]  # sin(theta4)*sin(theta5)
    """
    # Calculate theta5
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])

    # Calculate theta5, theta6 and select best solution.
    if (theta5 > pi) :
        theta4 = atan2(-R3_6[2,2], R3_6[0,2])
        theta6 = atan2(R3_6[1,1],-R3_6[1,0])
    else:
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1],R3_6[1,0])

    return theta4, theta5, theta6       
   
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"

        return -1
    else:

        ## Create individual transformation matrices
        T0_1 = TF_Mat(alpha0, a0, d1, q1).subs(DH)
        T1_2 = TF_Mat(alpha1, a1, d2, q2).subs(DH)
        T2_3 = TF_Mat(alpha2, a2, d3, q3).subs(DH)
        T3_4 = TF_Mat(alpha3, a3, d4, q4).subs(DH)
        T4_5 = TF_Mat(alpha4, a4, d5, q5).subs(DH)
        T5_6 = TF_Mat(alpha5, a5, d6, q6).subs(DH)
        T6_7 = TF_Mat(alpha6, a6, d7, q7).subs(DH)

        ## Composition of Homogeneous Transforms
        ## Transform from Base link_0 to End Effector (Gripper)
        T0_7 = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7) 

        # Correction Needed to Account for Orientation Difference Between
        # Definition of Gripper Link_G
        # Matrix is pre-calculated to improve performance.
        R_corr = Matrix([[0,0,1.0,0],[0,-1.0,0,0],[1.0,0,0,0],[0,0,0,1.0]])

        # Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
        # With orientation correction applied
        T0_7_corr = (T0_7 * R_corr)
        
        # Find EE rotation matrix RPY (Roll, Pitch, Yaw)
        r,p,y = symbols('r p y')

        # Roll
        ROT_x = Matrix([[       1,       0,       0],
                        [       0,  cos(r), -sin(r)],
                        [       0,  sin(r),  cos(r)]])
        # Pitch
        ROT_y = Matrix([[  cos(p),       0,  sin(p)],
                        [       0,       1,       0],
                        [ -sin(p),       0,  cos(p)]])
        # Yaw
        ROT_z = Matrix([[  cos(y), -sin(y),       0],
                        [  sin(y),  cos(y),       0],
                        [       0,       0,       1]])

        ROT_EE = ROT_z * ROT_y * ROT_x
        
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        # Correction Needed to Account for Orientation Difference Between
        # Definition of Gripper Link_G in URDF versus DH Convention
        ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
        ROT_EE = ROT_EE * ROT_corr   

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            # store EE position in a matrix
            EE = Matrix([[px],
                         [py],
                         [pz]])

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                                [req.poses[x].orientation.x, 
                                req.poses[x].orientation.y,
                                req.poses[x].orientation.z,
                                req.poses[x].orientation.w])

            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Calculate Wrest Center
            WC = EE - DH[d7] * ROT_EE[:,2]

            theta1, theta2, theta3 = Calculate_theta_1_2_3(WC)
            theta4, theta5, theta6 = Calculate_theta_4_5_6(T0_1,T1_2,T2_3,ROT_EE,theta1,theta2,theta3,q1,q2,q3)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
