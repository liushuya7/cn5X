#!/usr/bin/env python  
import roslib
roslib.load_manifest('verify_tf')
import rospy

import numpy as np
import math

import tf
from sensor_msgs.msg import JointState

def broadcast_fwd_kin_pose(msg):
    Lx = 0.130
    Ly = -0.165
    Lz = 0.211
    l = 0.120

    for i in range(len(msg.name)):
        if msg.name[i] == "rotary_base__rotary_middle":
            theta4 = -msg.position[i]
        elif msg.name[i] == "rotary_middle__rotary_top":
            theta5 = msg.position[i]
        elif msg.name[i] == "x_box__tool":
            d1 = abs(msg.position[i])
        elif msg.name[i] == "y_box1__x_box":
            d2 = abs(msg.position[i])
        elif msg.name[i] == "base_link__y_box1":
            d3 = abs(msg.position[i])

    tf_L_T = np.array( [ [1,0,0,-d1], [0,1,0,d2], [0,0, 1,-d3], [0,0,0,1]] ).astype(np.float)
    tf_O_L = np.array( [ [1,0,0,Lx],
                         [0,1,0,Ly],
                         [0,0,1,Lz],
                         [0,0,0,1]] ).astype(np.float)

    tf_O_U = np.array( [ [1,0,0,0],
                       [0,np.cos(math.pi/2 - theta4), -np.sin(math.pi/2 - theta4), -l*np.cos(theta4)],
                       [0,np.sin(math.pi/2 - theta4), np.cos(math.pi/2 - theta4), l*np.sin(theta4)],
                       [0,0,0,1]] ).astype(np.float)
    tf_U_W = np.array( [ [np.cos(theta5), -np.sin(theta5), 0, 0],
                       [np.sin(theta5), np.cos(theta5), 0, 0],
                       [0,0,1,0],
                       [0,0,0,1]] ).astype(np.float)

    tf_O_W = np.matmul(tf_O_U, tf_U_W)
    tf_O_T = np.matmul(tf_O_L, tf_L_T)

    tf_W_T = np.matmul(np.linalg.inv(tf_O_W), tf_O_T)

    br = tf.TransformBroadcaster()
    # br.sendTransform((tf_W_T[0,3], tf_W_T[1,3], tf_W_T[2,3]),
    #                  tf.transformations.quaternion_from_matrix(tf_W_T),
    #                  rospy.Time.now(),
    #                  "tool_tip",
    #                  "rotary_top")
    br.sendTransform((tf_W_T[0,3], tf_W_T[1,3], tf_W_T[2,3]),
                     tf.transformations.quaternion_from_matrix(tf_W_T),
                     rospy.Time.now(),
                     "tool_tip",
                     "rotary_top")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('joint_states', JointState, broadcast_fwd_kin_pose)
    rospy.spin()