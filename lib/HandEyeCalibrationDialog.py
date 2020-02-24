import os
from PyQt5.QtWidgets import QDialog, QFileDialog
from PyQt5.QtCore import pyqtSlot
from PyQt5 import uic

from grblCom import grblCom
from cn5X_config import *

import numpy as np
import cv2
import pickle
from scipy.spatial.transform import Rotation
from scipy.linalg import logm, sqrtm
import random
from itertools import combinations
import time

import roslibpy
import roslibpy.tf

self_dir = os.path.dirname(os.path.realpath(__file__))
SAVE_PATH = os.path.join(self_dir, 'data')

TIME_TO_WAIT = 1 # s
READY = 0
RECORDING = 1
MOTION = 2
COMPLETE = 3

class HandEyeCalibration:
    @staticmethod
    def pose_estimation(A,B):

        n=A.shape[2]
        T = np.zeros([9,9])
        X_est= np.eye(4)
        Y_est= np.eye(4)

        # Permutate A and B to get gross motions
        idx = np.random.permutation(n)
        A=A[:,:,idx]
        B=B[:,:,idx]

        for ii in range(n-1):
            Ra = A[0:3,0:3,ii]
            Rb = B[0:3,0:3,ii]
            T = T + np.kron(Rb,Ra)

        U, _, Vt=np.linalg.svd(T)
        xp=Vt.T[:,0]
        yp=U[:,0]
        X=np.reshape(xp, (3,3), order="F") # F: fortran/matlab reshape order
        Xn = (np.sign(np.linalg.det(X))/ np.abs(np.linalg.det(X))**(1/3))*X
        # re-orthogonalize to guarantee that they are indeed rotations.
        U_n, _, Vt_n=np.linalg.svd(Xn)
        X=np.matmul(U_n,Vt_n)

        Y=np.reshape(yp, (3,3), order="F") # F: fortran/matlab reshape order
        Yn = (np.sign(np.linalg.det(Y))/ np.abs(np.linalg.det(Y))**(1/3))*Y
        U_yn, _, Vt_yn=np.linalg.svd(Yn)
        Y=np.matmul(U_yn,Vt_yn)

        A_est = np.zeros([3*n,6])
        b_est = np.zeros([3*n,1])
        for ii in range(n-1):
            A_est[3*ii:3*ii+3,:] =np.concatenate((-A[0:3,0:3,ii], np.eye(3)), \
                axis=1)
            b_est[3*ii:3*ii+3,:] = np.transpose(A[0:3,3,ii] - \
                np.matmul(np.kron(B[0:3,3,ii].T,np.eye(3)), \
                np.reshape(Y, (9,1), order="F")).T)

        t_est_np=np.linalg.lstsq(A_est,b_est,rcond=None)
        if t_est_np[2]<A_est.shape[1]: # A_est.shape[1]=6
            print('Rank: ', t_est_np[2])
            print('Rank deficient')
        t_est=t_est_np[0]
        X_est[0:3,0:3]= X
        X_est[0:3,3]= t_est[0:3].T  
        Y_est[0:3,0:3]= Y
        Y_est[0:3,3]= t_est[3:6].T
        # verify Y_est using rigid_registration
        Y_est_check,ErrorStats= HandEyeCalibration.__rigid_registration(A,X_est,B)
        return X_est,Y_est, Y_est_check,ErrorStats

    @staticmethod
    def __rigid_registration(A,X,B):

        #nxnx4
        """solves for Y in YB=AX
        A: (4x4xn)
        B: (4x4xn)
        X= (4x4)
        Y= (4x4)
        n number of measurements
        ErrorStats: Registration error (mean,std)
        """
        n=A.shape[2]
        AX=np.zeros(A.shape)
        AXp=np.zeros(A.shape)
        Bp=np.zeros(B.shape)
        pAX=np.zeros(B[0:3,3,:].shape) # To calculate reg error
        pYB=np.zeros(B[0:3,3,:].shape) # To calculate reg error
        Y_est=np.eye(4)

        ErrorStats=np.zeros((2,1))

        for ii in range(n):
           AX[:,:,ii]=np.matmul(A[:,:,ii],X)

        # Centroid of transformations t and that
        t=1/n*np.sum(AX[0:3,3,:],1)
        that=1/n*np.sum(B[0:3,3,:],1)
        AXp[0:3,3,:]=AX[0:3,3,:]-np.tile(t[:,np.newaxis], (1, n))
        Bp[0:3,3,:]=B[0:3,3,:]-np.tile(that[:,np.newaxis], (1, n))

        [i,j,k]=AX.shape # 4x4xn
        # Convert AX and B to 2D arrays
        AXp_2D=AXp.reshape((i,j*k)) # now it is 4x(4xn)
        Bp_2D=Bp.reshape((i,j*k)) # 4x(4xn)
        # Calculates the best rotation
        U, _, Vt=np.linalg.svd(np.matmul(Bp_2D[0:3,:],AXp_2D[0:3,:].T)) # v is v' in matlab
        R_est = np.matmul(Vt.T, U.T)
        # special reflection case
        if np.linalg.det(R_est) < 0:
            print ('Warning: Y_est returned a reflection')
            R_est =np.matmul( Vt.T, np.matmul(np.diag([1,1,-1]),U.T))
        # Calculates the best transformation
        t_est = t-np.dot(R_est,that)
        Y_est[0:3,0:3]=R_est
        Y_est[0:3,3]=t_est
        # Calculate registration error
        pYB=np.matmul(R_est,B[0:3,3,:])+np.tile(t_est[:,np.newaxis],(1,n)) # 3xn
        pAX=AX[0:3,3,:]
        Reg_error=np.linalg.norm(pAX-pYB,axis=0) # 1xn
        ErrorStats[0]=np.mean(Reg_error)
        ErrorStats[1]=np.std(Reg_error)
        return Y_est, ErrorStats

    @staticmethod
    def convertToSE3(quat, tvec):
        """
        Converts quaternion and translation vector into SE3 lie group

        Args:
            quat: quaternion representation of rotation (scalar last)
            tvec: translation vector (meters)

        Returns:
            tf: SE3 transformation (rotation + translation)
        """
        r = Rotation.from_quat(quat)
        r_mat = r.as_matrix()
        tvec = tvec.reshape((3,1))
        tf = np.concatenate((r_mat, tvec), axis=1)
        last_row = [[0, 0, 0, 1]]
        tf = np.concatenate((tf, last_row), axis=0)
        return tf

    @staticmethod
    def format(tfs):
        """
        Formats sensor data to be sent to batch processing
        [ [ [ r11_1 r11_2 r11_3 ... r11_n ] [ r12_1 ... r12_n ] ... [ r14_1 ... r14_n ] ] ... [ [ r41_1 ...r41_n ] ... [ r44_1 ... r44_n ] ] ]

        Args:
            tfs: list of transformations

        Returns:
            A: correctly formatted data
        """
        A = []
        for i in range(4):
            sub_list = []
            for j in range(4):
                subsub_list = []
                for tf in tfs:
                    subsub_list.append(tf[i][j])
                sub_list.append(subsub_list)
            A.append(sub_list)
        return A


class HandEyeCalibrationDialog(QDialog):

    def __init__(self, grbl: grblCom, ros):
        super().__init__()

        self.ros = ros
        self.ros.on_ready(lambda: print('HandEye (ROS Connection):', self.ros.is_connected))

        self.__grblCom = grbl
        self.__grblCom.sig_status.connect(self.on_sig_status)
        
        ui_dlgHandEyeCalibration = os.path.join(self_dir, '../ui/handeye.ui')
        self.__di = uic.loadUi(ui_dlgHandEyeCalibration, self)

        self.__di.pushButton_collect.clicked.connect(self.recordData)
        self.__di.pushButton_undo.pressed.connect(self.deleteLastData)
        self.__di.pushButton_complete.pressed.connect(self.calibrate)
        self.__di.pushButton_auto_start.pressed.connect(self.automaticCalibration)
        self.__di.pushButton_save.pressed.connect(self.saveCalibration)

        self.__di.lineEdit_frame_camera.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_robot_W.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_robot_T.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_marker_W.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_marker_T.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_base_link.textChanged.connect(self.updateListener)

        self.updateListener()

        self.robot_poses = [] # list of [tx, ty, tz, qx, qy, qz, qw]
        self.marker_W_poses = []
        self.marker_T_poses = []
        
        # subscriber for aruco marker
        self.aruco_sub = roslibpy.Topic(self.ros, "/markersAruco", 'marker_msgs/MarkerDetection')
        self.aruco_sub.subscribe(self.updateMarkerDetectionNum)
        self.state = READY
        self.timestamp_action_complete = time.time()
        
        self.show()


    def updateListener(self):
        # get frame names from GUI, create two listener that subscribe to updated frames
        camera_frame = self.__di.lineEdit_frame_camera.text()
        marker_W_frame = self.__di.lineEdit_frame_marker_W.text()
        marker_T_frame = self.__di.lineEdit_frame_marker_T.text()
        W_frame = self.__di.lineEdit_frame_robot_W.text()
        T_frame = self.__di.lineEdit_frame_robot_T.text()
        base_link = self.__di.lineEdit_frame_base_link.text()

        # self.tf_listener_robot = roslibpy.tf.TFClient(self.ros, robot_fixed_frame, angular_threshold=0.001, translation_threshold=0.0001)
        # self.tf_listener_camera = roslibpy.tf.TFClient(self.ros, camera_frame, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_robot = roslibpy.tf.TFClient(self.ros, base_link, rate=100, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_camera = roslibpy.tf.TFClient(self.ros, camera_frame, rate=100, angular_threshold=0.001, translation_threshold=0.0001)

        self.tf_listener_robot.subscribe(W_frame, self.updateRobotPose)
        self.tf_listener_camera.subscribe(marker_W_frame,self.updateMarkerW)

    def updateMarkerDetectionNum(self, data):
        self.num_markers = len(data['markers'])

    def updateRobotPose(self, data):
        # update robot realtime pose
        self.robot_rt_pose = data

    def updateMarkerW(self, data):
        self.marker_W_pose = data

    def updateMarkerT(self, data):
        self.marker_T_pose = data

    def recordData(self):
        # record each pose as [tx, ty, tz, qx, qy, qz, qw]
        robot_quat = [self.robot_rt_pose['rotation']['x'], self.robot_rt_pose['rotation']['y'], self.robot_rt_pose['rotation']['z'], self.robot_rt_pose['rotation']['w']]
        robot_tvec = [self.robot_rt_pose['translation']['x'], self.robot_rt_pose['translation']['y'], self.robot_rt_pose['translation']['z']]
        robot_rt_pose = []
        robot_rt_pose.extend(robot_tvec)
        robot_rt_pose.extend(robot_quat)
        marker_W_quat = [self.marker_W_pose['rotation']['x'], self.marker_W_pose['rotation']['y'], self.marker_W_pose['rotation']['z'], self.marker_W_pose['rotation']['w']]
        marker_W_tvec = [self.marker_W_pose['translation']['x'], self.marker_W_pose['translation']['y'], self.marker_W_pose['translation']['z']]
        marker_W_pose = []
        marker_W_pose.extend(marker_W_tvec)
        marker_W_pose.extend(marker_W_quat)
        # marker_T_quat = [self.marker_T_pose['rotation']['x'], self.marker_T_pose['rotation']['y'], self.marker_T_pose['rotation']['z'], self.marker_T_pose['rotation']['w']]
        # marker_T_tvec = [self.marker_T_pose['translation']['x'], self.marker_T_pose['translation']['y'], self.marker_T_pose['translation']['z']]
        # marker_T_pose = []
        # marker_T_pose.extend(marker_T_tvec)
        # marker_T_pose.extend(marker_T_quat)

        print("robot")
        print(robot_rt_pose)
        print("marker_W")
        print(marker_W_pose)
        # print("marker_T")
        # print(marker_T_pose)

        self.robot_poses.append(robot_rt_pose)
        self.marker_W_poses.append(marker_W_pose)
        # self.marker_T_poses.append(marker_T_pose)
        self.__di.label_pose_count.setNum(len(self.robot_poses))

    def automaticCalibration(self):
        # Generate random robot motions by some joint values
        num_poses = self.__di.spinBox_pose_count.value()
        x_vals = np.linspace(-170, 0, num=num_poses)
        y_vals =  np.linspace(-130, 0, num=num_poses)
        z_vals = np.linspace(-35, 0, num=num_poses)
        a_vals = np.linspace(-90, -60, num=num_poses)
        b_vals = np.flip(np.linspace(-360, 0, num=num_poses, endpoint=False))
        random.shuffle(x_vals)
        random.shuffle(y_vals)
        random.shuffle(z_vals)
        random.shuffle(a_vals)
        # self.motion_joints = np.transpose( np.stack((x_vals, y_vals, z_vals, a_vals, b_vals),axis=0) )
        self.motion_joints = np.transpose( np.stack((z_vals, a_vals, b_vals),axis=0) )
        self.state = RECORDING
        self.i_cmd = 0 # motion command counter

    @pyqtSlot(str)
    def on_sig_status(self, buff: str):
        # State-Machine: READY -> RECORDING -> MOTION -> COMPLETE
        self.__grblStatus = buff[1:].split('|')[0]
        if self.__grblStatus == GRBL_STATUS_IDLE and self.state is not COMPLETE:
            delt_t = time.time() - self.timestamp_action_complete
            if self.state is RECORDING:
                if delt_t > TIME_TO_WAIT:
                    if self.num_markers > 0:
                        self.recordData()
                    self.state = MOTION # change state to motion
                    self.timestamp_action_complete = time.time() # update timestamp
                    if self.i_cmd == self.__di.spinBox_pose_count.value():
                        self.state = COMPLETE # change state to calibration complete
                        self.timestamp_action_complete = time.time() # update timestamp
                        self.calibrate()
            elif self.state is MOTION:
                if delt_t > TIME_TO_WAIT:
                    # self.__grblCom.gcodePush("G0 X{} Y{} Z{} A{} B{}".format(*self.motion_joints[self.i_cmd]),COM_FLAG_NO_OK)
                    self.__grblCom.gcodePush("G0 Z{} A{} B{}".format(*self.motion_joints[self.i_cmd]),COM_FLAG_NO_OK)
                    self.i_cmd = self.i_cmd + 1
                    self.state = RECORDING # change state to recording
                    self.timestamp_action_complete = time.time() # update timestamp
            else:
                if delt_t > TIME_TO_WAIT:
                    self.timestamp_action_complete = time.time() # update timestamp
                    pass

    # TODO
    def deleteLastData(self):
        if len(self.robot_poses) > 0:
            self.robot_poses = self.robot_poses[:-1]
            # self.camera_poses = self.camera_poses[:-1]
            self.marker_W_poses = self.marker_W_poses[:-1]
            self.marker_T_poses = self.marker_T_poses[:-1]
            num = len(self.robot_poses)
            self.__di.label_pose_count.setText(str(num))


    def calibrate(self):
        if len(self.robot_poses) < 4:
            print("Please record at least 4 points.")
        else:
            print('Number of data points: ', len(self.robot_poses))

            # lists homogenous transformations
            list_H_cam_tag = []
            list_H_base_W = []
            for i in range(len(self.robot_poses)):
                H_cam_tag = HandEyeCalibration.convertToSE3(np.asarray(self.marker_W_poses[i][3:]), np.asarray(self.marker_W_poses[i][0:3]))
                list_H_cam_tag.append(np.asarray(H_cam_tag))
                H_base_W = HandEyeCalibration.convertToSE3(np.asarray(self.robot_poses[i][3:]), np.asarray(self.robot_poses[i][0:3]))
                list_H_base_W.append(np.asarray(H_base_W))

            # construct A, B matrix
            A, B = self.constructMatAB(list_H_base_W, list_H_cam_tag)
            # solve for AX=XB
            X = self.solveAXeqXB(A, B)

            Rx = Rotation.from_matrix(X[:3,:3])
            print("euler angles")
            print(Rx.as_euler('zyx', degrees=True))
            print("quaternion")
            print(Rx.as_quat())
            print("Tx")
            print(X[:3,3])

            # A = np.array(HandEyeCalibration.format(list_H_base_W))
            # B = np.array(HandEyeCalibration.format(list_H_cam_tag))
            # self.X_est,Y_est,_,_ = HandEyeCalibration.pose_estimation(A,B)
            # print("Estimated X: ", self.X_est)
            # print("Estimated Y: ", Y_est)

            # # Accuracy Evaluation
            # rot_acc = 0
            # trans_acc = 0
            # list_H_base_W = np.asarray(list_H_base_W)
            # list_H_tag_cam = np.asarray(list_H_tag_cam)
            # for i in range(len(list_H_base_W)):
            #     row_norm = np.linalg.norm(np.matmul(list_H_base_W[i,0:3,0:3], self.X_est[0:3,0:3]) - np.matmul(Y_est[0:3,0:3], list_H_tag_cam[i,0:3,0:3]), axis=1)
            #     matrix_norm = np.linalg.norm(row_norm)**2
            #     rot_acc = rot_acc + (1 - matrix_norm/8) / len(list_H_base_W)

            #     vec1 = np.matmul(list_H_base_W[i,0:3,0:3], self.X_est[0:3,3]) + list_H_base_W[i,0:3,3]
            #     vec1 = vec1 / np.linalg.norm(vec1)
            #     vec2 = np.matmul(Y_est[0:3,0:3], list_H_tag_cam[i,0:3,3]) + Y_est[0:3,3]
            #     vec2 = vec2 / np.linalg.norm(vec2)
            #     scalar = np.matmul(vec1.T, vec2)
            #     trans_acc = trans_acc + abs(scalar) / len(list_H_base_W)
            # print("Rotational accuracy: ", rot_acc)
            # print("Translational accuracy: ", trans_acc)
        

    def saveCalibration(self):
        # create save folder if not existing yet
        if not os.path.isdir(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        file_name, _ = QFileDialog.getSaveFileName(self, 'Save File', directory=SAVE_PATH, filter="Pickle Files (*.p)")
        if file_name != '':
            file_name = str(file_name)
            file_name_yml = os.path.splitext(file_name)[0] + '.yml'
            self.saveYML(file_name_yml)
            dbfile = open(file_name, 'wb')
            pickle.dump(self.robot_poses, dbfile)
            pickle.dump(self.marker_W_poses, dbfile)
            # pickle.dump(self.marker_T_poses, dbfile)
            print("data saved!")
            dbfile.close()
    
    def saveYML(self, file_name):

        s = cv2.FileStorage(file_name, cv2.FileStorage_WRITE)
        s.write("frameCount", len(self.robot_poses))
        for i in range(len(self.robot_poses)):
            marker_W_tf = HandEyeCalibration.convertToSE3(np.asarray(self.marker_W_poses[i][3:]), np.asarray(self.marker_W_poses[i][0:3])*1000)
            marker_T_tf = HandEyeCalibration.convertToSE3(np.asarray(self.marker_T_poses[i][3:]), np.asarray(self.marker_T_poses[i][0:3])*1000)
            # camera_tf = np.eye(4)
            camera_tf = np.matmul(np.linalg.inv(marker_T_tf), marker_W_tf)
            robot_tf = HandEyeCalibration.convertToSE3(np.asarray(self.robot_poses[i][3:]), np.asarray(self.robot_poses[i][0:3])*1000)
            frame_1 = "T1_"+str(i)
            frame_2 = "T2_"+str(i)
            s.write(frame_1, robot_tf)
            s.write(frame_2, camera_tf)
        s.release()

    def constructMatAB(self, list_H_a, list_H_b):
        """construct A or B matrix of AX=XB from a list of transformations"""
        A = []
        B = []
        list_comb_a = list(combinations(list_H_a, 2))
        list_comb_b = list(combinations(list_H_b, 2))
        for H in list_comb_a:
            A.append( np.matmul(H[0], np.linalg.inv(H[1])) )
        for H in list_comb_b:
            B.append( np.matmul(H[0], np.linalg.inv(H[1])) )
        return A, B
        
    def solveAXeqXB(self, A, B):
        """Solve AX=XB"""
        # convert A, B rotation part into axis-angle representation
        alpha = self.convertToAxisAngle(A)
        beta = self.convertToAxisAngle(B)

        # First construct matrix M
        M = np.zeros(3)
        for i in range(len(alpha)):
            M = M + np.matmul(beta[i].reshape((3,1)), alpha[i].reshape((1,3)))
        # Compute the rotation matrix
        Rx = np.matmul( sqrtm( np.linalg.inv( np.matmul(M.T, M) ) ), M.T)

        # Once Rx is found, solve for Tx
        # (I-Ra) * Tx = (t_A - Rx * t_B)
        Q = np.empty((0,3))
        b = np.empty((0))
        for i in range(len(A)):
            Q = np.concatenate((Q, A[i][:3,:3] - np.eye(3)), axis=0)
            b = np.concatenate((b, np.matmul(Rx, B[i][:3,3]) - A[i][:3,3]), axis=0)

        # Compute the translation Q * Tx = b using least squres
        Tx = np.linalg.lstsq(np.asarray(Q), np.asarray(b))

        # Homogeneous
        upper = np.concatenate((Rx,Tx[0].reshape((3,1))),axis=1)
        X = np.concatenate((upper, np.array([[0,0,0,1]])),axis=0)

        return X

    def convertToAxisAngle(self, A_s):
        alpha_s = []
        for A in A_s:
            # slice out the rotation part, convert
            alpha_s.append(self.dhat(logm(A[:3,:3])))
        return alpha_s 

    def dhat(self, W):
        v = np.array( [W[2,1], W[0,2], W[1,0]] )
        return v