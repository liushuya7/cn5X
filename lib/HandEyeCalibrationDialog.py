import os
from PyQt5.QtWidgets import QDialog
from PyQt5 import uic

from grblCom import grblCom
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from robot_kins import CNC_5dof
import random

import roslibpy
import roslibpy.tf

self_dir = os.path.dirname(os.path.realpath(__file__))

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
            Y = A[0:3,0:3,ii] * Rx * B[0:3,0:3,ii].T
            A_est[3*ii:3*ii+3,:] =np.concatenate((-A[0:3,0:3,ii], np.eye(3)), \
                axis=1)
            b_est[3*ii:3*ii+3,:] = np.transpose(A[0:3,3,ii] - \
                np.matmul(np.kron(B[0:3,3,ii].T,np.eye(3)), \
                np.reshape(Y, (9,1), order="F")).T)

        t_est_np=np.linalg.lstsq(A_est,b_est,rcond=None)
        if t_est_np[2]<A_est.shape[1]: # A_est.shape[1]=6
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


class HandEyeCalibrationDialog(QDialog):

    def __init__(self, grbl: grblCom):
        super().__init__()

        self.__grblCom = grbl
        self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros.on_ready(lambda: print('ROS Connection (Handeye):', self.ros.is_connected))

        ui_dlgHandEyeCalibration = os.path.join(self_dir, '../ui/handeye.ui')
        self.__di = uic.loadUi(ui_dlgHandEyeCalibration, self)

        self.__di.pushButton_collect.clicked.connect(self.record_data)
        self.__di.pushButton_undo.pressed.connect(self.delete_last_data)
        self.__di.pushButton_complete.pressed.connect(self.calibrate)
        self.__di.pushButton_auto_start.pressed.connect(self.automaticCalibration)
        self.__di.lineEdit_frame_camera.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_object.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_robot_base.textChanged.connect(self.updateListener)
        self.__di.lineEdit_frame_robot_hand.textChanged.connect(self.updateListener)

        self.robot = CNC_5dof()
        self.updateListener() 
        self.robot_pose = None
        self.object_pose = None
        self.robot_poses = [] # list of [tx, ty, tz, qx, qy, qz, qw]
        # self.object_poses = [] # list of [tx, ty, tz, qx, qy, qz, qw]
        self.marker1_poses = []
        self.marker2_poses = []

        self.show()


    def updateListener(self):
        # get frame names from GUI, create two listener that subscribe to updated frames
        camera_fixed_frame = self.__di.lineEdit_frame_camera.text()
        robot_fixed_frame = self.__di.lineEdit_frame_robot_base.text()
        # checkerboard_frame = self.__di.lineEdit_frame_object.text()
        tool_frame = self.__di.lineEdit_frame_tool.text()
        marker1_frame = self.__di_lineEdit_frame_marker1.text()
        marker2_frame = self.__di_lineEdit_frame_marker2.text()
        end_effector_frame = self.__di.lineEdit_frame_robot_hand.text()

        # self.tf_listener_robot = roslibpy.tf.TFClient(self.ros, robot_fixed_frame, angular_threshold=0.001, translation_threshold=0.0001)
        # self.tf_listener_camera = roslibpy.tf.TFClient(self.ros, camera_fixed_frame, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_robot = roslibpy.tf.TFClient(self.ros, tool_frame, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_marker1 = roslibpy.tf.TFClient(self.ros, camera_fixed_frame, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_marker2 = roslibpy.tf.TFClient(self.ros, camera_fixed_frame, angular_threshold=0.001, translation_threshold=0.0001)

        # self.tf_listener_camera.subscribe(checkerboard_frame,self.update_camera_detection)
        self.tf_listener_robot.subscribe(end_effector_frame, self.update_robot_pose)
        self.tf_listener_marker1.subscribe(marker1_frame,self.update_marker1_detection)
        self.tf_listener_marker2.subscribe(marker2_frame,self.update_marker2_detection)


    def record_data(self):
        # record each pose as [tx, ty, tz, qx, qy, qz, qw]
        robot_quat = [self.robot_pose['rotation']['x'], self.robot_pose['rotation']['y'], self.robot_pose['rotation']['z'], self.robot_pose['rotation']['w']]
        robot_tvec = [self.robot_pose['translation']['x'], self.robot_pose['translation']['y'], self.robot_pose['translation']['z']]
        robot_pose = []
        robot_pose.extend(robot_tvec)
        robot_pose.extend(robot_quat)
        # object_quat = [self.object_pose['rotation']['x'], self.object_pose['rotation']['y'], self.object_pose['rotation']['z'], self.object_pose['rotation']['w']]
        # object_tvec = [self.object_pose['translation']['x'], self.object_pose['translation']['y'], self.object_pose['translation']['z']]
        # object_pose = []
        # object_pose.extend(object_tvec)
        # object_pose.extend(object_quat)
        marker1_quat = [self.marker1_pose['rotation']['x'], self.marker1_pose['rotation']['y'], self.marker1_pose['rotation']['z'], self.marker1_pose['rotation']['w']]
        marker1_tvec = [self.marker1_pose['translation']['x'], self.marker1_pose['translation']['y'], self.marker1_pose['translation']['z']]
        marker1_pose = []
        marker1_pose.extend(marker1_tvec)
        marker1_pose.extend(marker1_quat)
        marker2_quat = [self.marker2_pose['rotation']['x'], self.marker2_pose['rotation']['y'], self.marker2_pose['rotation']['z'], self.marker2_pose['rotation']['w']]
        marker2_tvec = [self.marker2_pose['translation']['x'], self.marker2_pose['translation']['y'], self.marker2_pose['translation']['z']]
        marker2_pose = []
        marker2_pose.extend(marker2_tvec)
        marker2_pose.extend(marker2_quat)

        print("robot")
        print(robot_pose)
        # print("checkerboard")
        # print(object_pose)
        print("marker1")
        print(marker1_pose)
        print("marker2")
        print(marker2_pose)
        self.robot_poses.append(robot_pose)
        # self.object_poses.append(object_pose)
        self.marker1_poses.append(marker1_pose)
        self.marker2_poses.append(marker2_pose)

        self.label_pose_count.setNum(len(self.robot_poses))


    def update_robot_pose(self, data):
        self.robot_pose = data


    def update_camera_detection(self, data):
        self.object_pose = data


    def update_marker1_detection(self, data):
        self.marker1_pose = data


    def update_marker2_detection(self, data):
        self.marker2_pose = data


    def automaticCalibration(self):
        num_poses = self.__di.spinBox_pose_count.value()
        x_vals = np.linspace(-100, 0, num=num_poses)
        y_vals =  np.linspace(-100, 0, num=num_poses)
        z_vals = np.linspace(-10, 0, num=num_poses)
        a_vals = np.linspace(-110, -60, num=num_poses)
        b_vals = np.linspace(-360, 0, num=num_poses, endpoint=False)
        random.shuffle(x_vals)
        random.shuffle(y_vals)
        random.shuffle(z_vals)
        random.shuffle(a_vals)
        random.shuffle(b_vals)
        num_poses = len(z_vals)

        for i in range(num_poses):
            self.ready_to_record = True
            self.__grblCom.gcodePush("G0 X{} Y{} Z{} A{} B{}".format(x_vals[i], y_vals[i], z_vals[i], a_vals[i], b_vals[i]))
            time.sleep(2) # wait till robot reaches position
            self.record_data()

        self.calibrate()


    def delete_last_data(self):
        # TODO: edit function later
        if len(self.robot_poses) > 0:
            self.robot_poses = self.robot_poses[:-1]
            # self.camera_poses = self.camera_poses[:-1]
            self.marker1_poses = self.marker1_poses[:-1]
            self.marker2_poses = self.marker2_poses[:-1]
            num = len(self.robot_poses)
            self.__di.label_pose_count.setText(str(num))


    def calibrate(self):
        if len(self.robot_poses) < 4:
            print("Please record at least 4 points.")
        else:
            robot_tfs = []
            camera_tfs = []
            for i in range(len(self.robot_poses)):
                robot_tf = HandEyeCalibrationDialog.convertToSE3(np.asarray(self.robot_poses[i][3:]), np.asarray(self.robot_poses[i][0:3]))
                robot_tfs.append(np.asarray(robot_tf))
                # camera_tf = HandEyeCalibrationDialog.convertToSE3(np.asarray(self.object_poses[i][3:]), np.asarray(self.object_poses[i][0:3]))
                # camera_tfs.append(np.asarray(camera_tf))
                marker1_tf = HandEyeCalibrationDialog.convertToSE3(np.asarray(self.marker1_poses[i][3:]), np.asarray(self.marker1_poses[i][0:3]))
                marker2_tf = HandEyeCalibrationDialog.convertToSE3(np.asarray(self.marker2_poses[i][3:]), np.asarray(self.marker2_poses[i][0:3]))
                camera_tf = np.zeros((4,4))
                camera_tf[3,3] = 1
                camera_tf[0:3,0:3] = marker2[0:3,0:3] * marker1[0:3,0:3].T
                camera_tf[0:3,3] = marker2[0:3,0:3] * -marker1[0:3,0:3].T * marker1[0:3,3] + marker2[0:3,3]
                camera_tfs.append(np.asarray(camera_tf))

            A = np.array(HandEyeCalibrationDialog.format(robot_tfs))
            B = np.array(HandEyeCalibrationDialog.format(camera_tfs))
            self.X_est,Y_est,_,_ = HandEyeCalibration.pose_estimation(A,B)
            print(self.X_est)

            # Accuracy Evaluation
            rot_acc = 0
            trans_acc = 0
            for i in range(len(robot_tfs)):
                row_norm = np.linalg.norm(robot_tfs[0:3,0:3,i] * X_est[0:3,0:3] - Y_est[0:3,0:3] * camera_tfs[0:3,0:3,i], axis=1)
                matrix_norm = np.linalg.norm(row_norm)**2
                rot_acc = rot_acc + (1 - matrix_norm/8) / len(robot_tfs)

                vec1 = robot_tfs[0:3,0:3,i] * X_est[0:3,3] + robot_tfs[0:3,3,i]
                vec1 = vec1 / np.linalg.norm(vec1)
                vec2 = Y_est[0:3,0:3] * camera_tfs[0:3,3,i] + Y_est[0:3,3]
                vec2 = vec2 / np.linalg.norm(vec2)
                scalar = vec1.T * vec2
                trans_acc = abs(scalar) / len(robot_tfs)
            print(rot_acc)
            print(trans_acc)
        

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
        r = R.from_quat(quat)
        r_mat = r.as_dcm()
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
