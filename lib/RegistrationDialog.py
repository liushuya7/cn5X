import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QSpinBox, QDoubleSpinBox, QLineEdit, QTableWidgetItem, QFileDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic
from cn5X_config import *
from grblCom import grblCom
from msgbox import *
from qweditmask import qwEditMask

# Registration
from Registration import Registration
import vtk
import numpy as np
import csv
# from robot_kins import CNC_5dof
from MeshProcessing import MeshProcessing
from scipy.spatial.transform import Rotation

import roslibpy
import roslibpy.tf

from compilOptions import grblCompilOptions

self_dir = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(self_dir, "../data")
DATA_FILTER = "CSV files (*.csv);;Text files (*.txt)"
RED = (255,0,0)
GREEN = (0,255,0)

class RegistrationDialog(QDialog):
    ''' Registration dialog '''

    def __init__(self, grbl: grblCom, ros, viewer=None):
        super().__init__()

        self.ros = ros
        self.ros.on_ready(lambda: print('Registration (ROS Connection):', self.ros.is_connected))

        ui_dlgRegistration = os.path.join(self_dir, '../ui/registration.ui')
        self.__di = uic.loadUi(ui_dlgRegistration, self)
        self.transformation_matrix = None
         # change table header names
        self.table_healder = ['x','y','z']
        self.tableWidget_model.setHorizontalHeaderLabels(self.table_healder)
        self.tableWidget_world.setHorizontalHeaderLabels(self.table_healder)

        self.is_loaded = True
        self.viewer = viewer

        self.__grblCom = grbl
        self.__grblCom.sig_alarm.connect(self.on_sig_alarm)
        self.__grblCom.sig_ok.connect(self.on_sig_ok)
        self.__grblCom.sig_status.connect(self.on_sig_status)
        self.is_probing = False

        self.__di.pushButton_collect.clicked.connect(self.on_sig_probe)
        self.__di.pushButton_delete_selected_model_points.pressed.connect(self.deleteSelectedModelPoint)
        self.__di.pushButton_delete_selected_world_points.pressed.connect(self.deleteSelectedWorldPoint)
        self.__di.pushButton_undo.pressed.connect(self.undoModel)
        self.__di.pushButton_extract_features.pressed.connect(self.extractFeatures)
        self.__di.tableWidget_model.currentItemChanged.connect(self.magnifyItem)
        self.__di.pushButton_load_world_features.pressed.connect(self.loadWorldFeatures)
        self.__di.pushButton_register_landmark.pressed.connect(self.registerLandmarks)
        self.__di.pushButton_register_icp.pressed.connect(self.registerICP)
        self.__di.pushButton_register_hungarian.pressed.connect(self.registerHungarian)
        self.__di.pushButton_load_data.pressed.connect(self.loadData)

        # self.robot = CNC_5dof()
        self.x = 0
        self.y = 0
        self.z = 0
        self.joints = np.zeros((5,1))

        self.tf_listener_robot = roslibpy.tf.TFClient(self.ros, "/W", rate=100, angular_threshold=0.001, translation_threshold=0.0001)
        self.tf_listener_robot.subscribe("/T", self.updateRobotPose)

        # mesh processing
        if self.viewer.current_mesh_file:
            self.mesh_processor = MeshProcessing(self.viewer.current_mesh_file)
        else:
            self.mesh_processor = None

        self.finished.connect(self.closeWindow)
        self.show()

    def updateRobotPose(self, data):
        # update robot realtime pose
        self.robot_rt_pose = data

    def undoModel(self):

        # delete point in table
        rowPosition = self.tableWidget_model.rowCount()
        self.tableWidget_model.removeRow(rowPosition - 1)

        # delete point in renderer
        self.viewer.deleteLatestPoint()

    def magnifyItem(self, current, previous):
        self.viewer.magnifyPoint(current, previous)
        self.tableWidget_model.setCurrentItem(current)

    def deleteSelectedTablePoint(self, table):
        current_row = table.currentRow()
        table.removeRow(current_row)
        return current_row

    def deleteSelectedWorldPoint(self):
        current_row = self.deleteSelectedTablePoint(self.tableWidget_world)

    def deleteSelectedModelPoint(self):
        current_row = self.deleteSelectedTablePoint(self.tableWidget_model)
        self.viewer.deleteSelectedPoint(current_row)
    
    def extractFeatures(self):
        if not self.mesh_processor:
            if not self.viewer.current_mesh_file:
                print("Error: No mesh file in the viewer yet!")
            else:
                self.mesh_processor = MeshProcessing(self.viewer.current_mesh_file)
        if self.mesh_processor:
            feature_points = self.mesh_processor.extractFeaturePoints()
            # render feature points in viewer
            for point in feature_points:
                actor = self.viewer.createPointActor(point, color=RED)
                self.viewer.addActor(actor)
                self.viewer.addPointToTable(point)

    def loadWorldFeatures(self):
        file_name, _ = QFileDialog.getOpenFileName(self, 'Open File', directory=DATA_PATH,filter=DATA_FILTER)
        if file_name != '' and os.path.splitext(file_name)[1] in DATA_FILTER:
            with open(file_name, 'r') as stream:
                reader = csv.reader(stream, delimiter=',')
                for row in reader:
                    rowPosition = self.tableWidget_world.rowCount()
                    self.tableWidget_world.insertRow(rowPosition)
                    self.tableWidget_world.setItem(rowPosition, 0, QTableWidgetItem(row[0]))
                    self.tableWidget_world.setItem(rowPosition, 1, QTableWidgetItem(row[1]))
                    self.tableWidget_world.setItem(rowPosition, 2, QTableWidgetItem(row[2]))

    def initializeRegistration(self, set_same_length=False):
        # get source points and target points from tablewidget_world and tablewidget_model 
        source_data = []
        for row in range(self.tableWidget_world.rowCount()):
            source_data.append([])
            for col in range(self.tableWidget_world.columnCount()):
                    source_data[row].append(float(self.tableWidget_world.item(row, col).text()))

        target_data = []
        for row in range(self.tableWidget_model.rowCount()):
            target_data.append([])
            for col in range(self.tableWidget_model.columnCount()):
                    target_data[row].append(float(self.tableWidget_model.item(row, col).text()))

        if set_same_length:
            # take correspondence source and target data
            num_valid_data = min(len(source_data), len(target_data))
            source_data = source_data[:num_valid_data]
            target_data = target_data[:num_valid_data]

        registration = Registration(source_data, target_data)

        return registration

    def registerLandmarks(self):

        registration = self.initializeRegistration(set_same_length=True)
        registration.performLandmarkTransform()
        self.transformation_matrix = registration.transformation_matrix

        transformed_source = self.verifyRegistration(registration.source_numpy)

        if self.checkBox_save_to_file.isChecked():
            self.saveData()

        # compute error
        error = Registration.computeError(transformed_source, registration.target_numpy)
        print("error")
        print(error)

        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix))

    def registerICP(self):
        print("not implemented yet")

        # registration = self.initializeRegistration()
        # registration.performICP()
        # self.transformation_matrix = registration.getTransformationMatrix()

        # transformed_targets = self.verifyRegistration(registration.target_numpy)

        # if self.checkBox_save_to_file.isChecked():
        #     self.saveData()

        # # compute error
        # # error = Registration.computeError(registration.source_numpy[registration.best_source_ind],\
        # #                           registration.target_numpy[registration.best_target_ind])

        # # display registration result in qt
        # self.label_registration_result.setText(str(self.transformation_matrix))
    
    def registerHungarian(self):

        registration = self.initializeRegistration()
        registration.performRegistrationByHungarian()

        self.transformation_matrix = registration.transformation_matrix
        transformed_source = self.verifyRegistration(registration.source_numpy)

        if self.checkBox_save_to_file.isChecked():
            self.saveData()
        
        # compute error
        error = Registration.computeError(transformed_source[registration.best_source_ind],\
                                  registration.target_numpy[registration.best_target_ind])

        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix) + '\n' + 'error: ' +str(error))

    def verifyRegistration(self, points):
        # transform world points to model space by registration result, visualize transformed points in viewer
        # points are 3D points of numpy array in shape (n,3)

        # make points to shape (n,4)
        num_pts = len(points)
        points = np.concatenate((points, np.array([1.0]*num_pts).reshape((num_pts, 1))), axis=1)
        points = np.matmul(self.transformation_matrix, points.T)
        transformed_points = []
        for point in points.T:
            point = point[:3]
            transformed_points.append(point.tolist())
            actor = self.viewer.createPointActor(point, color=GREEN)
            self.viewer.addActor(actor)

        transformed_points =np.array(transformed_points)

        return transformed_points

    def loadData(self):
        file_name, _ = QFileDialog.getOpenFileName(self, 'Open File', directory=DATA_PATH,filter=DATA_FILTER)
        if file_name != '' and os.path.splitext(file_name)[1] in DATA_FILTER:
            file_name = str(file_name)
            with open(file_name, 'r') as stream:
                reader = csv.reader(stream, delimiter=',')
                data_type = None
                for row in reader:
                    # check 
                    if len(row) == 1:
                        data_type = row[0]
                        # initialize self.transformation matrix to empty list if there is registration result
                        if data_type == "H":
                            self.transformation_matrix = []
                    else:
                        if data_type == "W":
                            rowPosition = self.tableWidget_world.rowCount()
                            self.tableWidget_world.insertRow(rowPosition)
                            self.tableWidget_world.setItem(rowPosition, 0, QTableWidgetItem(row[0]))
                            self.tableWidget_world.setItem(rowPosition, 1, QTableWidgetItem(row[1]))
                            self.tableWidget_world.setItem(rowPosition, 2, QTableWidgetItem(row[2]))
                        elif data_type == "M":
                            rowPosition = self.tableWidget_model.rowCount()
                            self.tableWidget_model.insertRow(rowPosition)
                            self.tableWidget_model.setItem(rowPosition, 0, QTableWidgetItem(row[0]))
                            self.tableWidget_model.setItem(rowPosition, 1, QTableWidgetItem(row[1]))
                            self.tableWidget_model.setItem(rowPosition, 2, QTableWidgetItem(row[2]))
                        elif data_type == "H":
                            transformation_matrix_row = [float(row[0]), float(row[1]), float(row[2]), float(row[3])]
                            self.transformation_matrix.append(transformation_matrix_row)
                # turn transformation matrix to np array if necessary
                if self.transformation_matrix is not None:
                    self.transformation_matrix = np.array(self.transformation_matrix)
                    self.label_registration_result.setText(str(self.transformation_matrix))



    def saveData(self):
        file_name, _ = QFileDialog.getSaveFileName(self, 'Save File', directory=DATA_PATH,filter=DATA_FILTER)
        if file_name != '' and os.path.splitext(file_name)[1] in DATA_FILTER:
            with open(file_name, 'wt') as stream:
                writer = csv.writer(stream, lineterminator='\n')
                # save table model
                writer.writerow("M")
                for row in range(self.tableWidget_model.rowCount()):
                    row_data = []
                    for column in range(self.tableWidget_model.columnCount()):
                        item = self.tableWidget_model.item(row, column).text()
                        row_data.append(item)
                    writer.writerow(row_data)
                # save table world 
                writer.writerow("W")
                for row in range(self.tableWidget_world.rowCount()):
                    row_data = []
                    for column in range(self.tableWidget_world.columnCount()):
                        item = self.tableWidget_world.item(row, column).text()
                        row_data.append(item)
                    writer.writerow(row_data)

                # save registration result
                writer.writerow("H")
                for i in range(4):
                    row = []
                    for j in range(4):
                        row.append(self.transformation_matrix[i,j])
                    writer.writerow(row)


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


    @pyqtSlot(int)
    def on_sig_alarm(self, alarmNum: int):
        if self.is_probing:
            self.is_probing = False

    @pyqtSlot(str)
    def on_sig_status(self, data: str):
        tblDecode = data[1:-1].split("|") # first remove the "< ... >" brackets, then split by "|"
        for D in tblDecode:
            if D[:5] == "MPos:" and self.is_probing:
                tblPos = D[5:].split(",")
                for i in range(5):
                    self.joints[i] = float(tblPos[i])
                # print(self.joints)

    @pyqtSlot()
    def on_sig_ok(self):
        print("OK")
        if self.is_probing:
            # do forward kinematics
            # tf_W_T = self.robot.fwd_kin(self.joints[0] + self.robot.offset_sensor[0], self.joints[1]+ self.robot.offset_sensor[1], self.joints[2] + self.robot.offset_sensor[2], self.joints[3], self.joints[4])
            # print(tf_W_T)
            robot_quat = [self.robot_rt_pose['rotation']['x'], self.robot_rt_pose['rotation']['y'], self.robot_rt_pose['rotation']['z'], self.robot_rt_pose['rotation']['w']]
            robot_tvec = [self.robot_rt_pose['translation']['x'], self.robot_rt_pose['translation']['y'], self.robot_rt_pose['translation']['z']]
            tf_T_W = RegistrationDialog.convertToSE3(np.asarray(robot_quat), np.asarray(robot_tvec))
            tf_S_T = np.asarray( [[1, 0, 0, 0.027289],
                                  [0, 1, 0, 0.00844],
                                  [0, 0, 1, -0.031],
                                  [0, 0, 0, 1]] )
            tf_S_W = np.matmul(tf_S_T, tf_T_W)

            self.x = tf_S_W[0,3]
            self.y = tf_S_W[1,3]
            self.z = tf_S_W[2,3]
            rowPosition = self.tableWidget_world.rowCount()
            self.tableWidget_world.insertRow(rowPosition)
            self.tableWidget_world.setItem(rowPosition, 0, QTableWidgetItem(str(self.x)))
            self.tableWidget_world.setItem(rowPosition, 1, QTableWidgetItem(str(self.y)))
            self.tableWidget_world.setItem(rowPosition, 2, QTableWidgetItem(str(self.z)))

            self.__grblCom.gcodePush("G0 Z0")
        self.is_probing = False

    @pyqtSlot()
    def on_sig_probe(self):
        self.__grblCom.gcodePush("G38.2 Z-{}".format(self.__di.doubleSpinBox_probe_limit.value()))
        self.is_probing = True

    def closeWindow(self):
        self.is_loaded = not self.is_loaded
        self.viewer.deleteAllPoints()