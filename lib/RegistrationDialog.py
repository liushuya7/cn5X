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
import vtk
import numpy as np
import csv
from robot_kins import CNC_5dof
from MeshProcessing import MeshProcessing
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance_matrix
from scipy.spatial.transform import Rotation as R

from compilOptions import grblCompilOptions

self_dir = os.path.dirname(os.path.realpath(__file__))
DEFAULT_PATH = os.path.join(self_dir, "../data")
DATA_FILTER = "CSV files (*.csv);;Text files (*.txt)"
RED = (255,0,0)
GREEN = (0,255,0)

class Registration():
    def __init__(self, source, target):
        
        self.source_numpy = np.array(source)
        self.target_numpy = np.array(target)
        self.initializeSourceAndTarget(source, target)

    def initializeSourceAndTarget(self, source, target):
        # create source vtkpolydata        
        sourcePoints = vtk.vtkPoints()
        sourceVertices = vtk.vtkCellArray()
        for point in source:
            id = sourcePoints.InsertNextPoint(point[0], point[1], point[2])
            sourceVertices.InsertNextCell(1)
            sourceVertices.InsertCellPoint(id)
        
        self.source = vtk.vtkPolyData()
        self.source.SetPoints(sourcePoints)
        self.source.SetVerts(sourceVertices)
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.source.Update()

        # create target vtkpolydata        
        targetPoints = vtk.vtkPoints()
        targetVertices = vtk.vtkCellArray()
        for point in target:
            id = targetPoints.InsertNextPoint(point[0], point[1], point[2])
            targetVertices.InsertNextCell(1)
            targetVertices.InsertCellPoint(id)
        
        self.target = vtk.vtkPolyData()
        self.target.SetPoints(targetPoints)
        self.target.SetVerts(targetVertices)
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.target.Update()

    def performICP(self):
        
        # create icp object
        icp = vtk.vtkIterativeClosestPointTransform()
        icp.SetSource(self.source)
        icp.SetTarget(self.target)
        icp.GetLandmarkTransform().SetModeToRigidBody()
        
        # parameters for icp
        #icp.DebugOn()
        # icp.SetMaximumNumberOfLandmarks(100)
        # icp.SetMaximumMeanDistance(0.0001)
        # icp.SetMaximumNumberOfIterations(100)
        icp.StartByMatchingCentroidsOn()
        icp.CheckMeanDistanceOn()
        icp.Modified()
        icp.Update()

        # icpTransformFilter = vtk.vtkTransformPolyDataFilter()
        # if vtk.VTK_MAJOR_VERSION <= 5:
        #     icpTransformFilter.SetInput(self.source)
        # else:
        #     icpTransformFilter.SetInputData(self.source)

        # icpTransformFilter.SetTransform(icp)
        # icpTransformFilter.Update()

        # transformedSource = icpTransformFilter.GetOutput()
        # # ============ display transformed points ==============
        # for index in range(6):
        #     point = [0,0,0]
        #     transformedSource.GetPoint(index, point)
        #     print("transformed source point[%s]=%s" % (index,point))

        self.transformation_matrix = icp.GetMatrix()

    def findCorrespondenceByHungarian(self, source_numpy, target_numpy):
        cost_matrix = distance_matrix(source_numpy, target_numpy)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        cost = cost_matrix[row_ind, col_ind].sum()
        
        return (cost, row_ind, col_ind)

    def performRegistrationByHungarian(self, transformation=None):
        # transform target points to align centroids with source target
        if transformation is not None:
            num_pts = len(self.target_numpy)
            target_centroided_numpy = self.target_numpy.copy()
            target_centroided_numpy = np.concatenate((target_centroided_numpy, np.array([1.0]*num_pts).reshape((num_pts, 1))), axis=1)
            target_centroided_numpy = np.matmul(np.linalg.inv(transformation), target_centroided_numpy.T) # (4xN)
            # make target_centroided_numpy from (4xN) to (Nx3)
            target_centroided_numpy = target_centroided_numpy.T[:,:3]
            # fit plane to feature points on implant and extract the normal
            c, normal = MeshProcessing.fitPlaneLTSQ(self.source_numpy)
            centroid = np.mean(target_centroided_numpy, axis=0)

            # rotate target_centroided points along the normal, find the one with smallest error
            cost_list = []
            source_ind_list = []
            target_ind_list = []
            for angle in np.linspace(0, 2*np.pi, num = 100):
                r = R.from_rotvec(angle*normal)
                rotation_matrix = r.as_matrix()
                translated_points = target_centroided_numpy - centroid.reshape((1,3))
                rotated_points = np.matmul(rotation_matrix, translated_points.T).T
                rotated_points = rotated_points + centroid.reshape((1,3))
                cost, source_ind, target_ind = self.findCorrespondenceByHungarian(self.source_numpy, rotated_points)
                cost_list.append(cost)
                source_ind_list.append(source_ind)
                target_ind_list.append(target_ind)
            cost_list = np.array(cost_list)
            source_ind_list = np.array(source_ind_list)
            target_ind_list = np.array(target_ind_list)

            best_match = np.argmin(cost_list)
            best_source_ind = source_ind_list[best_match]
            best_target_ind = target_ind_list[best_match]
            print("best source ind")
            print(best_source_ind)
            print("best target ind")
            print(best_target_ind)
            self.best_source_ind = best_source_ind
            self.best_target_ind = best_target_ind

            source = self.source_numpy[best_source_ind]
            target = target_centroided_numpy[best_target_ind]

        else:
            source_ind, target_ind = self.findCorrespondenceByHungarian(self.source_numpy, target_centroided_numpy)
            source = self.source_numpy[source_ind]
            target = self.target_centroided_numpy[target_ind]

        self.initializeSourceAndTarget(source, target)
        self.performLandmarkTransform()

    def performLandmarkTransform(self):

        # create landmark transform object
        lm_transform = vtk.vtkLandmarkTransform()
        lm_transform.SetSourceLandmarks(self.source.GetPoints())
        lm_transform.SetTargetLandmarks(self.target.GetPoints())
        lm_transform.SetModeToRigidBody()
        if vtk.VTK_MAJOR_VERSION <= 5:
            lm_transform.Update()
        self.transformation_matrix = lm_transform.GetMatrix()
        
    def getTransformationMatrix(self):
        # get numpy array from vtk matrix
        transformation_mat = np.ones((4,4))
        for i in range(4):
            for j in range(4):
                transformation_mat[i,j] = self.transformation_matrix.GetElement(i,j)
        return transformation_mat 

class RegistrationDialog(QDialog):
    ''' Registration dialog '''

    def __init__(self, grbl: grblCom, viewer=None):
        super().__init__()
        ui_dlgRegistration = os.path.join(self_dir, '../ui/registration.ui')
        self.__di = uic.loadUi(ui_dlgRegistration, self)
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

        self.robot = CNC_5dof()
        self.x = 0
        self.y = 0
        self.z = 0
        self.joints = np.zeros((5,1))

        # mesh processing
        if self.viewer.current_mesh_file:
            self.mesh_processor = MeshProcessing(self.viewer.current_mesh_file)
        else:
            self.mesh_processor = None

        self.finished.connect(self.closeWindow)
        self.show()

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
        file_name, _ = QFileDialog.getOpenFileName(self, 'Open File', directory=DEFAULT_PATH,filter=DATA_FILTER)
        if file_name != '' and os.path.splitext(file_name)[1] in DATA_FILTER:
            with open(file_name, 'r') as stream:
                reader = csv.reader(stream, delimiter=',')
                data_type = None
                for row in reader:
                    rowPosition = self.tableWidget_world.rowCount()
                    self.tableWidget_world.insertRow(rowPosition)
                    self.tableWidget_world.setItem(rowPosition, 0, QTableWidgetItem(row[0]))
                    self.tableWidget_world.setItem(rowPosition, 1, QTableWidgetItem(row[1]))
                    self.tableWidget_world.setItem(rowPosition, 2, QTableWidgetItem(row[2]))

    def initializeRegistration(self, set_same_length=False):
        # get source points and target points from tablewidget_world and tablewidget_model 
        target_data = []
        for row in range(self.tableWidget_world.rowCount()):
            target_data.append([])
            for col in range(self.tableWidget_world.columnCount()):
                    target_data[row].append(float(self.tableWidget_world.item(row, col).text()))

        source_data = []
        for row in range(self.tableWidget_model.rowCount()):
            source_data.append([])
            for col in range(self.tableWidget_model.columnCount()):
                    source_data[row].append(float(self.tableWidget_model.item(row, col).text()))

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
        self.transformation_matrix = registration.getTransformationMatrix()

        transformed_targets = self.verifyRegistration(registration.target_numpy)

        if self.checkBox_save_to_file.isChecked():
            self.saveData()

        # compute error
        error = self.computeError(registration.source_numpy[registration.best_source_ind],\
                                  registration.target_numpy[registration.best_target_ind])

        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix))

    def registerICP(self):

        registration = self.initializeRegistration()
        registration.performICP()
        self.transformation_matrix = registration.getTransformationMatrix()

        transformed_targets = self.verifyRegistration(registration.target_numpy)

        if self.checkBox_save_to_file.isChecked():
            self.saveData()

        # compute error
        error = self.computeError(registration.source_numpy[registration.best_source_ind],\
                                  registration.target_numpy[registration.best_target_ind])

        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix))
    
    def registerHungarian(self):

        registration = self.initializeRegistration()
        registration.performICP()
        transformation_align_centroids = registration.getTransformationMatrix()
        registration.performRegistrationByHungarian(transformation_align_centroids)
        self.transformation_matrix = registration.getTransformationMatrix()

        self.transformation_matrix = np.matmul(transformation_align_centroids, self.transformation_matrix)
        transformed_targets = self.verifyRegistration(registration.target_numpy)

        if self.checkBox_save_to_file.isChecked():
            self.saveData()
        
        # compute error
        error = self.computeError(registration.source_numpy[registration.best_source_ind],\
                                  transformed_targets[registration.best_target_ind])

        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix) + '\n' + 'error: ' +str(error))

    def verifyRegistration(self, points):
        # transform world points to model space by registration result, visualize transformed points in viewer
        # points are 3D points of numpy array in shape (n,3)
        # make points to shape (n,4)
        num_pts = len(points)
        points = np.concatenate((points, np.array([1.0]*num_pts).reshape((num_pts, 1))), axis=1)
        points = np.matmul(np.linalg.inv(self.transformation_matrix), points.T)
        transformed_points = []
        for point in points.T:
            point = point[:3]
            transformed_points.append(point.tolist())
            actor = self.viewer.createPointActor(point, color=GREEN)
            self.viewer.addActor(actor)

        transformed_points =np.array(transformed_points)

        return transformed_points

    def computeError(self, source, target):
        total_dist = 0
        for i in range(len(source)):
            dist = np.linalg.norm(source[i] - target[i])
            total_dist +=dist 
            print("source: " + str(source[i]))
            print("target: " + str(target[i]))
        avg_dist = total_dist/len(source)

        return avg_dist

    def saveData(self):
        file_name, _ = QFileDialog.getSaveFileName(self, 'Save File', directory=DEFAULT_PATH,filter=DATA_FILTER)
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
                print(self.joints)
                # self.x = float(tblPos[0])
                # self.y = float(tblPos[1])
                # self.z = float(tblPos[2])

    @pyqtSlot()
    def on_sig_ok(self):
        # print("OK")
        if self.is_probing:
            # self.reg_pts.append([x,y,z])
            # do forward kinematics
            tf_W_T = self.robot.fwd_kin(self.joints[0] + self.robot.offset_sensor[0], self.joints[1]+ self.robot.offset_sensor[1], self.joints[2] + self.robot.offset_sensor[2], self.joints[3], self.joints[4])
            print(tf_W_T)
            self.x = tf_W_T[0,3]
            self.y = tf_W_T[1,3]
            self.z = tf_W_T[2,3]
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