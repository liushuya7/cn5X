import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QSpinBox, QDoubleSpinBox, QLineEdit, QTableWidgetItem
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic
from cn5X_config import *
from grblCom import grblCom
from msgbox import *
from qweditmask import qwEditMask

# Registration
import vtk
import numpy as np
from robot_kins import CNC_5dof

from compilOptions import grblCompilOptions

self_dir = os.path.dirname(os.path.realpath(__file__))

class Registration():
    def __init__(self, source, target):
        
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
        icp.SetMaximumNumberOfLandmarks(5)
        icp.SetMaximumMeanDistance(0.0001)
        icp.SetMaximumNumberOfIterations(20)
        icp.StartByMatchingCentroidsOn()
        icp.CheckMeanDistanceOn()
        icp.Modified()
        if vtk.VTK_MAJOR_VERSION <= 5:
            icp.Update()

        self.transformation_matrix = icp.GetMatrix()

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
        self.__di.tableWidget_model.currentItemChanged.connect(self.magnifyItem)

        self.robot = CNC_5dof()
        self.x = 0
        self.y = 0
        self.z = 0
        self.joints = np.zeros((5,1))

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

    def register(self):
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

        registration = Registration(source_data, target_data)
        registration.performLandmarkTransform()
        self.transformation_matrix = registration.getTransformationMatrix()
        # display registration result in qt
        self.label_registration_result.setText(str(self.transformation_matrix))

        if self.checkBox_save_to_file.isChecked():
            self.saveData()

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
                # self.x = float(tblPos[0])
                # self.y = float(tblPos[1])
                # self.z = float(tblPos[2])

    @pyqtSlot()
    def on_sig_ok(self):
        # print("OK")
        if self.is_probing:
            # self.reg_pts.append([x,y,z])
            # do forward kinematics
            tf_W_T = self.robot.fwd_kin(self.joints[0], self.joints[1], self.joints[2], self.joints[3], self.joints[4])
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