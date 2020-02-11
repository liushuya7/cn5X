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
from Viewer import Viewer

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

    def __init__(self, grbl: grblCom):
        super().__init__()
        ui_dlgRegistration = os.path.join(self_dir, '../ui/registration.ui')
        self.__di = uic.loadUi(ui_dlgRegistration, self)

        self.__grblCom = grbl
        self.__grblCom.sig_alarm.connect(self.on_sig_alarm)
        self.__grblCom.sig_ok.connect(self.on_sig_ok)
        self.__grblCom.sig_status.connect(self.on_sig_status)
        self.is_probing = False

        self.__di.pushButton_collect.clicked.connect(self.on_sig_probe)

        self.x = 0
        self.y = 0
        self.z = 0

        self.show()

    def undoModel(self):

        # delete point in table
        rowPosition = self.tableWidget_model.rowCount()
        self.tableWidget_model.removeRow(rowPosition - 1)

        # delete point in renderer
        self.vtk_widget.delete_latest_point()

    def magnifyItem(self, current, previous):
        print("magnify triggered!!!!!!!!!")
        self.vtk_widget.magnify_point(current, previous)
        print("items")
        print(current)
        print(previous)

    def deleteSelectedTablePoint(self, table):
        current_row = table.currentRow()
        table.removeRow(current_row)

    def deleteSelectedWorldPoint(self):
        self.deleteSelectedTablePoint(self.tableWidget_world)

    def deleteSelectedModelPoint(self):
        current_row = self.tableWidget_model.currentRow()
        # self.tableWidget_model.removeRow(current_row)
        self.deleteSelectedTablePoint(self.tableWidget_model)
        self.vtk_widget.delete_selected_point(current_row)

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
                self.x = float(tblPos[0])
                self.y = float(tblPos[1])
                self.z = float(tblPos[2])

    @pyqtSlot()
    def on_sig_ok(self):
        # print("OK")
        if self.is_probing:
            # self.reg_pts.append([x,y,z])
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
