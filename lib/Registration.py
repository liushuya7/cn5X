import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QSpinBox, QDoubleSpinBox, QLineEdit
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

class RegistrationDialog(QObject):
	''' Registration dialog '''

	def __init__(self, grbl: grblCom, nbAxis: int, axisNames: list, reg_pts):
		super().__init__()
		self.__dlgConfig = QDialog()
		ui_dlgConfig = os.path.join(self_dir, '../ui/dlgConfig.ui')
		self.__di = uic.loadUi(ui_dlgConfig, self.__dlgConfig)
		self.__configChanged = False
		self.__changedParams = []
		self.__nbAxis = nbAxis
		self.__axisNames = axisNames
		self.__setNbAxes(self.__nbAxis, self.__axisNames)
        self.__reg_pts = reg_pts

	def 