
import os
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QTableWidgetItem, QMenu, QAction
from PyQt5.QtCore import Qt
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np

import Utils
from Utils import VTKUtils
from TwoMeshesInteraction import TwoMeshesInteraction
from PathState import PathState

class ActorInteractor(vtk.vtkInteractorStyleTrackballActor):
    def __init__(self, parent):
        super().__init__()
        self.AddObserver("RightButtonPressEvent",self.rightButtonPressEvent)
        self.picked_actor = None
        self.parent = parent

    def rightButtonPressEvent(self, obj, event):
        clickPos = self.GetInteractor().GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        
        # get actor 
        self.picked_actor = picker.GetActor()

        # self.OnLeftButtonDown()
        return