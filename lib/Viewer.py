import os
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QTableWidgetItem, QMenu, QAction
from PyQt5.QtCore import Qt
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np

from Utils import VTKUtils
from TwoMeshesInteraction import TwoMeshesInteraction

MAGNIFY_RATIO = 3
POINT_SIZE = 5
RED = (255,0,0)
GREEN = (255,0,0)

class Viewer(QFrame):

    def __init__(self, parent, Qmaster):
        super(Viewer,self).__init__(parent)
        self.Qmaster = Qmaster

        # Make the actual QtWidget a child so that it can be re parented
        interactor = QVTKRenderWindowInteractor(self)
        self.layout = QHBoxLayout()
        self.layout.addWidget(interactor)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)

        # Setup VTK environment
        renderer = vtk.vtkRenderer()
        render_window = interactor.GetRenderWindow()
        render_window.AddRenderer(renderer)

        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        render_window.SetInteractor(interactor)
        renderer.SetBackground(0.2,0.2,0.2)
        
        self.renderer = renderer
        self.interactor = interactor
        self.render_window = render_window

        # set up for context menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.onContextMenu)
        self.actor_menu = ActorMenu(self)

        # mesh file
        self.mesh_actors = {}

        # Set up picker (ray-casting)
        self.picker_tag_id = None
        self.points = []
        self.picker = vtk.vtkCellPicker()
        self.picker.AddObserver("EndPickEvent", self.processPick)
        self.interactor.SetPicker(self.picker)

        # Path Generation
        self.to_show_normal = False
        self.normal_vector = None
        self.picked_point = None
        self.cut_path = None

        # Multi-mesh interaction
        self.two_mesh_interaction = None
        self.extraction_finished = None
        
        # key A for actor mode
        self.interactor.AddObserver("MiddleButtonPressEvent", self.onPressedActor)
        self.interactor.AddObserver("MiddelButtonReleaseEvent", self.onReleasedActor)

    def start(self):
        self.interactor.Initialize()
        self.interactor.Start()

    def loadModel(self, file_name):
        mesh_name = os.path.split(file_name)[-1]

        # Load mesh file
        actor = VTKUtils.loadSTL(file_name)

        self.mesh_actors[mesh_name] = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

    def processPick(self, object, event):
        point_id = object.GetPointId()
        if point_id >= 0:
            picked_point = object.GetPickPosition()#GetSelectionPoint()
            if self.to_show_normal:
                picked_normal = object.GetPickNormal()
                self.showAxis(picked_point, picked_normal)
                self.normal_vector = picked_normal
                self.picked_point = picked_point
            # add a point
            actor = VTKUtils.createPointActor(picked_point, color=RED)
            self.addActor(actor)
            self.points.append(actor)

            # # debug
            # actor_collection = self.renderer.GetActors()
            # print(actor_collection.GetNumberOfItems())
            if self.Qmaster.registration_dialog and self.Qmaster.registration_dialog.is_loaded:
                # add picked points to table in RegistrationDialog
                self.addPointToTable(picked_point)

        else:
            print("not picking anything!")

    def addPointToTable(self, picked_point):
        rowPosition = self.Qmaster.registration_dialog.tableWidget_model.rowCount()
        self.Qmaster.registration_dialog.tableWidget_model.insertRow(rowPosition)
        self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 0, QTableWidgetItem(str(picked_point[0])))
        self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 1, QTableWidgetItem(str(picked_point[1])))
        self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 2, QTableWidgetItem(str(picked_point[2])))

    def clickToPick(self, object, event):
        x, y = object.GetEventPosition()
        self.picker.Pick(x,y,0, self.renderer)

    def switchMode(self, mode):
        if mode == "draw":
            self.picker_tag_id = self.interactor.AddObserver(vtk.vtkCommand.LeftButtonPressEvent, self.clickToPick, 10)
            print("id")
            print(self.picker_tag_id)
        else:
            if self.picker_tag_id:
                self.interactor.RemoveObserver(self.picker_tag_id)
    
    def deleteLatestPoint(self):
        self.renderer.RemoveActor(self.points.pop())
        self.update()

    def deleteSelectedPoint(self, point_id):
        # delete point in renderer
        self.renderer.RemoveActor(self.points[point_id])
        del self.points[point_id]
        self.update()
    
    def deleteAllPoints(self):

        for point in self.points:
            self.renderer.RemoveActor(point)
        self.points.clear()
        self.update()
        print("delete all points")


    def magnifyPoint(self, current, previous):
        self.points[current.row()].GetProperty().SetPointSize(POINT_SIZE * MAGNIFY_RATIO)
        if previous:
            self.points[previous.row()].GetProperty().SetPointSize(POINT_SIZE)
        self.update()

    def solveCuttingVector(self, point, center, vector, theta):
        # get solution from matlab
        numerator1 = -(4*center[0]**2*np.cos(theta)**4*vector[1]**2 + 4*center[0]**2*np.cos(theta)**4*vector[2]**2 + 8*center[0]*np.cos(theta)**4*point[1]*vector[0]*vector[1] + 8*center[0]*np.cos(theta)**4*point[2]*vector[0]*vector[2] + 4*center[1]**2*np.cos(theta)**4*vector[0]**2 + 4*center[1]**2*np.cos(theta)**4*vector[2]**2 + 8*center[1]*np.cos(theta)**4*point[0]*vector[0]*vector[1] + 8*center[1]*np.cos(theta)**4*point[2]*vector[1]*vector[2] + 4*center[2]**2*np.cos(theta)**4*vector[0]**2 + 4*center[2]**2*np.cos(theta)**4*vector[1]**2 + 8*center[2]*np.cos(theta)**4*point[0]*vector[0]*vector[2] + 8*center[2]*np.cos(theta)**4*point[1]*vector[1]*vector[2] + 4*np.cos(theta)**4*point[0]**2*vector[1]**2 + 4*np.cos(theta)**4*point[0]**2*vector[2]**2 + 4*np.cos(theta)**4*point[1]**2*vector[0]**2 + 4*np.cos(theta)**4*point[1]**2*vector[2]**2 + 4*np.cos(theta)**4*point[2]**2*vector[0]**2 + 4*np.cos(theta)**4*point[2]**2*vector[1]**2 ) + ( 8*center[0]*np.cos(theta)**4*point[0]*vector[1]**2 + 8*center[0]*np.cos(theta)**4*point[0]*vector[2]**2 + 8*center[1]*np.cos(theta)**4*point[1]*vector[0]**2 + 8*center[1]*np.cos(theta)**4*point[1]*vector[2]**2 + 8*center[2]*np.cos(theta)**4*point[2]*vector[0]**2 + 8*center[2]*np.cos(theta)**4*point[2]*vector[1]**2 + 8*center[0]*center[1]*np.cos(theta)**4*vector[0]*vector[1] + 8*center[0]*center[2]*np.cos(theta)**4*vector[0]*vector[2] + 8*center[1]*center[2]*np.cos(theta)**4*vector[1]*vector[2] + 8*np.cos(theta)**4*point[0]*point[1]*vector[0]*vector[1] + 8*np.cos(theta)**4*point[0]*point[2]*vector[0]*vector[2] + 8*np.cos(theta)**4*point[1]*point[2]*vector[1]*vector[2] + 1)
        numerator1 = -numerator1
        # numerator1 = 4*center[0]**2*np.cos(theta)**4*vector[1]**2 + 4*center[0]**2*np.cos(theta)**4*vector[2]**2 + 8*center[0]*np.cos(theta)**4*point[1]*vector[0]*vector[1] + 8*center[0]*np.cos(theta)**4*point[2]*vector[0]*vector[2] + 4*center[1]**2*np.cos(theta)**4*vector[0]**2 + 4*center[1]**2*np.cos(theta)**4*vector[2]**2 + 8*center[1]*np.cos(theta)**4*point[0]*vector[0]*vector[1] + 8*center[1]*np.cos(theta)**4*point[2]*vector[1]*vector[2] + 4*center[2]**2*np.cos(theta)**4*vector[0]**2 + 4*center[2]**2*np.cos(theta)**4*vector[1]**2 + 8*center[2]*np.cos(theta)**4*point[0]*vector[0]*vector[2] + 8*center[2]*np.cos(theta)**4*point[1]*vector[1]*vector[2] + 4*np.cos(theta)**4*point[0]**2*vector[1]**2 + 4*np.cos(theta)**4*point[0]**2*vector[2]**2 + 4*np.cos(theta)**4*point[1]**2*vector[0]**2 + 4*np.cos(theta)**4*point[1]**2*vector[2]**2 + 4*np.cos(theta)**4*point[2]**2*vector[0]**2 + 4*np.cos(theta)**4*point[2]**2*vector[1]**2
        # numerator2 = 8*center[0]*np.cos(theta)**4*point[0]*vector[1]**2 + 8*center[0]*np.cos(theta)**4*point[0]*vector[2]**2 + 8*center[1]*np.cos(theta)**4*point[1]*vector[0]**2 + 8*center[1]*np.cos(theta)**4*point[1]*vector[2]**2 + 8*center[2]*np.cos(theta)**4*point[2]*vector[0]**2 + 8*center[2]*np.cos(theta)**4*point[2]*vector[1]**2 + 8*center[0]*center[1]*np.cos(theta)**4*vector[0]*vector[1] + 8*center[0]*center[2]*np.cos(theta)**4*vector[0]*vector[2] + 8*center[1]*center[2]*np.cos(theta)**4*vector[1]*vector[2] + 8*np.cos(theta)**4*point[0]*point[1]*vector[0]*vector[1] + 8*np.cos(theta)**4*point[0]*point[2]*vector[0]*vector[2] + 8*np.cos(theta)**4*point[1]*point[2]*vector[1]*vector[2] + 1
        numerator2 = 2*np.cos(theta)**2*point[0]*vector[0] + 2*np.cos(theta)**2*point[1]*vector[1] + 2*np.cos(theta)**2*point[2]*vector[2] - 2*center[0]*np.cos(theta)**2*vector[0] - 2*center[1]*np.cos(theta)**2*vector[1] - 2*center[2]*np.cos(theta)**2*vector[2] + 1
        denominator = 2*np.cos(theta)**2*(vector[0]**2 + vector[1]**2 + vector[2]**2)
        condition = numerator1 > 0 and (denominator != 0)
        solu = None
        if condition:
            t1 = (np.sqrt(numerator1) + numerator2) /denominator
            t2 = (-np.sqrt(numerator1) + numerator2) /denominator
            solu = t1 if t1 < t2 else t2
        else:
            print("condition fails!")
        return solu

    def findCuttingVectors(self, angle):
        theta = angle/180.0*np.pi # angle (degree), theta (radian)
        self.cut_path = []
        for point in self.points:
            bounds = point.GetBounds()
            pt = (bounds[0], bounds[2], bounds[4])
            t = self.solveCuttingVector(pt, self.picked_point, self.normal_vector, theta)
            if t:
                p = np.array(pt)
                q = np.array(self.picked_point) + t*np.array((self.normal_vector))
                vector = (q-p)/np.linalg.norm(q-p)
                self.showAxis(pt, (vector[0], vector[1], vector[2]))
                path = list(pt)
                path.extend(vector.tolist())
                self.cut_path.append(path)

    def showAxis(self, point, vector):
        scale = 10
        p0 = np.array(point) + np.array(vector) * scale
        p1 = np.array(point) - np.array(vector) * scale

        actor = VTKUtils.createLineActor(p0, p1)
        self.addActor(actor)

    def getCutPath(self):
        return self.cut_path

    def addActor(self, actor):
        self.renderer.AddActor(actor)
        self.update()

    def transformSource(self, source_name, target_name):

        if self.picked_point:
            self.two_mesh_interaction = TwoMeshesInteraction(self.mesh_actors[source_name], self.mesh_actors[target_name])
            self.two_mesh_interaction.transformSource(self.picked_point)
            self.update()
        else:
            print("No target point clicked yet.")

    def boolean_operation(self, source_name, target_name, operation='intersection'):
        if not self.two_mesh_interaction:
            self.two_mesh_interaction = TwoMeshesInteraction(self.mesh_actors[source_name], self.mesh_actors[target_name])
        
        print("working on boolean operation...")
        actor = self.two_mesh_interaction.createBooleanOperationActor()
        actor = self.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='all')
        self.renderer.AddActor(actor)

    def update(self):
        self.renderer.GetRenderWindow().Render()

    def setInteractorStyle(self, style):
        if style == 'camera':
            interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        elif style == 'actor':
            interactor_style = ActorInteractor(self)
        
        self.interactor.SetInteractorStyle(interactor_style)

    def addPathGenerationDialog(self, path_generation_dialog):
        self.path_generation_dialog = path_generation_dialog
        self.to_show_normal = self.path_generation_dialog.checkBox_show_normal.isChecked()


    def onContextMenu(self, point):
        self.actor_menu.exec_(self.mapToGlobal(point))
        # self.popMenu.exec_(self.parent.mapToGlobal(point))
    
    def onPressedActor(self, obj, event):
        print("key pressed")
        self.setInteractorStyle('actor')

    def onReleasedActor(self, obj, event):
        print("key released")
        self.setInteractorStyle('camera')


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

class ActorMenu(QMenu):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        action_target = QAction("Target", self)
        action_source = QAction("Source", self)
        action_start_extraction = QAction("Start Extraction", self)
        action_extract_cell = QAction("Extract Cell", self)
        action_finish_extraction = QAction("Finish Extraction", self)
        action_delete_cell = QAction("Delete Cell", self)
        action_target.triggered.connect(self.setTarget)
        action_source.triggered.connect(self.setSource)
        action_start_extraction.triggered.connect(self.startExtraction)
        action_extract_cell.triggered.connect(self.extractCell)
        action_finish_extraction.triggered.connect(self.finishExtraction)
        action_delete_cell.triggered.connect(self.deleteCell)
        self.addAction(action_target)
        self.addAction(action_source)
        self.addAction(action_start_extraction)
        self.addAction(action_extract_cell)
        self.addAction(action_finish_extraction)
        self.addAction(action_delete_cell)
    
    def setTarget(self):
        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                for name, mesh_actor in self.parent.mesh_actors.items():
                    if picked_actor is mesh_actor:
                        self.parent.path_generation_dialog.putTargetName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)

        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)

    def setSource(self):
        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                for name, mesh_actor in self.parent.mesh_actors.items():
                    if picked_actor is mesh_actor:
                        self.parent.path_generation_dialog.putSourceName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)
        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)

    def startExtraction(self):
        clickPos = self.parent.interactor.GetEventPosition()
        cell_picker = vtk.vtkCellPicker()
        cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        cell_id = cell_picker.GetCellId()
        if cell_id > 0:
            self.actor_to_be_extracted = cell_picker.GetActor()
            self.parent.extraction_finished = False
            self.append_filter = vtk.vtkAppendPolyData()
            print("Start extraction.")
        else:
            print("No actor is selected, extraction is not started yet.")

    def finishExtraction(self):
        # reset extraction mode
        self.parent.extraction_finished = None
        # create extraction actor
        mapper = self.actor_to_be_extracted.GetMapper()
        mapper.SetInputConnection(self.append_filter.GetOutputPort())  
        self.parent.update()
        print("Finished extraction")

    def extractCell(self):
        clickPos = self.parent.interactor.GetEventPosition()
        cell_picker = vtk.vtkCellPicker()
        cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        cell_id = cell_picker.GetCellId()
        if self.parent.extraction_finished is None:
            print("not in extraction mode")
            return
        if not self.parent.extraction_finished:
            cell_id = cell_picker.GetCellId()
            if cell_id > 0:
                print(cell_id)
                actor = cell_picker.GetActor()
                actor_cc = self.parent.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)

                # append poly data
                poly_data = actor_cc.GetMapper().GetInput()
                self.append_filter.AddInputData(poly_data)

            else:
                print("No cell picked")

    def deleteCell(self):
        pass
        # clickPos = self.parent.interactor.GetEventPosition()
        # cell_picker = vtk.vtkCellPicker()
        # cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        # cell_id = cell_picker.GetCellId()
        # if self.parent.extraction_finished is None:
        #     print("not in extraction mode")
        #     return
        # if not self.parent.extraction_finished:
        #     cell_id = object.GetCellId()
        #     if cell_id > 0:
        #         print(cell_id)
        #         actor = cell_picker.GetActor()
        #         actor_cc = self.parent.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)
        #         # # delete cell from actor
        #         # poly_data = actor.GetMapper().GetInput()
        #         # poly_data.DeleteCell(cell_id)
        #         # poly_data.RemoveDeletedCells()

        #         # append poly data
        #         poly_data = actor_cc.GetMapper().GetInput()
        #         append_filter.AddInputData(poly_data)