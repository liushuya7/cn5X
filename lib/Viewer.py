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
from ViewerActionMenu import ViewerActionMenu
from ActorInteractor import ActorInteractor

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
        print("boolean operation is finished")

        for mesh_actor in self.mesh_actors.values():
            self.renderer.RemoveActor(mesh_actor)
        self.mesh_actors['path_volume'] = actor
        self.renderer.AddActor(actor)
        self.update()

    def update(self):
        self.renderer.GetRenderWindow().Render()

    def setInteractorStyle(self, style):
        if style == 'camera':
            interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        elif style == 'actor':
            interactor_style = ActorInteractor(self)
        
        self.interactor.SetInteractorStyle(interactor_style)

    def addPathGenerationDialog(self, path_generation_dialog):
        # add path generation dialog
        self.path_generation_dialog = path_generation_dialog
        self.to_show_normal = self.path_generation_dialog.checkBox_show_normal.isChecked()

    def addActionMenu(self):
        # set up for context menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.onContextMenu)
        self.action_menu = ViewerActionMenu(self)

    def onContextMenu(self, point):
        self.action_menu.exec_(self.mapToGlobal(point))
        # self.popMenu.exec_(self.parent.mapToGlobal(point))

    def generatePath(self, mesh_name, dist_layer, dist_line):
        mesh_actor = self.mesh_actors[mesh_name]
        VTKUtils.vtkPointsToNumpyArray(mesh_actor.GetMapper().GetInput().GetPoints())

        # c_main, normal_main = Utils.fitPlaneLTSQ()
        # cross section
        centroid = VTKUtils.getCenterOfActor(mesh_actor)
        point_reference = self.picked_point
        normal_main = self.normal_vector
        vector_augmented = np.array(point_reference) - np.array(centroid)
        normal = np.cross(normal_main, vector_augmented)
        normal = normal/np.linalg.norm(normal)
        
        layer_finished = False
        layer_count = 0
        while not layer_finished: 
            layer_count += 1
            #create a main plane to cut
            plane_main=vtk.vtkPlane()
            point_main = np.array(point_reference) - np.array(normal_main) * dist_layer * layer_count
            plane_main.SetOrigin(point_main)
            plane_main.SetNormal(normal_main)
            #create cutter
            cutter_main=vtk.vtkCutter()
            cutter_main.SetCutFunction(plane_main)
            cutter_main.SetInputData(mesh_actor.GetMapper().GetInput())
            cutter_main.Update()
            if cutter_main.GetOutput().GetNumberOfCells() > 0:
                cutterMapper_main=vtk.vtkPolyDataMapper()
                cutterMapper_main.SetInputConnection(cutter_main.GetOutputPort())
                # actor_main = vtk.vtkActor()
                # actor_main.SetMapper(cutterMapper_main)
                # ren.AddActor(actor_main)
                line_1_finished = False
                line_2_finished = False
                line_count = 0
                while not (line_1_finished and line_2_finished):
                    if not line_1_finished:
                        plane=vtk.vtkPlane()
                        point = point_main + np.array(normal) * dist_line * line_count
                        plane.SetOrigin(point)
                        plane.SetNormal(normal)
                        #create cutter
                        cutter=vtk.vtkCutter()
                        cutter.SetCutFunction(plane)
                        cutter.SetInputConnection(cutter_main.GetOutputPort())
                        cutter.Update()
                        poly_data_vtk = cutter.GetOutput()
                        poly_data_numpy = VTKUtils.vtkPointsToNumpyArray(poly_data_vtk.GetPoints())
                        if len(poly_data_numpy) > 0:
                            if len(poly_data_numpy)== 2:
                                line_actor = VTKUtils.createLineActor(poly_data_numpy[0], poly_data_numpy[1])
                                self.renderer.AddActor(line_actor)
                            cutterMapper=vtk.vtkPolyDataMapper()
                            cutterMapper.SetInputConnection(cutter.GetOutputPort())
                            actor = vtk.vtkActor()
                            actor.SetMapper(cutterMapper)
                            self.renderer.AddActor(actor)
                        else:
                            line_1_finished = True

                    line_count += 1

                    if not line_2_finished:
                        plane=vtk.vtkPlane()
                        point = point_main - np.array(normal) * dist_line * line_count
                        plane.SetOrigin(point)
                        plane.SetNormal(normal)
                        #create cutter
                        cutter=vtk.vtkCutter()
                        cutter.SetCutFunction(plane)
                        cutter.SetInputConnection(cutter_main.GetOutputPort())
                        cutter.Update()
                        poly_data_vtk = cutter.GetOutput()
                        poly_data_numpy = VTKUtils.vtkPointsToNumpyArray(poly_data_vtk.GetPoints())
                        if len(poly_data_numpy) > 0:
                            if len(poly_data_numpy)== 2:
                                line_actor = VTKUtils.createLineActor(poly_data_numpy[0], poly_data_numpy[1])
                                self.renderer.AddActor(line_actor)
                            cutterMapper=vtk.vtkPolyDataMapper()
                            cutterMapper.SetInputConnection(cutter.GetOutputPort())
                            actor = vtk.vtkActor()
                            actor.SetMapper(cutterMapper)
                            self.renderer.AddActor(actor)
                        else:
                            line_2_finished = True
                    
            else:
                layer_finished = True
        self.renderer.RemoveActor(mesh_actor)
        