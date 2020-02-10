import os
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QTableWidgetItem
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np

MAGNIFY_RATIO = 3
POINT_SIZE = 5

class Viewer(QFrame):

    def __init__(self, parent, Qmaster):
        super(Viewer,self).__init__(parent)
        self.Qmaster = Qmaster

        # Make the actual QtWidget a child so that it np.cos(theta)n be re parented
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

        # Set up picker (ray-casting)
        self.picker_tag_id = None
        self.points = []
        self.picker = vtk.vtkCellPicker()
        self.picker.AddObserver("EndPickEvent", self.processPick)
        self.interactor.SetPicker(self.picker)

        # Path Generation
        self.to_show_axis = False
        self.axis_vector = None
        self.machine_center = None
        self.cut_path = None

    def start(self):
        self.interactor.Initialize()
        self.interactor.Start()

    def loadModel(self, file_name):
        # Load mesh file
        reader = vtk.vtkSTLReader()
        reader.SetFileName(file_name)
        # mapper 
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(reader.GetOutput())
        else:
            mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

    def processPick(self, object, event):
        point_id = object.GetPointId()
        if point_id >= 0:
            picked_point = object.GetPickPosition()#GetSelectionPoint()
            if self.to_show_axis:
                picked_normal = object.GetPickNormal()
                self.showAxis(picked_point, picked_normal)
                self.axis_vector = picked_normal
                self.machine_center = picked_point
            # add a point
            # Create the geometry of a point (the coordinate)
            points = vtk.vtkPoints()
            p = list(picked_point)
            # Create the topology of the point (a vertex)
            vertices = vtk.vtkCellArray()
            id = points.InsertNextPoint(p)
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(id)
            # Create a polydata object
            point = vtk.vtkPolyData()
            # Set the points and vertices we created as the geometry and topology of the polydata
            point.SetPoints(points)
            point.SetVerts(vertices)

            # Create a mapper
            # Visualize
            mapper = vtk.vtkPolyDataMapper()
            if vtk.VTK_MAJOR_VERSION <= 5:
                mapper.SetInput(point)
            else:
                mapper.SetInputData(point)
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(1,0,0)
            actor.GetProperty().SetPointSize(POINT_SIZE)
            self.points.append(actor)
            self.renderer.AddActor(actor)

            # # debug
            # actor_collection = self.renderer.GetActors()
            # print(actor_collection.GetNumberOfItems())
            if self.Qmaster.registrationLoaded:
                # add picked points to table in RegistrationDialog
                rowPosition = self.Qmaster.registration_dialog.tableWidget_model.rowCount()
                self.Qmaster.registration_dialog.tableWidget_model.insertRow(rowPosition)
                self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 0, QtGui.QTableWidgetItem(str(picked_point[0])))
                self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 1, QtGui.QTableWidgetItem(str(picked_point[1])))
                self.Qmaster.registration_dialog.tableWidget_model.setItem(rowPosition, 2, QtGui.QTableWidgetItem(str(picked_point[2])))

        else:
            print("not picking anything!")

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


    def magnifyPoint(self, current, previous):
        print("row selected")
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
            t = self.solveCuttingVector(pt, self.machine_center, self.axis_vector, theta)
            if t:
                p = np.array(pt)
                q = np.array(self.machine_center) + t*np.array((self.axis_vector))
                vector = (q-p)/np.linalg.norm(q-p)
                self.showAxis(pt, (vector[0], vector[1], vector[2]))
                path = list(pt)
                path.extend(vector.tolist())
                self.cut_path.append(path)

    def showAxis(self, point, vector):
        # Create three points. Join (Origin and P0) with a red line and
        scale = 10
        p0 = np.array(point) + np.array(vector) * scale
        p1 = np.array(point) - np.array(vector) * scale

        # Create a vtkPoints object and store the points in it
        pts = vtk.vtkPoints()
        pts.InsertNextPoint(p0)
        pts.InsertNextPoint(p1)

        # Setup two colors - one for each line
        green = [0, 255, 0]

        # Setup the colors array
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")

        # Add the colors we created to the colors array
        colors.InsertNextTypedTuple(green)

        # Create the first line (between Origin and P0)
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0,0) # the second 0 is the index of the Origin in the vtkPoints
        line.GetPointIds().SetId(1,1) # the second 1 is the index of P0 in the vtkPoints

        # Create a cell array to store the lines in and add the lines to it
        lines = vtk.vtkCellArray()
        lines.InsertNextCell(line)

        # Create a polydata to store everything in
        linesPolyData = vtk.vtkPolyData()

        # Add the points to the dataset
        linesPolyData.SetPoints(pts)

        # Add the lines to the dataset
        linesPolyData.SetLines(lines)

        # Color the lines - associate the first component (red) of the
        # colors array with the first component of the cell array (line 0)
        # and the second component (green) of the colors array with the
        # second component of the cell array (line 1)
        linesPolyData.GetCellData().SetScalars(colors)

        # Visualize
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(linesPolyData)
        else:
            mapper.SetInputData(linesPolyData)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.renderer.AddActor(actor)
        self.update()

    def getCutPath(self):
        return self.cut_path

    def update(self):
        self.renderer.GetRenderWindow().Render()



