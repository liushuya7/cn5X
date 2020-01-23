import os
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QTableWidgetItem
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

MAGNIFY_RATIO = 3
POINT_SIZE = 5

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

        # Set up picker (ray-casting)
        self.picker_tag_id = None
        self.points = []
        self.picker = vtk.vtkCellPicker()
        self.picker.AddObserver("EndPickEvent", self.processPick)
        self.interactor.SetPicker(self.picker)

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

    def processPick(self, object, event):
        point_id = object.GetPointId()
        if point_id >= 0:
            picked_point = object.GetPickPosition()#GetSelectionPoint()
            # print(picked_point)

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

            # add coordinate to qtable
            rowPosition = self.Qmaster.tableWidget_model.rowCount()
            self.Qmaster.tableWidget_model.insertRow(rowPosition)
            self.Qmaster.tableWidget_model.setItem(rowPosition, 0, QTableWidgetItem(str(picked_point[0]*0.001)))
            self.Qmaster.tableWidget_model.setItem(rowPosition, 1, QTableWidgetItem(str(picked_point[1]*0.001)))
            self.Qmaster.tableWidget_model.setItem(rowPosition, 2, QTableWidgetItem(str(picked_point[2]*0.001)))

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

    def update(self):
        self.renderer.GetRenderWindow().Render()



