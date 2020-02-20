import vtk
import trimesh
import numpy as np
import time

file_name = 'implant_registration_pts.stl'
POINT_SIZE = 5

def addPoint(point_coordinate):
    # Create the geometry of a point (the coordinate)
    points = vtk.vtkPoints()
    # Create the topology of the point (a vertex)
    vertices = vtk.vtkCellArray()
    id = points.InsertNextPoint(point_coordinate)
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
    return actor


## Trimesh
t0 = time.time()
mesh = trimesh.load(file_name)
t1 = time.time()
print("time for loading: " + str(t1-t0))
t0 = time.time()
curvatures =  trimesh.curvature.discrete_gaussian_curvature_measure(mesh, mesh.vertices, radius=2) #0.01
t1 = time.time()
print("time for computing curvature: " + str(t1-t0))

centroid = mesh.centroid
print(centroid)

# print(mesh.vertex_normals)
## VTK 
# load stl model
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

# render setting
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
iren.SetRenderWindow(renWin)
ren.AddActor(actor)

# add point according to curvature
thresh = 0.25 
for i, curvature in enumerate(curvatures):
    if curvature > thresh:
        p = mesh.vertices[i].tolist()
        actor = addPoint(p)
        ren.AddActor(actor)

iren.Initialize()
iren.Start()
