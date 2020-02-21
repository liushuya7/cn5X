import vtk
import trimesh
import numpy as np
import scipy.optimize
from sklearn.cluster import KMeans
import time

file_name = 'implant_registration_pts.stl'
POINT_SIZE = 5

def addPoint(point_coordinate, color):
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
    actor.GetProperty().SetColor(color)
    actor.GetProperty().SetPointSize(POINT_SIZE)
    return actor

def addLine(p0, p1):
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

    return actor

def fitPlaneLTSQ(XYZ):
    (rows, cols) = XYZ.shape
    G = np.ones((rows, 3))
    G[:, 0] = XYZ[:, 0]  #X
    G[:, 1] = XYZ[:, 1]  #Y
    Z = XYZ[:, 2]
    (a, b, c),resid,rank,s = np.linalg.lstsq(G, Z)
    normal = (a, b, -1)
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return (c, normal)

## Trimesh
t0 = time.time()
mesh = trimesh.load(file_name)
t1 = time.time()
print("time for loading: " + str(t1-t0))
t0 = time.time()
curvatures =  trimesh.curvature.discrete_gaussian_curvature_measure(mesh, mesh.vertices, radius=0.01) #radius=2
t1 = time.time()
print("time for computing curvature: " + str(t1-t0))


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
potential_points = []
thresh_max = 0.011
thresh_min = 0.01
print(np.min(curvatures))
print(np.max(curvatures))

for i, curvature in enumerate(curvatures):
    if curvature > thresh_min:
    # if curvature == 0:
        p = mesh.vertices[i].tolist()
        potential_points.append(p)
        # actor = addPoint(p)
        # ren.AddActor(actor)

potential_points = np.array(potential_points)
print("num of potential points")
print(len(potential_points))

# plot fitting plane
c, normal = fitPlaneLTSQ(potential_points)
centroid = mesh.centroid
print("center")
print(centroid)
# actor = addPoint(centroid)
# ren.AddActor(actor)
source = vtk.vtkPlaneSource()
source.SetCenter(centroid)
source .SetNormal(normal)
# make plane larger
origin = source.GetOrigin()
print("origin")
print(origin)
vec_origin_to_center = centroid - np.array(list(origin))
point1 = source.GetPoint1()
vec1 = np.array(list(point1)) - origin 
point2 = source.GetPoint2()
vec2 = np.array(list(point2)) - origin 
origin = origin - vec_origin_to_center * 200
point1 = origin + 200 * vec1
point2 = origin + 200 * vec2
source.SetOrigin(origin)
source.SetPoint1(point1)
source.SetPoint2(point2)
source.Push(100)
# add plane
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(source.GetOutputPort())
actor = vtk.vtkActor()
actor.SetMapper(mapper)
ren.AddActor(actor)

# project potential points onto plane
plane = vtk.vtkPlane()
plane.SetOrigin(origin)
plane.SetNormal(normal)
origin = source.GetOrigin()
thresh_normal = 0 
count = 0
desired_points = []
for i, point in enumerate(potential_points):
    # # check normal vector:
    # if np.dot(mesh.vertex_normals[i], normal) < thresh_normal:
    projected_point = np.zeros(3)
    plane.ProjectPoint(point, origin, normal, projected_point)
    # check ray casting:
    locations, index_ray, index_tri = mesh.ray.intersects_location(projected_point.reshape((1,3)), -normal.reshape((1,3)), multiple_hits=False)
    if len(locations) > 0 and np.linalg.norm(point-locations[0]) > 0.1:
        count += 1
        desired_points.append(point.tolist())
        # actor_point = addPoint(point, color=(1,0,0))
        # actor_proj_point = addPoint(projected_point)
        # actor_line = addLine(point, projected_point)
        # ren.AddActor(actor_point)
        # ren.AddActor(actor_proj_point)
        # ren.AddActor(actor_line)
print("number of point:")
print(count)

# KMeans for clustering
desired_points = np.array(desired_points)
km = KMeans(n_clusters=17)
y_km = km.fit(desired_points)
print(y_km.cluster_centers_)

for cluster_center in y_km.cluster_centers_:
    candidates = potential_points - cluster_center
    print(candidates.shape)
    candidates = np.linalg.norm(candidates, axis = 1)
    print(candidates.shape)
    closest_pt = potential_points[np.argmin(candidates)]
    actor_point = addPoint(closest_pt, color=(1,0,0) )
    ren.AddActor(actor_point)



# show render window
iren.Initialize()
iren.Start()
