import vtk
import trimesh
import numpy as np
import scipy.optimize
from sklearn.cluster import KMeans
import time

POINT_SIZE = 5

class MeshProcessing(object):
    def __init__(self, file_name):
        self.mesh = trimesh.load(file_name)

    def extractFeaturePoints(self):
        """ Function to extract vertices from a mesh by:
            1) extract points by curvature
            2) create a projection plane for mesh and project points in (1) onto plane
            3) extract points by ray casting projected points to mesh, choose points that are not first-hit,
            * Step (3) should extract feature clusters on top surface of the mesh
            4) extract representative feature points of clusters by KMeans

        Args:
            mesh: Mesh to be used for vertices and computing curvature.
            threshold: Threshold value for filtering curvature.
            radius: Radius of discrete_gaussian_cuvature_measure() # reference: ‘Restricted Delaunay triangulations and normal cycle’, Cohen-Steiner and Morvan.

        Returns:
           Vertices filtered out by curvature.

        """

        # Step 1
        print("processing curvature ...")
        t0 = time.time()
        potential_vertices = MeshProcessing.extractPointsByCurvature(self.mesh, threshold=0.01, radius=0.01) 
        t1 = time.time()
        print("time for extracting points by curvature: " + str(t1-t0))

        # Step 2
        print("creating plane and projection...")
        t0 = time.time()
        c, normal = MeshProcessing.fitPlaneLTSQ(potential_vertices)
        centroid = self.mesh.centroid

        # TODO: make following block clean later, add following block to get same result as test_implant_3d.py
        source = vtk.vtkPlaneSource()
        source.SetCenter(centroid)
        source .SetNormal(normal)
        # make plane larger
        origin = source.GetOrigin()
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
        # move plane away along normal direction
        source.Push(100)

        # vtk plane for projection
        origin = source.GetOrigin()
        plane = vtk.vtkPlane()
        plane.SetOrigin(origin) # use centroid if without above block
        plane.SetNormal(normal)
        # plane.Push(100)# translate plane along normal direction by distance 100
        # origin = plane.GetOrigin()
        projected_points = []
        for i, point in enumerate(potential_vertices):
            projected_point = np.zeros(3)
            plane.ProjectPoint(point, origin, normal, projected_point)
            projected_points.append(projected_point)
        t1 = time.time()
        print("time for creating plane and projection: " + str(t1-t0))

        # Step 3
        print("ray casting ... ")
        projected_points = np.array(projected_points)
        normals = np.repeat(-normal.reshape((1,3)), len(projected_points), axis=0)
        # check ray casting:
        t0 = time.time()

        # non-vectorization implementation of ray casting
        # desired_points = []
        # for i, projected_point in enumerate(projected_points):
        #     locations, index_ray, index_tri = self.mesh.ray.intersects_location(projected_point.reshape((1,3)), -normal.reshape((1,3)), multiple_hits=False)
        #     if len(locations) > 0 and np.linalg.norm(potential_vertices[i] - locations[0]) > 0.1:
        #         desired_points.append(potential_vertices[i].tolist())

        # vectorization implementation of ray casting
        locations, index_ray, index_tri = self.mesh.ray.intersects_location(projected_points, normals, multiple_hits=False)
        potential_vertices_raycasting_order = potential_vertices[index_ray]
        difference = np.linalg.norm(potential_vertices_raycasting_order - locations, axis=1)
        desired_points = potential_vertices_raycasting_order[np.where(difference > 0.5)] # TODO: find a threshold for thickness
        t1 = time.time()
        print("time for extracting points by ray casting: " + str(t1-t0))

        # Step 4
        print("K-Means ...")
        t0 = time.time()
        feature_point = []
        km = KMeans(n_clusters=17) # TODO:manually input number of clusters first, can try elbow
        y_km = km.fit(desired_points)
        for cluster_center in y_km.cluster_centers_:
            candidates = potential_vertices_raycasting_order - cluster_center
            candidates = np.linalg.norm(candidates, axis = 1)
            closest_pt = potential_vertices_raycasting_order[np.argmin(candidates)]
            feature_point.append(closest_pt)
        t1 = time.time()
        print("time for extracting points by K Means: " + str(t1-t0))

        return feature_point

    @staticmethod
    def extractPointsByCurvature(mesh, threshold, radius):
        """ Function to extract vertices from a mesh by curvature.

        Args:
            mesh: Mesh to be used for vertices and computing curvature.
            threshold: Threshold value for filtering curvature.
            radius: Radius of discrete_gaussian_cuvature_measure() # reference: ‘Restricted Delaunay triangulations and normal cycle’, Cohen-Steiner and Morvan.

        Returns:
           Vertices filtered out by curvature.

        """
        curvatures =  trimesh.curvature.discrete_gaussian_curvature_measure(mesh, mesh.vertices, radius=radius)
        vertices = mesh.vertices[np.where(curvatures > threshold)]
        return vertices


    @staticmethod
    def fitPlaneLTSQ(XYZ):
        """ Function to fit a plane to a set of 3D points.

        Args:
            XYZ: Array of (n,3) 3D points.

        Returns:
            plane with c and normal.

        """
        (rows, cols) = XYZ.shape
        G = np.ones((rows, 3))
        G[:, 0] = XYZ[:, 0]  #X
        G[:, 1] = XYZ[:, 1]  #Y
        Z = XYZ[:, 2]
        (a, b, c),resid,rank,s = np.linalg.lstsq(G, Z, rcond=None)
        normal = (a, b, -1)
        nn = np.linalg.norm(normal)
        normal = normal / nn
        return (c, normal)

def createPointActor(point_coordinate, color):
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

def createPlaneActor(centroid, normal, get_origin = False):

    source = vtk.vtkPlaneSource()
    source.SetCenter(centroid)
    source .SetNormal(normal)
    # make plane larger
    origin = source.GetOrigin()
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
    # move plane away along normal direction
    source.Push(100)
    origin = source.GetOrigin()
    # add plane
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    if get_origin:
        return (actor, origin)
    else:
        return actor


def createLineActor(p0, p1):
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

def main():
    RED = (255,0,0)
    file_name = '../mesh/implant_registration_pts.stl'
    mesh_processor = MeshProcessing(file_name)

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

    # visualize projection plane
    potential_vertices = MeshProcessing.extractPointsByCurvature(mesh_processor.mesh, threshold=0.01, radius=0.01) 
    c, normal = MeshProcessing.fitPlaneLTSQ(potential_vertices)
    centroid = mesh_processor.mesh.centroid
    plane_actor, origin = createPlaneActor(centroid, normal, get_origin=True)
    ren.AddActor(plane_actor)

    # vtk plane for projection
    # origin = plane_actor.GetOrigin()
    plane = vtk.vtkPlane()
    plane.SetOrigin(origin) # use centroid if without above block
    plane.SetNormal(normal)
    # plane.Push(100)# translate plane along normal direction by distance 100
    # origin = plane.GetOrigin()
    projected_points = []
    for i, point in enumerate(potential_vertices):
        projected_point = np.zeros(3)
        plane.ProjectPoint(point, origin, normal, projected_point)
        projected_points.append(projected_point)
        actor = createPointActor(projected_point, color=RED)
        ren.AddActor(actor)

    # non-vectorization implementation of ray casting
    desired_points = []
    for i, projected_point in enumerate(projected_points):
        locations, index_ray, index_tri = mesh_processor.mesh.ray.intersects_location(projected_point.reshape((1,3)), -normal.reshape((1,3)), multiple_hits=False)
        if len(locations) > 0 and np.linalg.norm(potential_vertices[i] - locations[0]) > 0.1:
            desired_points.append(potential_vertices[i].tolist())
            actor = createLineActor(projected_point, desired_points[-1])
            ren.AddActor(actor)

    feature_points = mesh_processor.extractFeaturePoints()
    # render feature points in viewer
    for point in feature_points:
        actor = createPointActor(point, color=RED)
        ren.AddActor(actor)

    ren.ResetCamera()
    # show render window
    iren.Initialize()
    iren.Start()

if __name__ == "__main__":
    main()