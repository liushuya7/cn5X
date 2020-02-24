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
        plane = vtk.vtkPlane()
        plane.SetOrigin(centroid)
        plane.SetNormal(normal)
        plane.Push(100)# translate plane along normal direction by distance 100
        origin = plane.GetOrigin()
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
        locations, index_ray, index_tri = self.mesh.ray.intersects_location(projected_points, normals, multiple_hits=False)
        potential_vertices_raycasting_order = potential_vertices[index_ray]
        difference = np.linalg.norm(potential_vertices_raycasting_order - locations, axis=1)
        desired_points = potential_vertices_raycasting_order[np.where(difference > 0.1)]
        t1 = time.time()
        print("time for extracting points by ray casting: " + str(t1-t0))

        # Step 4
        print("K-Means ...")
        t0 = time.time()
        feature_point = []
        km = KMeans(n_clusters=17) # debug, manually input number of clusters first, can try elbow
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