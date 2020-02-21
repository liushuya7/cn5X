import vtk
import trimesh
import numpy as np
import scipy.optimize
from sklearn.cluster import KMeans
import time

POINT_SIZE = 5

class MeshProcessing(object):
    def __init__(self, file_name):
        mesh = trimesh.load(file_name)
        curvatures =  trimesh.curvature.discrete_gaussian_curvature_measure(mesh, mesh.vertices, radius=0.01) #radius=2


    def extractFeaturePoints(self):
        pass

    @staticmethod
    def extractPointsByCurvature(mesh, threshold):


    @staticmethod
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

    

