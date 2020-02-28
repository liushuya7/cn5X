import os
# Registration
import vtk
import numpy as np
import csv
from MeshProcessing import MeshProcessing
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance_matrix
from scipy.spatial.transform import Rotation as R
import itertools

self_dir = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(self_dir, "../data")
DATA_FILTER = "CSV files (*.csv);;Text files (*.txt)"
RED = (255,0,0)
GREEN = (0,255,0)

class Registration():
    """Registration with VTK.

    Note that the transformation matrix of registration result from vtk is: from source to target

    In the setting of IROS paper, we set model points as target and world points as source.
    We want to transform world points to modle space so that we can visualize points in vtk.

    Properties created with the ``@property`` decorator should be documented
    in the property's getter method.

    Attributes:
        attr1 (str): Description of `attr1`.
        attr2 (:obj:`int`, optional): Description of `attr2`.

    """

    def __init__(self, source, target):

        # source_numpy and target_numpy are raw data, should not be changed during process
        self.source_numpy = np.array(source) # (Nx3)
        self.target_numpy = np.array(target) # (Nx3)
        self.transformation_matrix = None # can be cascaded during operation, (4x4) numpy array

        # self.source and self.target for registration operation can be updated during process,
        # therefore, they might differ from source_numpy and target_numpy
        self.initializeSourceAndTarget(source, target)

    def initializeSourceAndTarget(self, source, target):
        """ Function to initialize source and target for vtk registration.

        Args:
            source: can be list or numpy array of shape (N,3)
            target: can be list or numpy array of shape (N,3)

        Returns:
           None

        """
        # create source vtkpolydata        
        sourcePoints = vtk.vtkPoints()
        sourceVertices = vtk.vtkCellArray()
        for point in source:
            id = sourcePoints.InsertNextPoint(point[0], point[1], point[2])
            sourceVertices.InsertNextCell(1)
            sourceVertices.InsertCellPoint(id)
        
        self.source = vtk.vtkPolyData()
        self.source.SetPoints(sourcePoints)
        self.source.SetVerts(sourceVertices)
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.source.Update()

        # create target vtkpolydata        
        targetPoints = vtk.vtkPoints()
        targetVertices = vtk.vtkCellArray()
        for point in target:
            id = targetPoints.InsertNextPoint(point[0], point[1], point[2])
            targetVertices.InsertNextCell(1)
            targetVertices.InsertCellPoint(id)
        
        self.target = vtk.vtkPolyData()
        self.target.SetPoints(targetPoints)
        self.target.SetVerts(targetVertices)
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.target.Update()

    def updateTransformationMatrix(self, new_transformation_matrix=None, vtk_transformation=None):
        """ Function to self.transformation_matrix.

        Args:

        Returns:
           None

        """
        if new_transformation_matrix is not None:
            transformation_matrix = new_transformation_matrix

        elif vtk_transformation is not None:
            transformation_matrix = Registration.getTransformationMatrixFromVtk(vtk_transformation.GetMatrix())

        else:
            print("Error: no transformation matrix provided!")

        if self.transformation_matrix is not None:
            self.transformation_matrix = np.matmul(self.transformation_matrix, transformation_matrix)
        else:
            self.transformation_matrix = transformation_matrix

    def alignCentroids(self):
        """ Function to align centroids of source and target. 
            Function would update self.source and self.target for further registration.
            Function would update self.transformation_matrix.
        Args:
            None

        Returns:
           transformed source numpy array of shape (Nx4)

        """

        # use vtk ICP startByMatchingCentroids to align centroids, self.transformation_matrix will be updated in performICP()
        self.performICP()
        # transform target points
        num_pts = len(self.target_numpy)
        source_centroided_numpy = self.source_numpy.copy()
        source_centroided_numpy = np.concatenate((source_centroided_numpy, np.array([1.0]*num_pts).reshape((num_pts, 1))), axis=1)
        source_centroided_numpy = np.matmul(self.transformation_matrix, source_centroided_numpy.T) # (4xN)

        # re-initialize registration
        self.initializeSourceAndTarget(source_centroided_numpy.T[:,:3], self.target_numpy)

        return source_centroided_numpy.T

    def alignNormals(self, normal_source, normal_target, source_numpy):
        """ Function to align normal of source plane and target plane. 
            Function would update self.source and self.target for further registration.
            Function would update self.transformation_matrix.
        Args:
           normal_source: normal vector of source plane.
           normal_target: normal vector of target plane.
           source_numpy: numpy array of shape (Nx4).

        Returns:
           transformed source numpy array of shape (Nx4)

        """

        # debug: check axis direction
        axis = np.cross(normal_source, normal_target)
        axis = axis/np.linalg.norm(axis)
        angle = np.arccos(np.dot(normal_source, normal_target)/(np.linalg.norm(normal_source)*np.linalg.norm(normal_target)))
        print("axis")
        print(axis)
        print("angle")
        print(angle)
        rotation = R.from_rotvec(axis*angle)
        rotation_matrix = rotation.as_matrix()
        tvec = np.mean(source_numpy, axis=0)[:3]
        source_transformed_numpy, transformation_matrix = Registration.applySimilarityTransform(rotation_matrix, tvec, source_numpy, True)
        
        self.updateTransformationMatrix(new_transformation_matrix=transformation_matrix)

        # re-initialize registration
        self.initializeSourceAndTarget(source_transformed_numpy, self.target_numpy)

        return source_transformed_numpy

    def performICP(self):
        
        # create icp object
        icp = vtk.vtkIterativeClosestPointTransform()
        icp.SetSource(self.source)
        icp.SetTarget(self.target)
        icp.GetLandmarkTransform().SetModeToRigidBody()
        
        # parameters for icp
        #icp.DebugOn()
        # icp.SetMaximumNumberOfLandmarks(100)
        # icp.SetMaximumMeanDistance(0.0001)
        # icp.SetMaximumNumberOfIterations(100)
        icp.StartByMatchingCentroidsOn()
        icp.CheckMeanDistanceOn()
        icp.Modified()
        icp.Update()

        # icpTransformFilter = vtk.vtkTransformPolyDataFilter()
        # if vtk.VTK_MAJOR_VERSION <= 5:
        #     icpTransformFilter.SetInput(self.source)
        # else:
        #     icpTransformFilter.SetInputData(self.source)

        # icpTransformFilter.SetTransform(icp)
        # icpTransformFilter.Update()

        # transformedSource = icpTransformFilter.GetOutput()
        # # ============ display transformed points ==============
        # for index in range(6):
        #     point = [0,0,0]
        #     transformedSource.GetPoint(index, point)
        #     print("transformed source point[%s]=%s" % (index,point))
        self.updateTransformationMatrix(vtk_transformation=icp)

    def performLandmarkTransform(self):

        # create landmark transform object
        lm_transform = vtk.vtkLandmarkTransform()
        lm_transform.SetSourceLandmarks(self.source.GetPoints())
        lm_transform.SetTargetLandmarks(self.target.GetPoints())
        lm_transform.SetModeToRigidBody()
        if vtk.VTK_MAJOR_VERSION <= 5:
            lm_transform.Update()

        self.updateTransformationMatrix(vtk_transformation=lm_transform)
        
    def performRegistrationByHungarian(self):

        print("target numpy")
        print(self.target_numpy)
        # align centroids, get transformed source
        source_centroided_numpy = self.alignCentroids()

        # fit a plane to source_centroided and self.target respectively
        c_target, normal_target = MeshProcessing.fitPlaneLTSQ(self.target_numpy)
        c_source, normal_source = MeshProcessing.fitPlaneLTSQ(source_centroided_numpy)

        # align normal, get transformed source 
        source_normaled_numpy = self.alignNormals(normal_source, normal_target, source_centroided_numpy)

        centroid = np.mean(source_normaled_numpy, axis=0)[:3]
        flip_axis = np.cross(normal_source, normal_target)
        r = R.from_rotvec(np.pi * flip_axis)
        rotation_matrix_flipped = r.as_matrix()
        source_flipped_numpy, flip_transformation_matrix = Registration.applySimilarityTransform(rotation_matrix_flipped, centroid, source_normaled_numpy, get_transformation=True)
        print("source flipped numpy")
        print(source_flipped_numpy[:,:3])

        # rotate source_normaled_numpy with normal_target, find the one with smallest error
        min_cost = float('inf')
        best_match_case = 'unflipped'
        best_source_ind = None
        best_target_ind = None

        for angle in np.linspace(0, 2*np.pi, num = 100):
            r = R.from_rotvec(angle * normal_target)
            rotation_matrix = r.as_matrix()

            # unflipped case
            source_transformed_numpy = Registration.applySimilarityTransform(rotation_matrix, centroid, source_normaled_numpy)
            cost, source_ind, target_ind = Registration.findCorrespondenceByHungarian(source_transformed_numpy[:,:3], self.target_numpy)
            print("min cost")
            print(min_cost)
            if cost < min_cost:
                min_cost = cost
                best_match_case = 'unflipped'
                best_source_ind = source_ind
                best_target_ind = target_ind

            # flipped case
            source_transformed_numpy = Registration.applySimilarityTransform(rotation_matrix, centroid, source_flipped_numpy)
            cost, source_ind, target_ind = Registration.findCorrespondenceByHungarian(source_transformed_numpy[:,:3], self.target_numpy)
            if cost < min_cost:
                min_cost = cost
                best_match_case = 'flipped'
                best_source_ind = source_ind
                best_target_ind = target_ind
        
        # update self variables
        self.best_match_case = best_match_case
        print("best match case: " + best_match_case)
        self.best_source_ind = best_source_ind
        self.best_target_ind = best_target_ind

        if best_match_case == 'unflipped':
            source = source_normaled_numpy[best_source_ind]
        else:
            # update self.transformation_matrix by flip
            source = source_flipped_numpy[best_source_ind]
            self.updateTransformationMatrix(new_transformation_matrix=flip_transformation_matrix)
        target = self.target_numpy[best_target_ind]

        self.initializeSourceAndTarget(source, target)
        self.performLandmarkTransform()

    @staticmethod
    def computeError(source, target):
        total_dist = 0
        for i in range(len(source)):
            dist = np.linalg.norm(source[i] - target[i])
            total_dist +=dist 
            print("source: " + str(source[i]))
            print("target: " + str(target[i]))
        avg_dist = total_dist/len(source)

        return avg_dist
    
    @staticmethod
    def applySimilarityTransform(rotation_matrix, tvec, source_numpy, get_transformation=False):
        """ Function to apply similarity transformation on source_numpy given rotation matrix and tvec . 
        Args:
           rotation_matrix: numpy array of shape (3x3).
           tvec: numpy array of shape (3,) or (1,3) or (3,1).
           source_numpy: numpy array of shape (Nx4).
           get_transformation: flag to see if the funtion returns the transformation matrix

        Returns:
           transformed source numpy array of shape (Nx4)

        """
        
        tvec = tvec.reshape((3,1))
        tvec = np.matmul(rotation_matrix, tvec) - tvec
        transformation = np.concatenate((rotation_matrix, tvec), axis=1)
        last_row = [[0, 0, 0, 1]]
        transformation = np.concatenate((transformation, last_row), axis=0)
        source_transformed_numpy = np.matmul(transformation, source_numpy.T)

        if get_transformation:
            return (source_transformed_numpy.T, transformation)
        else:
            return source_transformed_numpy.T


    @staticmethod
    def findCorrespondenceByHungarian(source_numpy, target_numpy):
        """ Function to find correspondence of source and target by Hungarian algorithm. 
        Args:
           source_numpy: numpy array of shape (Nx3).
           source_numpy: numpy array of shape (Nx3).

        Returns:
            cost:
            row_ind:
            col_ind:

        """
        cost_matrix = distance_matrix(source_numpy, target_numpy)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        cost = cost_matrix[row_ind, col_ind].sum()
        
        return (cost, row_ind, col_ind)
        
    @staticmethod
    def getTransformationMatrixFromVtk(transformation_matrix):
        # get numpy array from vtk matrix
        transformation_mat = np.ones((4,4))
        for i in range(4):
            for j in range(4):
                transformation_mat[i,j] = transformation_matrix.GetElement(i,j)

        return transformation_mat 


