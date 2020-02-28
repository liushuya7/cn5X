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
            self.transformation_matrix = transformation_matrix
            return 
        else:
            print("Error: no transformation matrix provided!")

        if self.transformation_matrix is not None:
            self.transformation_matrix = np.matmul(transformation_matrix, self.transformation_matrix)
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

    def pivotCentroid(self, pivot_source, pivot_target, source_numpy):
        """ Function to make source_numpy centroided around pivot
            Function would update self.source and self.target for further registration.
            Function would update self.transformation_matrix.
        Args:
            source_numpy: numpy array of shape (Nx4).

        Returns:
           transformed source numpy array of shape (Nx4)

        """
        num_pts = len(source_numpy)
        source_numpy_homogeneous = np.concatenate((source_numpy, np.array([1.0]*num_pts).reshape((num_pts, 1))), axis=1)

        tvec = pivot_target - pivot_source
        transformation_matrix = np.concatenate((np.concatenate((np.eye(3), tvec.reshape(3,1)), axis=1), np.array([[0,0,0,1]])), axis=0)
        source_transformed_numpy = np.matmul(transformation_matrix, source_numpy_homogeneous.T) # (4xN)

        self.updateTransformationMatrix(new_transformation_matrix=transformation_matrix)

        return source_transformed_numpy.T

    def alignNormals(self, pivot, normal_source, normal_target, source_numpy):
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
        tvec = pivot.reshape((3,1)) 
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
        """ Function to perform registration by finding correspondences with Hungarian first. 
            Steps:
            1) Choose arbitrary point in source as pivot source(point 0 here), 
               iterate target points to be pivot target, transform source to target by pivot.
            2) For every iteration in (1), fit plane to transformed source and target,
                align normal of source plane to normal of target plane and transform source points.
            3) From 0 to 2*pi, rotate transformed source points in (2) around pivot point and
               along normal vector, and then transform source points.
            4) Use Hungarian to find correspondences in (3).
            5) Find the global optimal correspondence with lowest Hungarian cost from steps (1) to (4).
            6) Do landmark transform (2 sets of 3D points with known correspondences) to find the transformation
               from source to target.
        Args:
            None

        Returns:
            None

        """

        # choose arbitrary source point as pivot point
        pivot_source = self.source_numpy[0].copy()
        # fit a plane to source_centroided and self.target respectively
        c_target, normal_target = MeshProcessing.fitPlaneLTSQ(self.target_numpy)

        # iterate over self.target_numpy to find best match of pivot point
        min_cost = float('inf')
        best_match_case = 'unflipped'
        best_source_ind = None
        best_target_ind = None
        best_transformed_source = None

        for pivot_target in self.target_numpy:

            self.transformation_matrix = None
            # shift to pivot 
            source_pivot_centroided_numpy = self.pivotCentroid(pivot_source, pivot_target, source_numpy=self.source_numpy)

            # align normal, get transformed source 
            # test if align normal is needed
            c_source, normal_source = MeshProcessing.fitPlaneLTSQ(source_pivot_centroided_numpy)
            if np.dot(normal_target, normal_source) < 0.999:
                source_normaled_numpy = self.alignNormals(source_pivot_centroided_numpy[0][:3].copy(), normal_source, normal_target, source_pivot_centroided_numpy)
            else:
                source_normaled_numpy = source_pivot_centroided_numpy

            # generate flip case
            flip_axis = np.cross(normal_source, normal_target)
            flip_axis = flip_axis / np.linalg.norm(flip_axis)
            r = R.from_rotvec(np.pi * flip_axis)
            rotation_matrix_flipped = r.as_matrix()
            source_flipped_numpy, flip_transformation_matrix = Registration.applySimilarityTransform(rotation_matrix_flipped, source_normaled_numpy[0][:3].copy(), source_normaled_numpy, get_transformation=True)

            # rotate source_normaled_numpy with normal_target, find the correspondences by Hungarian cost
            # save the correspondence if the cost is lower than current minimum cost
            for angle in np.linspace(0, 2*np.pi, num = 100):
                r = R.from_rotvec(angle * normal_target)
                rotation_matrix = r.as_matrix()

                # unflipped case
                source_transformed_numpy, pivot_rotation_matrix = Registration.applySimilarityTransform(rotation_matrix, source_normaled_numpy[0][:3].copy(), source_normaled_numpy, get_transformation=True)
                cost, source_ind, target_ind = Registration.findCorrespondenceByHungarian(source_transformed_numpy[:,:3], self.target_numpy)
                if cost < min_cost:
                    min_cost = cost
                    best_match_case = 'unflipped'
                    best_source_ind = source_ind
                    best_target_ind = target_ind
                    best_transformed_source = source_transformed_numpy

                # flipped case
                source_transformed_numpy, pivot_rotation_matrix = Registration.applySimilarityTransform(rotation_matrix, source_flipped_numpy[0][:3].copy(), source_flipped_numpy, get_transformation=True)
                cost, source_ind, target_ind = Registration.findCorrespondenceByHungarian(source_transformed_numpy[:,:3], self.target_numpy)
                if cost < min_cost:
                    min_cost = cost
                    best_match_case = 'flipped'
                    best_source_ind = source_ind
                    best_target_ind = target_ind
                    best_transformed_source = source_transformed_numpy
        
        # update self variables
        self.best_match_case = best_match_case
        print("best match case: " + best_match_case)
        print("min cost from Hungarian: " +str(min_cost))
        self.best_source_ind = best_source_ind
        self.best_target_ind = best_target_ind

        # re-assign self.source and self.target to perform landmark transforma
        source = self.source_numpy[best_source_ind]
        target = self.target_numpy[best_target_ind]
        self.initializeSourceAndTarget(source, target)
        self.performLandmarkTransform()

    @staticmethod
    def computeError(source, target):
        total_dist = 0
        for i in range(len(source)):
            dist = np.linalg.norm(source[i] - target[i])
            total_dist +=dist 
            # print("source: " + str(source[i]))
            # print("target: " + str(target[i]))
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
        tvec -= np.matmul(rotation_matrix, tvec)
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


