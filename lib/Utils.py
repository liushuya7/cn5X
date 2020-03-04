import vtk
import trimesh
import numpy as np

POINT_SIZE = 5

class VTKUtils(object):
    def __init__(self):
        pass

    @staticmethod
    def loadSTL(file_name):
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

        return actor

    @staticmethod
    def createPointActor(point_coordinate, color, point_size=POINT_SIZE):
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
        actor.GetProperty().SetPointSize(point_size)

        return actor

    @staticmethod
    def createPlaneActor(centroid, normal, offset=100, size=200, get_origin = False):

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
        origin = origin - vec_origin_to_center * size
        point1 = origin + size * vec1
        point2 = origin + size * vec2
        source.SetOrigin(origin)
        source.SetPoint1(point1)
        source.SetPoint2(point2)
        # move plane away along normal direction
        source.Push(offset)
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


    @staticmethod
    def createLineActor(p0, p1, color=(0,255,0)):
        # Create a vtkPoints object and store the points in it
        pts = vtk.vtkPoints()
        pts.InsertNextPoint(p0)
        pts.InsertNextPoint(p1)

        # Setup two colors - one for each line

        # Setup the colors array
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")

        # Add the colors we created to the colors array
        colors.InsertNextTypedTuple(color)

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

    @staticmethod
    def transformActor(actor, vtk_transform):

        actor.SetUserTransform(vtk_transform)

def main():
    pass

if __name__ == "__main__":
    main()