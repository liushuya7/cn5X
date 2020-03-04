import vtk
import trimesh
import numpy as np
import scipy.optimize
# from scipy.spatial.transform import Rotation as R
import time
# import pymesh

from Utils import VTKUtils
from MeshProcessing import MeshProcessing

class MultiMeshesInteraction(object):
    def __init__(self, file_name_dic):
        
        self.meshes = {}
        for name, path in file_name_dic.items():
            self.meshes[name] = trimesh.load(path)
            # print(self.meshes[name].bounds)

    def takeIntersection(self):
        

    
    
def main():
    RED = (255,0,0)
    mesh_dict = {'implant':'../mesh/implant.stl', 'template':'../mesh/RNS_Cutter_No_Rounds.stl'}
    meshes_processor = MultiMeshesInteraction(mesh_dict)

    # render setting
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    iren.SetRenderWindow(renWin)

    # load STL
    actors = {}
    for name, path in mesh_dict.items():
        actor = VTKUtils.loadSTL(path)
        actors[name] = actor
        # ren.AddActor(actor)
    
    # transform template to align its centroid to the centroid of implant,
    # fit plane to both meshes,
    # then transform template to align its normal to the normal of implant
    centroid_implant = meshes_processor.meshes['implant'].centroid
    centroid_template = meshes_processor.meshes['template'].centroid
    translation = centroid_implant - centroid_template
    c_implant, normal_implant = MeshProcessing.fitPlaneLTSQ(meshes_processor.meshes['implant'].vertices)
    c_template, normal_template = MeshProcessing.fitPlaneLTSQ(meshes_processor.meshes['template'].vertices)
    axis = np.cross(normal_template, normal_implant)
    axis = axis/np.linalg.norm(axis)
    angle = np.arccos(np.dot(normal_template, normal_implant)/(np.linalg.norm(normal_template)*np.linalg.norm(normal_implant)))
    transform = vtk.vtkTransform()
    transform.RotateWXYZ(angle, -axis)
    transform.Translate(translation)
    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputData(actors['template'].GetMapper().GetInput())
    transform_filter.SetTransform(transform)
    transform_filter.Update()
    # print(actors['template'].GetMapper().GetInput())

    # VTKUtils.transformActor(actors['template'], transform)
    # print(actors['implant'].GetMapper().GetInput())

    # boolean operation
    booleanOperation = vtk.vtkBooleanOperationPolyDataFilter()
    booleanOperation.SetOperationToIntersection()
    # booleanOperation.SetOperationToDifference()

    booleanOperation.SetInputData(0, actors['implant'].GetMapper().GetInput())
    booleanOperation.SetInputData(1, transform_filter.GetOutput())
    # booleanOperation.SetInputConnection(1, transform_filter.GetOutputPort())
    booleanOperation.Update()

    booleanOperationMapper = vtk.vtkPolyDataMapper()
    booleanOperationMapper.SetInputConnection(booleanOperation.GetOutputPort())
    booleanOperationMapper.ScalarVisibilityOff()

    booleanOperationActor = vtk.vtkActor()
    booleanOperationActor.SetMapper(booleanOperationMapper)
    # booleanOperationActor.GetProperty().SetDiffuseColor(colors.GetColor3d("Banana"))
    ren.AddActor(booleanOperationActor)

    ren.ResetCamera()
    # ren.GetRenderWindow().Render()
    # show render window
    iren.Initialize()
    iren.Start()


if __name__ == "__main__":
    main()
