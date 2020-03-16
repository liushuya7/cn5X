import vtk
import trimesh
import numpy as np
import scipy.optimize

import Utils
from Utils import VTKUtils

class TwoMeshesInteraction(object):
    def __init__(self, source, target):
        
        # source and target are vtk actor from stl
        self.source = source
        self.target = target

    def transformSource(self, point_target):
        """ Function to transform center of source to align the point.

        Args:
            point: 

        Returns:
            None
        """
        # get center of source polydata
        center_source = VTKUtils.getCenterOfActor(self.source)

        # change to numpy
        center_source = np.array(center_source)
        point_target = np.array(point_target)

        # geometry calculation and create transform
        translation = np.array(point_target) - np.array(center_source )
        transform = vtk.vtkTransform()
        transform.Translate(translation)
        
        self.source.SetUserTransform(transform)

    def concatenateUserTransformSource(self):
        # get final transform combined with user-defined transform 
        user_transform_source = self.source.GetMatrix()
        transform = vtk.vtkTransform()
        transform.SetMatrix(user_transform_source)
        transform_filter = VTKUtils.transformData(self.source.GetMapper().GetInput(), transform)
        return transform_filter

    def createBooleanOperationActor(self, operation='intersection'):
        transform_filter = self.concatenateUserTransformSource()
        polydata_source = transform_filter.GetOutput()
        polydata_target = self.target.GetMapper().GetInput()
        if operation == 'intersection':
            actor = self.createIntersectionActor(polydata_source, polydata_target)

        return actor

    def createIntersectionActor(self, input1, input2 ):
        """ Function to take intersection of input1 and input2.

        Args:
            input1: 
            input2: 

        Returns:
            booleanOperationActor

        """
        # boolean operation
        booleanOperation = vtk.vtkBooleanOperationPolyDataFilter()
        booleanOperation.SetOperationToIntersection()

        booleanOperation.SetInputData(0, input1)
        booleanOperation.SetInputData(1, input2)
        booleanOperation.Update()

        booleanOperationMapper = vtk.vtkPolyDataMapper()
        booleanOperationMapper.SetInputConnection(booleanOperation.GetOutputPort())
        booleanOperationMapper.ScalarVisibilityOff()

        booleanOperationActor = vtk.vtkActor()
        booleanOperationActor.SetMapper(booleanOperationMapper)
        booleanOperationActor.GetProperty().SetDiffuseColor((0,0,255))
        return booleanOperationActor

    def extractConnectedRegionActor(self, actor, extract_mode='largest', cell_id=None):
        # poly data connectivity
        connectivity = vtk.vtkPolyDataConnectivityFilter()
        connectivity.ScalarConnectivityOff()
        connectivity.SetInputData(actor.GetMapper().GetInput())
        if extract_mode == 'cell':
            if cell_id is None:
                print("Cell id should be provided")
            elif cell_id < 0:
                print("No cell is selected by picker")
            else:
                connectivity.InitializeSeedList()
                connectivity.AddSeed(cell_id)
                connectivity.SetExtractionModeToCellSeededRegions()

        elif extract_mode == 'all':
            connectivity.SetExtractionModeToAllRegions()
            connectivity.ColorRegionsOn()
        else:
            connectivity.SetExtractionModeToLargestRegion()

        # prepare for new actor 
        connectivity.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(connectivity.GetOutput())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return actor
    
def main():
    RED = (255,0,0)

    mesh_dict = {'implant':"../mesh/implant.stl", 'template':"../mesh/template.stl" }
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

    two_mesh_interaction = TwoMeshesInteraction(actors['template'], actors['implant'])
    center_target = VTKUtils.getCenterOfActor(actors['implant'])
    two_mesh_interaction.transformSource(center_target)
    intersection_actor = two_mesh_interaction.createBooleanOperationActor()
    actor_all = two_mesh_interaction.extractConnectedRegionActor(intersection_actor, extract_mode='all')
    ren.AddActor(actor_all)

    append_filter = vtk.vtkAppendPolyData()
    extracted_actor = None
    extraction_finished = None


    # for picker
    def processCellPick(object, event):
        nonlocal extraction_finished
        if extraction_finished is None:
            print("not in extraction mode")
            return
        if not extraction_finished:
            cell_id = object.GetCellId()
            if cell_id > 0:
                print(cell_id)
                actor = cell_picker.GetActor()
                actor_cc = two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)
                # # delete cell from actor
                # poly_data = actor.GetMapper().GetInput()
                # poly_data.DeleteCell(cell_id)
                # poly_data.RemoveDeletedCells()

                # append poly data
                poly_data = actor_cc.GetMapper().GetInput()
                append_filter.AddInputData(poly_data)


    def processRightButton(object, event):
        print("right button clicked")
        nonlocal extraction_finished
        if  extraction_finished is None:
            extraction_finished = False

        elif extraction_finished == False:
            extraction_finished = True
        else:
            # create extraction actor
            mapper = actor_all.GetMapper()
            mapper.SetInputConnection(append_filter.GetOutputPort())  
            # ren.RemoveActor(actor)
            # ren.AddActor(actor_cc)
            ren.GetRenderWindow().Render()
            print("Finished")

    cell_picker = vtk.vtkCellPicker()
    cell_picker.AddObserver("EndPickEvent", processCellPick)
    iren.SetPicker(cell_picker)
    iren.AddObserver("RightButtonPressEvent", processRightButton)

    # polydata_template = actors['template'].GetMapper().GetInput()
    # points_template_vtk = polydata_template.GetPoints()

    # c_template, normal_template = Utils.fitPlaneLTSQ(VTKUtils.vtkPointsToNumpyArray(points_template_vtk))
    # center_template = VTKUtils.getCenterOfActor(actors['template'])
    # plane_actor = VTKUtils.createPlaneActor(center_template, normal_template, offset=0)
    # ren.AddActor(plane_actor)

    # #create a plane to cut,here it cuts in the XZ direction (xz normal=(1,0,0);XY =(0,0,1),YZ =(0,1,0)
    # plane=vtk.vtkPlane()
    # plane.SetOrigin()
    # plane.SetNormal(1,0,0)

    # # #create cutter
    # # cutter=vtk.vtkCutter()
    # # cutter.SetCutFunction(plane)
    # # cutter.SetInputConnection(cube.GetOutputPort())
    # # cutter.Update()
    # # cutterMapper=vtk.vtkPolyDataMapper()
    # # cutterMapper.SetInputConnection( cutter.GetOutputPort())

    ren.ResetCamera()
    ren.GetRenderWindow().Render()
    # show render window
    iren.Initialize()
    iren.Start()

if __name__ == "__main__":
    main()
