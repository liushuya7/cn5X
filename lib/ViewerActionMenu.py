from PyQt5.QtWidgets import QFrame, QHBoxLayout, QTableWidgetItem, QMenu, QAction
from PyQt5.QtCore import Qt
from PathState import PathState
from ActorInteractor import ActorInteractor
import vtk

class ViewerActionMenu(QMenu):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        action_target = QAction("Target", self)
        action_source = QAction("Source", self)
        action_extract_cell = QAction("Extract Cell", self)
        action_finish_extraction = QAction("Finish Extraction", self)
        action_delete_cell = QAction("Delete Cell", self)
        action_set_volume_mesh = QAction("Set Volume Mesh", self)

        action_target.triggered.connect(self.setTarget)
        action_source.triggered.connect(self.setSource)
        action_extract_cell.triggered.connect(self.extractCell)
        action_finish_extraction.triggered.connect(self.finishExtraction)
        action_delete_cell.triggered.connect(self.deleteCell)
        action_set_volume_mesh.triggered.connect(self.setVolumeMesh)

        # self.addSection("Start")
        self.addAction(action_target)
        self.addAction(action_source)
        # self.addSection("Clean Fragments")
        self.addAction(action_extract_cell)
        self.addAction(action_finish_extraction)
        self.addAction(action_delete_cell)
        # self.addSection("Path Generation")
        self.addAction(action_set_volume_mesh)
        
    def setEnableExtraction(self):
        self.setEnabled()
    
    def setTarget(self):
        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                for name, mesh_actor in self.parent.mesh_actors.items():
                    if picked_actor is mesh_actor:
                        self.parent.path_generation_dialog.putTargetName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)

        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)

    def setSource(self):
        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                for name, mesh_actor in self.parent.mesh_actors.items():
                    if picked_actor is mesh_actor:
                        self.parent.path_generation_dialog.putSourceName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)
        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)

    def startExtraction(self):
        clickPos = self.parent.interactor.GetEventPosition()
        cell_picker = vtk.vtkCellPicker()
        cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        cell_id = cell_picker.GetCellId()
        if cell_id > 0:
            self.actor_to_be_extracted = cell_picker.GetActor()
            self.parent.extraction_finished = False
            self.append_filter = vtk.vtkAppendPolyData()
            text_info = "Start Extraction: Right click on the viewer to select cell for its connected region"
            self.parent.path_generation_dialog.label_status.setText(text_info)
        else:
            print("No actor is selected, extraction is not started yet.")

    def finishExtraction(self):
        # reset extraction mode
        self.parent.extraction_finished = None
        # create extraction actor
        mapper = self.actor_to_be_extracted.GetMapper()
        mapper.SetInputConnection(self.append_filter.GetOutputPort())  
        self.parent.update()
        text_info = "Start Extraction: Right click on the viewer to select cell for its connected region"
        self.parent.path_generation_dialog.label_status.setText(text_info)
        self.parent.state = PathState['EXECUTE']
        self.parent.setEnableDisableGroupActions()

    def extractCell(self):
        clickPos = self.parent.interactor.GetEventPosition()
        cell_picker = vtk.vtkCellPicker()
        cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        cell_id = cell_picker.GetCellId()
        if self.parent.extraction_finished is None:
            print("not in extraction mode")
            return
        if not self.parent.extraction_finished:
            cell_id = cell_picker.GetCellId()
            if cell_id > 0:
                print(cell_id)
                actor = cell_picker.GetActor()
                actor_cc = self.parent.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)

                # append poly data
                poly_data = actor_cc.GetMapper().GetInput()
                self.append_filter.AddInputData(poly_data)
                text_info = "Start Extraction: Right click on the viewer to select cell for its connected region"
                self.parent.path_generation_dialog.label_status.setText(text_info)

            else:
                print("No cell picked")

    def deleteCell(self):
        pass
        # clickPos = self.parent.interactor.GetEventPosition()
        # cell_picker = vtk.vtkCellPicker()
        # cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        # cell_id = cell_picker.GetCellId()
        # if self.parent.extraction_finished is None:
        #     print("not in extraction mode")
        #     return
        # if not self.parent.extraction_finished:
        #     cell_id = object.GetCellId()
        #     if cell_id > 0:
        #         print(cell_id)
        #         actor = cell_picker.GetActor()
        #         actor_cc = self.parent.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)
        #         # # delete cell from actor
        #         # poly_data = actor.GetMapper().GetInput()
        #         # poly_data.DeleteCell(cell_id)
        #         # poly_data.RemoveDeletedCells()

        #         # append poly data
        #         poly_data = actor_cc.GetMapper().GetInput()
        #         append_filter.AddInputData(poly_data)

    def setVolumeMesh(self):
        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                for name, mesh_actor in self.parent.mesh_actors.items():
                    if picked_actor is mesh_actor:
                        self.parent.path_generation_dialog.putVolumeMeshName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)

        else:
            text_info = "Select volume mesh in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)