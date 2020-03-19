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
        action_select_active = QAction("Select Active", self)

        action_target.triggered.connect(self.setTarget)
        action_source.triggered.connect(self.setSource)
        action_extract_cell.triggered.connect(self.extractCell)
        action_finish_extraction.triggered.connect(self.finishExtraction)
        action_delete_cell.triggered.connect(self.deleteCell)
        action_select_active.triggered.connect(self.selectActive)

        # self.addSection("Start")
        self.addAction(action_target)
        self.addAction(action_source)
        # self.addSection("Clean Fragments")
        self.addAction(action_extract_cell)
        self.addAction(action_finish_extraction)
        self.addAction(action_delete_cell)
        # self.addSection("Path Generation")
        self.addAction(action_select_active)
        
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
                print(text_info)

        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)

    def initializeExtraction(self):
        self.actor_to_be_extracted = None

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
                print(text_info)
        else:
            text_info = "Select source and target in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)

    def selectActive(self):

        if isinstance(self.parent.interactor.GetInteractorStyle(), ActorInteractor):
            if self.parent.interactor.GetInteractorStyle().picked_actor:
                picked_actor = self.parent.interactor.GetInteractorStyle().picked_actor
                # check state
                if self.parent.path_generation_dialog.state == PathState['EXTRACT']:
                    self.actor_to_be_extracted = picked_actor 
                    self.append_filter = vtk.vtkAppendPolyData()
                    text_info = "Start Extraction: Right click on the viewer to select cell for its connected region"
                    self.parent.path_generation_dialog.label_status.setText(text_info)
                    print(text_info)

                elif self.parent.path_generation_dialog.state == PathState['EXECUTE']:
                    for name, mesh_actor in self.parent.mesh_actors.items():
                        if picked_actor is mesh_actor:
                            self.parent.path_generation_dialog.putVolumeMeshName(name)
            else:
                text_info = "Please select on an actor."
                self.parent.path_generation_dialog.label_status.setText(text_info)

        else:
            text_info = "Select volume mesh in wrong mode. \n Check VTK mode to be actor."
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)
    
    def finishExtraction(self):
        # create extraction actor
        mapper = self.actor_to_be_extracted.GetMapper()
        mapper.SetInputConnection(self.append_filter.GetOutputPort())  
        self.parent.update()

        self.parent.path_generation_dialog.state = PathState['EXECUTE']
        self.setEnableDisableGroupActions(PathState['EXECUTE'])

        text_info = "Finished Extraction. Turn to EXECUTE state"
        self.parent.path_generation_dialog.label_status.setText(text_info)
        print(text_info)

    def extractCell(self):

        if not self.actor_to_be_extracted:
            text_info = "No actor selected to be extracted yet!"
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)
            return 
        if self.parent.path_generation_dialog.state != PathState['EXTRACT']:
            text_info = "Not in extraction mode!"
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)
            return

        clickPos = self.parent.interactor.GetEventPosition()
        cell_picker = vtk.vtkCellPicker()
        cell_picker.Pick(clickPos[0], clickPos[1], 0, self.parent.renderer)
        cell_id = cell_picker.GetCellId()
        if cell_id > 0:

            actor = cell_picker.GetActor()
            actor_cc = self.parent.two_mesh_interaction.extractConnectedRegionActor(actor, extract_mode='cell', cell_id=cell_id)

            # append poly data
            poly_data = actor_cc.GetMapper().GetInput()
            self.append_filter.AddInputData(poly_data)

            text_info = "Cell " + str(cell_id) + " and its connected region is selected!"
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)

        else:
            text_info = "No cell picked"
            self.parent.path_generation_dialog.label_status.setText(text_info)
            print(text_info)

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


    def setEnableDisableGroupActions(self, state):

        if state == PathState['LOAD_FILES']:
            # print(dir(self.action_menu))
            self.actions()[0].setEnabled(True)
            self.actions()[1].setEnabled(True)
            self.actions()[2].setEnabled(False)
            self.actions()[3].setEnabled(False)
            self.actions()[4].setEnabled(False)
            self.actions()[5].setEnabled(False)

        elif state == PathState['EXTRACT']:
            self.actions()[0].setEnabled(False)
            self.actions()[1].setEnabled(False)
            self.actions()[2].setEnabled(True)
            self.actions()[3].setEnabled(True)
            self.actions()[4].setEnabled(True)
            self.actions()[5].setEnabled(True)

        elif state == PathState['EXECUTE']:
            self.actions()[0].setEnabled(False)
            self.actions()[1].setEnabled(False)
            self.actions()[2].setEnabled(False)
            self.actions()[3].setEnabled(False)
            self.actions()[4].setEnabled(False)
            self.actions()[5].setEnabled(True)
        else:
            self.actions()[0].setEnabled(False)
            self.actions()[1].setEnabled(False)
            self.actions()[2].setEnabled(False)
            self.actions()[3].setEnabled(False)
            self.actions()[4].setEnabled(False)
            self.actions()[5].setEnabled(False)