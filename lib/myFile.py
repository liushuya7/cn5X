# -*- coding: UTF-8 -*-

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                         '
' Copyright 2018 Gauthier Brière (gauthier.briere "at" gmail.com)         '
'                                                                         '
' This file is part of cn5X++                                               '
'                                                                         '
' cn5X++ is free software: you can redistribute it and/or modify it         '
'  under the terms of the GNU General Public License as published by      '
' the Free Software Foundation, either version 3 of the License, or       '
' (at your option) any later version.                                     '
'                                                                         '
' cn5X++ is distributed in the hope that it will be useful, but           '
' WITHOUT ANY WARRANTY; without even the implied warranty of              '
' MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           '
' GNU General Public License for more details.                            '
'                                                                         '
' You should have received a copy of the GNU General Public License       '
' along with this program.  If not, see <http://www.gnu.org/licenses/>.   '
'                                                                         '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QModelIndex, QItemSelectionModel
from PyQt5.QtGui import QKeySequence, QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QListView
from cn5X_config import *
from msgbox import *
from grblCom import grblCom
from Viewer import Viewer

self_dir = os.path.dirname(os.path.realpath(__file__))
MESH_PATH = os.path.join(self_dir, '../mesh')


class gcodeFile(QObject):
  '''
   Management of a GCode file in the QListView of the GUI
   Methods :
   - __init __ (QListView) -> Initializes and defines the elements of the UI which will receive the content of the file
   - showFileOpen () -> Displays the opening dialog box
   - showFileSave () -> Displays the recording dialog
   - readFile (filePath) -> Loads a file into the QListView
   - saveFile (filePath) -> Save the contents of the QListView in a file
   - closeFile () -> Empty the QListView
   - setGcodeChanged (bool) -> Defines if the content of the list has been modified since the reading or saving of the file
   - bool = gcodeChanged () -> Return true if the content of the list has been modified since the file was read or saved
  '''

  sig_log     = pyqtSignal(int, str) # Component operation message

  def __init__(self, gcodeFileUi: QListView):
    super().__init__()
    self.__filePath         = ""
    self.__gcodeFileUi      = gcodeFileUi
    self.__gcodeFileUiModel = QStandardItemModel(self.__gcodeFileUi)
    self.__gcodeFileUiModel.itemChanged.connect(self.on_gcodeChanged)
    self.__gcodeCharge      = False
    self.__gcodeChanged     = False

  def showFileOpen(self):
    # Displays the opening dialog
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getOpenFileName(None, "Open a GCode file", "" ,"Open GCode (*.gcode *.ngc *.nc *.gc *.cnc)", options=opt)
    return fileName

  def readFile(self, filePath: str):
    self.sig_log.emit(logSeverity.info.value, "Read the Gcode file : {}".format(filePath))
    try:
      f = open(filePath,'r')
      lignes  = f.readlines()
      f.close()
      self.sig_log.emit(logSeverity.info.value, "{} lines in the file".format(len(lignes)))
      # Sending content to the list
      self.__gcodeFileUiModel.clear()
      for l in lignes:
        item = QStandardItem(l.strip())
        self.__gcodeFileUiModel.appendRow(item)
      self.__gcodeFileUi.setModel(self.__gcodeFileUiModel)
      # Select the first line of the file in the list
      self.selectGCodeFileLine(0)
      # Select the file tab
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, "File read error : {}".format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
      self.__gcodeFileUiModel.clear()
      self.__filePath     = ""
      self.__gcodeChanged = False
      return False

    # Everything is good
    self.__gcodeCharge = True
    self.__filePath     = filePath
    self.__gcodeChanged = False
    return True

  def isFileLoaded(self):
    return self.__gcodeCharge

  def filePath(self):
    return self.__filePath


  def selectGCodeFileLine(self, num: int):
    ''' Select an item from the list of GCode file '''
    idx = self.__gcodeFileUiModel.index(num, 0, QModelIndex())
    self.__gcodeFileUi.selectionModel().clearSelection()
    self.__gcodeFileUi.selectionModel().setCurrentIndex(idx, QItemSelectionModel.SelectCurrent)

  def getGCodeSelectedLine(self):
    ''' Returns the #(0 base) of the line selected in the GCode list and the data of this line. '''
    idx = self.__gcodeFileUi.selectionModel().selectedIndexes()
    return [idx[0].row(), self.__gcodeFileUiModel.data(idx[0])]


  def saveAs(self):
    fileName = self.showFileSave()
    if fileName[0] != "":
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs({})").format(fileName[0]))
      self.saveFile(fileName[0])
    else:
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs() canceled !"))


  def showFileSave(self):
    ''' Displays the "Save as" dialog '''
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getSaveFileName(None, "Save a GCode file", "", "Save GCode (*.gcode *.ngc *.nc *.gc *.cnc)", options=opt)
    return fileName


  def saveFile(self, filePath: str = ""):
    if filePath == "":
      if self.__filePath == "":
        self.sig_log.emit(logSeverity.info.value, "The name of the file is not defined, there is no file loaded, therefore, nothing to save!")
        return
      else:
        filePath = self.__filePath
    self.sig_log.emit(logSeverity.info.value, "Save file: {}".format(filePath))
    try:
      f = open(filePath, 'w')
      for I in range(self.__gcodeFileUiModel.rowCount()):
        idx = self.__gcodeFileUiModel.index( I, 0, QModelIndex())
        if self.__gcodeFileUiModel.data(idx) != "":
          f.write(self.__gcodeFileUiModel.data(idx) + '\n')
      f.close()
      self.__filePath = filePath
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, self.tr("Error saving file: {}").format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
    # Delete blank lines in the display grid
    self.delEmptyRow()
    # Reinstates flag
    self.__gcodeChanged = False


  def enQueue(self, com: grblCom, startLine: int = 0, endLine: int = -1):
    """ Sending lines from startLine to endLine in the grblCom queue """
    if endLine == -1:
      endLine = self.__gcodeFileUiModel.rowCount()
    for I in range(startLine, endLine + 1):
      idx = self.__gcodeFileUiModel.index( I, 0, QModelIndex())
      if self.__gcodeFileUiModel.data(idx) != "":
        com.gcodePush(self.__gcodeFileUiModel.data(idx))
        com.gcodePush(CMD_GRBL_GET_GCODE_STATE, COM_FLAG_NO_OK)


  def delEmptyRow(self):
    """ Eliminate empty GCode lines """
    for I in reversed(range(self.__gcodeFileUiModel.rowCount())):
      # We start at the end to be able to delete without shifting everything for the rest
      idx = self.__gcodeFileUiModel.index( I, 0, QModelIndex())
      if self.__gcodeFileUiModel.data(idx) == "":
        self.__gcodeFileUiModel.removeRow(I)


  def deleteGCodeFileLine(self, num: int):
    self.__gcodeFileUiModel.removeRow(num)
    self.__gcodeChanged = True


  def insertGCodeFileLine(self, num: int):
    item = QStandardItem("")
    self.__gcodeFileUiModel.insertRow(num, item)


  def addGCodeFileLine(self, num: int):
    item = QStandardItem("")
    self.__gcodeFileUiModel.insertRow(num+1, item)


  def showConfirmChangeLost(self):
    m = msgBox(
                  title     = "Save Changes",
                  text      = "Do you want to save the changes before closing?",
                  info      = "If you do not save, all changes made since last opening of the file will be lost.",
                  icon      = msgIconList.Question,
                  stdButton = msgButtonList.Save | msgButtonList.Cancel | msgButtonList.Discard,
                  defButton = msgButtonList.Save,
                  escButton = msgButtonList.Cancel
    )
    return(m.afficheMsg())


  def closeFile(self):
    if self.__gcodeChanged:
      # GCode changes, we ask for confirmation
      Ret = self.showConfirmChangeLost()
      if Ret == msgButtonList.Save:
        if self.__filePath == "":
          filePath = self.showFileSave()
          if filePath == "":
            # Canceling the dialog box
            return False
          else:
            self.__filePath = filePath
        self.saveFile(self.__filePath)
        return True
      elif Ret == msgButtonList.Discard:
        # Closing the file consists of emptying the GCode window
        self.__gcodeFileUiModel.clear()
        self.__gcodeChanged = False
        self.__gcodeCharge  =False
        return True
      else: # Ret == msgButtonList.Cancel:
        return False
    else:
      # GCode not modified, we close without confirmation
      # Fermer le fichier consiste en vider la fenetre GCode
      # and delete the GCode charge status.
      self.__gcodeFileUiModel.clear()
      self.__gcodeChanged = False
      self.__gcodeCharge  =False
      return True


  @pyqtSlot("QStandardItem*")
  def on_gcodeChanged(self, item):
    self.__gcodeChanged = True


  def gcodeChanged(self):
    return self.__gcodeChanged


  def setGcodeChanged(self, value:bool):
    self.__gcodeChanged = value


'''--------------------------------- STL File Class -------------------------------------------------'''

class stlFile(QObject):
  '''
   Management of a STL file in VTK Viewer
   Methods :
   - __init __ (Viewer) -> Initializes and defines the elements of the UI which will receive the content of the file
   - showFileOpen () -> Displays the opening dialog box
   - showFileSave () -> Displays the recording dialog
   - readFile (filePath) -> Loads a file into the VTK Viewer
   - saveFile (filePath) -> Save the contents of the VTK Viewer in a file
   - closeFile () -> Clear the VTK Viewer
   - setStlTransformed (bool) -> Defines if the content of the list has been modified since the reading or saving of the file
   - bool = modelTransformed () -> Return true if the content of the list has been modified since the file was read or saved
  '''

  sig_log     = pyqtSignal(int, str)

  def __init__(self, stlRender: Viewer):
    super().__init__()
    self.__filePath         = ""
    self.__stlRender      = stlRender
    self.__stlCharge      = False
    self.__modelTransformed     = False

  def showFileOpen(self):
    # Displays the opening dialog
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getOpenFileName(None, "Open a STL file", MESH_PATH ,"STL (*.stl)", options=opt)
    return fileName

  def readFile(self, filePath: str):
    self.sig_log.emit(logSeverity.info.value, "Read the STL file : {}".format(filePath))
    try:
      # Load the model into VTK render
      self.__stlRender.loadModel(filePath)
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, "STL File read error : {}".format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
      self.__filePath     = ""
      self.__modelTransformed = False
      return False

    # Everything is good
    self.__stlCharge = True
    self.__filePath     = filePath
    self.__modelTransformed = False
    return True

  def isFileLoaded(self):
    return self.__stlCharge

  def filePath(self):
    return self.__filePath

  def saveAs(self):
    fileName = self.showFileSave()
    if fileName[0] != "":
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs({})").format(fileName[0]))
      self.saveFile(fileName[0])
    else:
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs() canceled !"))

  def showFileSave(self):
    ''' Displays the "Save as" dialog '''
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getSaveFileName(None, "Save a GCode file", "", "Save GCode (*.gcode *.ngc *.nc *.gc *.cnc)", options=opt)
    return fileName

  def saveFile(self, filePath: str = ""):
    if filePath == "":
      if self.__filePath == "":
        self.sig_log.emit(logSeverity.info.value, "The name of the file is not defined, there is no file loaded, therefore, nothing to save!")
        return
      else:
        filePath = self.__filePath
    self.sig_log.emit(logSeverity.info.value, "Save file: {}".format(filePath))
    try:
      f = open(filePath, 'w')
      for I in range(self.__stlRenderModel.rowCount()):
        idx = self.__stlRenderModel.index( I, 0, QModelIndex())
        if self.__stlRenderModel.data(idx) != "":
          f.write(self.__stlRenderModel.data(idx) + '\n')
      f.close()
      self.__filePath = filePath
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, self.tr("Error saving file: {}").format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
    # Delete blank lines in the display grid
    self.delEmptyRow()
    # Reinstates flag
    self.__modelTransformed = False

  def showConfirmChangeLost(self):
    m = msgBox(
                  title     = "Save Changes",
                  text      = "Do you want to save the changes before closing?",
                  info      = "If you do not save, all changes made since last opening of the file will be lost.",
                  icon      = msgIconList.Question,
                  stdButton = msgButtonList.Save | msgButtonList.Cancel | msgButtonList.Discard,
                  defButton = msgButtonList.Save,
                  escButton = msgButtonList.Cancel
    )
    return(m.afficheMsg())

  def closeFile(self):
    if self.__modelTransformed:
      # GCode changes, we ask for confirmation
      Ret = self.showConfirmChangeLost()
      if Ret == msgButtonList.Save:
        if self.__filePath == "":
          filePath = self.showFileSave()
          if filePath == "":
            # Canceling the dialog box
            return False
          else:
            self.__filePath = filePath
        self.saveFile(self.__filePath)
        return True
      elif Ret == msgButtonList.Discard:
        # Closing the file consists of emptying the GCode window
        self.__stlRenderModel.clear()
        self.__modelTransformed = False
        self.__stlCharge  =False
        return True
      else: # Ret == msgButtonList.Cancel:
        return False
    else:
      # GCode not modified, we close without confirmation
      # Fermer le fichier consiste en vider la fenetre GCode
      # and delete the GCode charge status.
      self.__stlRenderModel.clear()
      self.__modelTransformed = False
      self.__stlCharge  =False
      return True


  @pyqtSlot("QStandardItem*")
  def on_modelTransformed(self, item):
    self.__modelTransformed = True

  def modelTransformed(self):
    return self.__modelTransformed

  def setStlTransformed(self, value:bool):
    self.__modelTransformed = value