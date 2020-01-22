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

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QModelIndex, QItemSelectionModel
from PyQt5.QtGui import QKeySequence, QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QListView
from cn5X_config import *
from msgbox import *
from grblCom import grblCom


class gcodeFile(QObject):
  '''
   Management of a GCode file in the QListView of the graphical interface reserved for this use
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
    # Affiche la boite de dialogue d'ouverture
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getOpenFileName(None, self.tr("Ouvrir un fichier GCode"), "", self.tr("Fichier GCode (*.gcode *.ngc *.nc *.gc *.cnc)"), options=opt)
    return fileName

  def readFile(self, filePath: str):
    self.sig_log.emit(logSeverity.info.value, self.tr("Lecture du fichier : {}").format(filePath))
    try:
      f = open(filePath,'r')
      lignes  = f.readlines()
      f.close()
      self.sig_log.emit(logSeverity.info.value, self.tr("{} lignes dans le fichier").format(len(lignes)))
      # Envoi du contenu dans la liste
      self.__gcodeFileUiModel.clear()
      for l in lignes:
        item = QStandardItem(l.strip())
        self.__gcodeFileUiModel.appendRow(item)
      self.__gcodeFileUi.setModel(self.__gcodeFileUiModel)
      # Selectionne la premiere ligne du fichier dans la liste
      self.selectGCodeFileLine(0)
      # Selectionne l'onglet du fichier
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, self.tr("Erreur lecture du fichier : {}").format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
      self.__gcodeFileUiModel.clear()
      self.__filePath     = ""
      self.__gcodeChanged = False
      return False

    # Pas d'erreur
    self.__gcodeCharge = True
    self.__filePath     = filePath
    self.__gcodeChanged = False
    return True

  def isFileLoaded(self):
    return self.__gcodeCharge


  def filePath(self):
    return self.__filePath


  def selectGCodeFileLine(self, num: int):
    ''' Selectionne un element de la liste du fichier GCode '''
    idx = self.__gcodeFileUiModel.index(num, 0, QModelIndex())
    self.__gcodeFileUi.selectionModel().clearSelection()
    self.__gcodeFileUi.selectionModel().setCurrentIndex(idx, QItemSelectionModel.SelectCurrent)


  def getGCodeSelectedLine(self):
    ''' Renvoie le N° (0 base) de la ligne selectionnee dans la liste GCode et les donnees de cette ligne. '''
    idx = self.__gcodeFileUi.selectionModel().selectedIndexes()
    return [idx[0].row(), self.__gcodeFileUiModel.data(idx[0])]


  def saveAs(self):
    fileName = self.showFileSave()
    if fileName[0] != "":
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs({})").format(fileName[0]))
      self.saveFile(fileName[0])
    else:
      self.sig_log.emit(logSeverity.info.value, self.tr("saveAs() annule !"))


  def showFileSave(self):
    ''' Affiche la boite de dialogue "Save as" '''
    opt = QtWidgets.QFileDialog.Options()
    opt |= QtWidgets.QFileDialog.DontUseNativeDialog
    fileName = QtWidgets.QFileDialog.getSaveFileName(None, self.tr("Enregistrer un fichier GCode"), "", self.tr("Fichier GCode (*.gcode *.ngc *.nc *.gc *.cnc)"), options=opt)
    return fileName


  def saveFile(self, filePath: str = ""):
    if filePath == "":
      if self.__filePath == "":
        # Le nom du fichier n'est pas definit, il n'y a pas de fichier charge, donc, rien a sauvegarder !
        return
      else:
        filePath = self.__filePath
    self.sig_log.emit(logSeverity.info.value, self.tr("Enregistrement du fichier : {}").format(filePath))
    try:
      f = open(filePath, 'w')
      for I in range(self.__gcodeFileUiModel.rowCount()):
        idx = self.__gcodeFileUiModel.index( I, 0, QModelIndex())
        if self.__gcodeFileUiModel.data(idx) != "":
          f.write(self.__gcodeFileUiModel.data(idx) + '\n')
      f.close()
      self.__filePath = filePath
    except Exception as e:
      self.sig_log.emit(logSeverity.error.value, self.tr("Erreur Enregistrement du fichier : {}").format(filePath))
      self.sig_log.emit(logSeverity.error.value, str(e))
    # Supprime les lignes vides dans la grille d'affichage
    self.delEmptyRow()
    # Reinit du flag fichier change
    self.__gcodeChanged = False


  def enQueue(self, com: grblCom, startLine: int = 0, endLine: int = -1):
    """ Envoi des lignes de startLine a endLine dans la file d'attente du grblCom """
    if endLine == -1:
      endLine = self.__gcodeFileUiModel.rowCount()
    for I in range(startLine, endLine + 1):
      idx = self.__gcodeFileUiModel.index( I, 0, QModelIndex())
      if self.__gcodeFileUiModel.data(idx) != "":
        com.gcodePush(self.__gcodeFileUiModel.data(idx))
        com.gcodePush(CMD_GRBL_GET_GCODE_STATE, COM_FLAG_NO_OK)


  def delEmptyRow(self):
    """ Elimination des lignes GCode vides """
    for I in reversed(range(self.__gcodeFileUiModel.rowCount())):
      # On commence par la fin pour pouvoir supprimer sans tout decaler pour la suite
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
                  title     = self.tr("Enregistrer les modifications"),
                  text      = self.tr("Voulez-vous enregistrer les modifications avant de fermer ?"),
                  info      = self.tr("Si vous n'enregistrez pas, toutes les modifications effectuees depuis l'ouverture ou la derniere sauvegarde seront perdues."),
                  icon      = msgIconList.Question,
                  stdButton = msgButtonList.Save | msgButtonList.Cancel | msgButtonList.Discard,
                  defButton = msgButtonList.Save,
                  escButton = msgButtonList.Cancel
    )
    return(m.afficheMsg())


  def closeFile(self):
    if self.__gcodeChanged:
      # GCode modifie, on demande confirmation
      Ret = self.showConfirmChangeLost()
      if Ret == msgButtonList.Save:
        if self.__filePath == "":
          filePath = self.showFileSave()
          if filePath == "":
            # Annulation de la boite de dialogue
            return False
          else:
            self.__filePath = filePath
        self.saveFile(self.__filePath)
        return True
      elif Ret == msgButtonList.Discard:
        # Fermer le fichier consiste en vider la fenetre GCode
        self.__gcodeFileUiModel.clear()
        self.__gcodeChanged = False
        self.__gcodeCharge  =False
        return True
      else: # Ret == msgButtonList.Cancel:
        return False
    else:
      # GCode non modifie, on ferme sans confirmation
      # Fermer le fichier consiste en vider la fenetre GCode
      # et a supprimer le status GCode charge.
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

