# -*- coding: UTF-8 -*-

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                         '
' Copyright 2018-2019 Gauthier Brière (gauthier.briere "at" gmail.com)    '
'                                                                         '
' This file is part of cn5X++                                             '
'                                                                         '
' cn5X++ is free software: you can redistribute it and/or modify it       '
' under the terms of the GNU General Public License as published by       '
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
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QSpinBox, QDoubleSpinBox, QLineEdit
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic
from cn5X_config import *
from grblCom import grblCom
from msgbox import *
from compilOptions import grblCompilOptions

self_dir = os.path.dirname(os.path.realpath(__file__))

class cn5XAbout(QObject):
  ''' About dialog box '''
  def __init__(self, versionString: str, licenceFile: str):
    super().__init__()
    self.__dlgAbout = QDialog()
    ui_dlgAbout = os.path.join(self_dir, '../ui/dlgAbout.ui')
    self.__di = uic.loadUi(ui_dlgAbout, self.__dlgAbout)
    self.__di.lblVersion.setText(versionString)

    text=open(licenceFile).read()
    self.__di.qptLicence.setPlainText(text)

  def showDialog(self):
    # Centrage de la boite de dialogue sur la fenetre principale
    ParentX = self.parent().geometry().x()
    ParentY = self.parent().geometry().y()
    ParentWidth = self.parent().geometry().width()
    ParentHeight = self.parent().geometry().height()
    myWidth = self.__dlgAbout.geometry().width()
    myHeight = self.__dlgAbout.geometry().height()
    self.__dlgAbout.setFixedSize(self.__dlgAbout.geometry().width(),self.__dlgAbout.geometry().height())
    self.__dlgAbout.move(ParentX + ((ParentWidth - myWidth) / 2),ParentY + ((ParentHeight - myHeight) / 2),)
    self.__dlgAbout.setWindowFlags(Qt.Dialog)

    RC = self.__dlgAbout.exec()
    return(RC)

