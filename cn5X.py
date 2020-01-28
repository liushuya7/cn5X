#! /usr/bin/env python3
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

import sys, os, time
import argparse

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import Qt, QCoreApplication, QObject, QThread, pyqtSignal, pyqtSlot, QModelIndex,  QItemSelectionModel, QFileInfo, QTranslator, QLocale, QSettings
from PyQt5.QtGui import QKeySequence, QStandardItemModel, QStandardItem, QValidator
from PyQt5.QtWidgets import QDialog, QAbstractItemView, QHBoxLayout
from PyQt5.QtSerialPort import QSerialPortInfo
from cn5X_config import *
from msgbox import *
from speedOverrides import *
from grblCom import grblCom
from grblDecode import grblDecode
from gcodeQLineEdit import gcodeQLineEdit
from cnQPushButton import cnQPushButton
from grblJog import grblJog
from myFile import gcodeFile, stlFile
from grblConfig import grblConfig
from cn5Xabout import cn5XAbout
from xml.dom.minidom import parse, Node, Element

import cn5X_rc

from Viewer import Viewer

self_dir = os.path.dirname(os.path.realpath(__file__))

class upperCaseValidator(QValidator):
  def validate(self, string, pos):
    return QValidator.Acceptable, string.upper(), pos

class GrblMainwindow(QtWidgets.QMainWindow):

  def __init__(self, parent=None):
    QtWidgets.QMainWindow.__init__(self, parent)

    self.upper_case = upperCaseValidator(self)
    self.__gcodes_stack = []
    self.__gcodes_stack_pos = -1
    self.__gcode_recall_flag = False
    self.__gcode_current_txt = ""

    # QSettings class provides persistent platform-independent application settings
    self.settings = QSettings(QSettings.NativeFormat, QSettings.UserScope, ORG_NAME, APP_NAME)

    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", action="store_true", help="Connect to serial port")
    parser.add_argument("-f", "--file", help="Open a GCode file")
    parser.add_argument("-p", "--port", help="Select serial port")
    parser.add_argument("-u", "--noEStop", action="store_true", help="Unlock the E-STOP")
    self.__args = parser.parse_args()

    # Find the license file in the same directory as the executable
    if getattr(sys, 'frozen', False):
        # frozen
        dir_ = os.path.dirname(sys.executable)
    else:
        # unfrozen
        dir_ = os.path.dirname(os.path.realpath(__file__))
    self.__licenceFile = "{}/COPYING".format(dir_)

    # Initialize the main window
    ui_mainwindow = os.path.join(self_dir, 'mainWindow.ui')
    self.ui = uic.loadUi(ui_mainwindow, self)

    # connect frmVtk with viewer
    self.vtk_viewer = Viewer(self.ui.frmVtk, self)
    self.vtk_layout = QHBoxLayout()
    self.vtk_layout.addWidget(self.vtk_viewer)
    self.vtk_layout.setContentsMargins(0,0,0,0)
    self.ui.frmVtk.setLayout(self.vtk_layout)
    self.vtk_viewer.start()

    # Emergency button picture location
    self.btnEmergencyPictureLocation = ":/cn5X/images/btnEmergency.svg"
    self.btnEmergencyOffPictureLocation = ":/cn5X/images/btnEmergencyOff.svg"
    self.timerDblClick = QtCore.QTimer() # double click timer for activating Emergency Stop

    self.logGrbl  = self.ui.txtGrblOutput    # All Grbl messages will be redirected to the txtGrblOutput widget
    self.logCn5X  = self.ui.txtConsoleOutput # All application messages will be redirected to the txtConsoleOutput widget
    self.logDebug = self.ui.txtDebugOutput   # Debug message widget

    self.logGrbl.document().setMaximumBlockCount(2000)  # Limits the size of logs to 2000 lines
    self.logCn5X.document().setMaximumBlockCount(2000)
    self.logDebug.document().setMaximumBlockCount(2000)
    self.ui.grpConsole.setCurrentIndex(2)  # active the 3rd tab

    self.__stlFile = stlFile(self.vtk_viewer)
    self.__gcodeFile = gcodeFile(self.ui.gcodeTable)
    self.__gcodeFile.sig_log.connect(self.on_sig_log)

    self.__grblCom = grblCom()
    self.__grblCom.sig_log.connect(self.on_sig_log)
    self.__grblCom.sig_connect.connect(self.on_sig_connect)
    self.__grblCom.sig_init.connect(self.on_sig_init)
    self.__grblCom.sig_ok.connect(self.on_sig_ok)
    self.__grblCom.sig_error.connect(self.on_sig_error)
    self.__grblCom.sig_alarm.connect(self.on_sig_alarm)
    self.__grblCom.sig_status.connect(self.on_sig_status)
    self.__grblCom.sig_config.connect(self.on_sig_config)
    self.__grblCom.sig_data.connect(self.on_sig_data)
    self.__grblCom.sig_emit.connect(self.on_sig_emit)
    self.__grblCom.sig_recu.connect(self.on_sig_recu)
    self.__grblCom.sig_debug.connect(self.on_sig_debug)

    self.decode = grblDecode(self.ui, self.log, self.__grblCom)

    self.__jog = grblJog(self.__grblCom)
    self.ui.dsbJogSpeed.setValue(DEFAULT_JOG_SPEED)
    self.ui.dsbJogSpeed.valueChanged.connect(self.on_dsbJogSpeed_valueChanged)

    self.__connectionStatus = False
    self.__eStopON          = True
    self.__cycleRun         = False
    self.__cyclePause       = False
    self.__grblConfigLoaded = False
    self.__nbAxis           = DEFAULT_NB_AXIS
    self.__axisNames        = DEFAULT_AXIS_NAMES
    self.updateAxisNumber()
    self.__maxTravel        = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.__firstGetSettings = False
    self.__jogModContinue   = False


    QtGui.QFontDatabase.addApplicationFont(":/cn5X/fonts/LEDCalculator.ttf")  # Police type "LED"
    self.ui.btnConnect.setText("Connect")
    self.populatePortList()                                                   # On rempli la liste des ports serie

    app.setStyleSheet("QToolTip { background-color: rgb(248, 255, 192); color: rgb(0, 0, 63); }")

    curIndex = -1                                                             # On rempli la liste des vitesses
    for v in QSerialPortInfo.standardBaudRates():
      self.ui.cmbBauds.addItem(str(v))
      curIndex += 1
      if v == COM_DEFAULT_BAUD_RATE:
        self.ui.cmbBauds.setCurrentIndex(curIndex)

    # on affiche une chaine vide texte en bas de la fenetre (status bar)
    self.__statusText = ""
    self.ui.statusBar.showMessage(self.__statusText)

    # Positionne l'etat d'activation des controles
    self.setEnableDisableGroups()

    """---------- GUI event connections ----------"""
    self.ui.btnEmergency.pressed.connect(self.on_setEmergency)       # Emergency stop button events
    self.ui.cmbPort.currentIndexChanged.connect(self.on_cmbPort_changed) # A click on the item of the list will call the method 'on_cmbPort_changed'

    self.ui.mnuBar.hovered.connect(self.on_mnuBar)     # Application menu connections
    self.ui.mnuAppOpenStl.triggered.connect(self.on_mnuAppOpenStl)
    self.ui.mnuAppOpenGcode.triggered.connect(self.on_mnuAppOpenGcode)
    self.ui.mnuAppSaveGcode.triggered.connect(self.on_mnuAppSaveGcode)
    self.ui.mnuAppSaveGcodeAs.triggered.connect(self.on_mnuAppSaveGcodeAs)
    self.ui.mnuAppCloseGcode.triggered.connect(self.on_mnuAppCloseGcode)
    self.ui.mnuAppQuit.triggered.connect(self.on_mnuAppQuit)

    self.ui.mnu_GrblConfig.triggered.connect(self.on_mnu_GrblConfig)
    self.ui.mnu_MPos.triggered.connect(self.on_mnu_MPos)
    self.ui.mnu_WPos.triggered.connect(self.on_mnu_WPos)
    self.ui.mnuDebug_mode.triggered.connect(self.on_mnuDebug_mode)

    self.ui.mnuA_propos.triggered.connect(self.on_mnuA_propos)

    self.ui.btnRefresh.clicked.connect(self.populatePortList)            # Refresh serial ports list
    self.ui.btnConnect.clicked.connect(self.action_btnConnect)           # A click on the button "(Un)Connect" will call the method 'action_btnConnect'
    self.ui.btnSend.pressed.connect(self.sendCmd)                        # Button for sending cmds
    self.ui.txtGCode.setValidator(self.upper_case)                       # Force all GCodes in capital letters
    self.ui.txtGCode.returnPressed.connect(self.sendCmd)                 # Same function by the enter key as the send button
    self.ui.txtGCode.textChanged.connect(self.txtGCode_on_Change)        # Analyze the input field after being edited
    self.ui.txtGCode.keyPressed.connect(self.on_keyPressed)
    self.ui.btnDebug.clicked.connect(self.on_btnDebug)
    self.ui.btnPausePolling.clicked.connect(self.on_btnPausePolling)

    self.ui.btnClearDebug.clicked.connect(self.clearDebug)
    self.ui.btnSpinM3.clicked.connect(self.on_btnSpinM3)
    self.ui.btnSpinM4.clicked.connect(self.on_btnSpinM4)
    self.ui.btnSpinM5.clicked.connect(self.on_btnSpinM5)

    self.ui.lblG54.clicked.connect(self.on_lblG5xClick)
    self.ui.lblG55.clicked.connect(self.on_lblG5xClick)
    self.ui.lblG56.clicked.connect(self.on_lblG5xClick)
    self.ui.lblG57.clicked.connect(self.on_lblG5xClick)
    self.ui.lblG58.clicked.connect(self.on_lblG5xClick)
    self.ui.lblG59.clicked.connect(self.on_lblG5xClick)

    # Jogging buttons
    self.ui.btnJogMoinsX.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusX.mousePress.connect(self.on_jog)
    self.ui.btnJogMoinsY.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusY.mousePress.connect(self.on_jog)
    self.ui.btnJogMoinsZ.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusZ.mousePress.connect(self.on_jog)
    self.ui.btnJogMoinsA.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusA.mousePress.connect(self.on_jog)
    self.ui.btnJogMoinsB.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusB.mousePress.connect(self.on_jog)
    self.ui.btnJogMoinsC.mousePress.connect(self.on_jog)
    self.ui.btnJogPlusC.mousePress.connect(self.on_jog)

    self.ui.btnJogMoinsX.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusX.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogMoinsY.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusY.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogMoinsZ.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusZ.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogMoinsA.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusA.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogMoinsB.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusB.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogMoinsC.mouseRelease.connect(self.stop_jog)
    self.ui.btnJogPlusC.mouseRelease.connect(self.stop_jog)

    self.ui.btnJogStop.mousePress.connect(self.__jog.jogCancel)
    self.ui.rbRapid025.clicked.connect(lambda: self.__grblCom.realTimePush(REAL_TIME_RAPID_25_POURCENT))
    self.ui.rbRapid050.clicked.connect(lambda: self.__grblCom.realTimePush(REAL_TIME_RAPID_50_POURCENT))
    self.ui.rbRapid100.clicked.connect(lambda: self.__grblCom.realTimePush(REAL_TIME_RAPID_100_POURCENT))
    self.ui.dialAvance.valueChanged.connect(self.on_feedOverride)
    self.ui.dialBroche.valueChanged.connect(self.on_spindleOverride)
    self.ui.btnLinkOverride.clicked.connect(self.on_btnLinkOverride)
    self.ui.btnResetAvance.clicked.connect(self.on_btnResetAvance)
    self.ui.btnResetBroche.clicked.connect(self.on_btnResetBroche)
    self.ui.btnKillAlarm.clicked.connect(self.on_btnKillAlarm)
    self.ui.btnHomeCycle.clicked.connect(self.on_btnHomeCycle)
    self.ui.btnReset.clicked.connect(self.on_btnReset)
    self.ui.btnStart.clicked.connect(self.startCycle)
    self.ui.btnPause.clicked.connect(self.pauseCycle)
    self.ui.btnStop.clicked.connect(self.stopCycle)
    self.ui.gcodeTable.customContextMenuRequested.connect(self.on_gcodeTableContextMenu)
    self.ui.dialAvance.customContextMenuRequested.connect(self.on_dialAvanceContextMenu)
    self.ui.dialBroche.customContextMenuRequested.connect(self.on_dialBrocheContextMenu)
    self.ui.label_grblPos_1.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(0))
    self.ui.label_grblPos_2.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(1))
    self.ui.label_grblPos_3.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(2))
    self.ui.label_grblPos_4.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(3))
    self.ui.label_grblPos_5.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(4))
    self.ui.label_grblPos_6.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(5))
    self.ui.grblPos_1.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(0))
    self.ui.grblPos_2.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(1))
    self.ui.grblPos_3.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(2))
    self.ui.grblPos_4.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(3))
    self.ui.grblPos_5.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(4))
    self.ui.grblPos_6.customContextMenuRequested.connect(lambda: self.on_grblPos_6ontextMenu(5))
    self.ui.lblPlan.customContextMenuRequested.connect(self.on_lblPlanContextMenu)
    self.ui.lblUnites.customContextMenuRequested.connect(self.on_lblUnitesContextMenu)
    self.ui.lblCoord.customContextMenuRequested.connect(self.on_lblCoordContextMenu)
    self.ui.lblG54.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(1))
    self.ui.lblG55.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(2))
    self.ui.lblG56.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(3))
    self.ui.lblG57.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(4))
    self.ui.lblG58.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(5))
    self.ui.lblG59.customContextMenuRequested.connect(lambda: self.on_lblGXXContextMenu(6))

    # Path Generation
    self.ui.pushButton_delete_point.clicked.connect(self.deletePoint)
    self.ui.pushButton_show_axis.clicked.connect(self.showAxis)
    self.ui.pushButton_cut_vector.clicked.connect(self.find_cut_vector)


    #--------------------------------------------------------------------------------------
    # Parse arguments from the command line
    #--------------------------------------------------------------------------------------
    if self.__args.connect:
      # Connection du port serie
      self.action_btnConnect()

    if self.__args.file != None:
      # Charge le fichier GCode a l'ouverture
      # Curseur sablier
      self.setCursor(Qt.WaitCursor)
      RC = self.__gcodeFile.readFile(self.__args.file)
      if RC:
        # Selectionne l'onglet du fichier sauf en cas de debug actif
        if not self.ui.btnDebug.isChecked():
          self.ui.grpConsole.setCurrentIndex(1)
        self.setWindowTitle(APP_NAME + " - " + self.__gcodeFile.filePath())
      else:
        # Selectionne l'onglet de la console pour que le message d'erreur s'affiche sauf en cas de debug
        if not self.ui.btnDebug.isChecked():
          self.ui.grpConsole.setCurrentIndex(2)
      # Restore le curseur de souris
      self.setCursor(Qt.ArrowCursor)

    if self.__args.noEStop:
      self.__eStopON = False
      self.log(logSeverity.info.value, "Emergency stop unlocked.")

    # Initializes the state of activation of the controls depending on the selection of the serial port or not
    self.setEnableDisableConnectControls()
    # Enables or disables the cycle buttons
    self.setEnableDisableGroups()

  def populatePortList(self):
    ''' Rempli la liste des ports serie '''
    self.ui.cmbPort.clear()
    self.ui.cmbPort.addItem("")
    if len(QSerialPortInfo.availablePorts()) > 0:
      for p in QSerialPortInfo.availablePorts():
        self.ui.cmbPort.addItem(p.portName() + ' - ' + p.description())
        if self.__args.port != None:
          if self.__args.port == p.portName() or self.__args.port == p.systemLocation():
            self.ui.cmbPort.setCurrentIndex(len(self.ui.cmbPort)-1)
    else:
      m = msgBox(
                  title  = "Attention !",
                  text   = "No communication port available!",
                  info   = "{} could not find a serial port to communicate with grbl.".format(sys.argv[0]),
                  icon   = msgIconList.Information,
                  detail = "\n class serialCom:\n The call to serial.tools.list_ports.comports()\n did not return any port.",
                  stdButton = msgButtonList.Close
                )
      m.afficheMsg()
    # S'il n'y a qu'un seul port serie et que l'on a rien precise comme option port, on le selectionne
    if self.__args.port == None:
      if len(QSerialPortInfo.availablePorts()) == 1:
        self.ui.cmbPort.setCurrentIndex(1)
    # Definit l'activation des controles en fonction de la selection du port serie ou non
    self.setEnableDisableConnectControls()


  def setEnableDisableConnectControls(self):
    '''
    Activate or deactivate connection checks according to port connection and selection status
    '''
    if self.__connectionStatus:
      self.ui.cmbPort.setEnabled(False)
      self.ui.cmbBauds.setEnabled(False)
      self.ui.btnConnect.setEnabled(True)
    else:
      self.ui.cmbPort.setEnabled(True)
      if self.ui.cmbPort.currentText() == "":
        self.ui.cmbBauds.setEnabled(False)
        self.ui.btnConnect.setEnabled(False)
      else:
        self.ui.cmbBauds.setEnabled(True)
        self.ui.btnConnect.setEnabled(True)


  def setEnableDisableGroups(self):
    '''
    Determines the Enable / Disable state of the different control groups
    depending on the connection state and the state of the emergency stop button.
    '''
    if not self.__connectionStatus:
      # Not connected, everything must be deactivated and the emergency stop pressed
      self.ui.btnEmergency.setIcon(QtGui.QIcon(self.btnEmergencyOffPictureLocation))
      self.ui.btnEmergency.setToolTip("Double click to \n to unlock the emergency stop")
      self.ui.frmEmergency.setEnabled(False)
      self.ui.frmControlSpeed.setEnabled(False)
      self.ui.grpJog.setEnabled(False)
      self.ui.frmGcodeInput.setEnabled(False)
      self.ui.frmButtons.setEnabled(False)
      self.ui.grpStatus.setEnabled(False)
      self.ui.frmHomeAlarm.setEnabled(False)
    elif self.__eStopON:
      # Connect but under emergency stop: Everything is deactivated except emergency stop
      self.ui.btnEmergency.setIcon(QtGui.QIcon(self.btnEmergencyOffPictureLocation))
      self.ui.btnEmergency.setToolTip("Double click to \n to unlock the emergency stop")
      self.ui.frmEmergency.setEnabled(True)
      self.ui.frmControlSpeed.setEnabled(False)
      self.ui.grpJog.setEnabled(False)
      self.ui.frmGcodeInput.setEnabled(False)
      self.ui.frmButtons.setEnabled(False)
      self.ui.grpStatus.setEnabled(False)
      self.ui.frmHomeAlarm.setEnabled(False)
    else:
      # Everything is in order, we activate everything
      self.ui.btnEmergency.setIcon(QtGui.QIcon(self.btnEmergencyPictureLocation))
      self.ui.btnEmergency.setToolTip("Emergency Stop")
      self.ui.frmEmergency.setEnabled(True)
      self.ui.frmControlSpeed.setEnabled(True)
      self.ui.grpJog.setEnabled(True)
      self.ui.frmGcodeInput.setEnabled(True)
      self.ui.frmButtons.setEnabled(True)
      self.ui.grpStatus.setEnabled(True)
      self.ui.frmHomeAlarm.setEnabled(True)
      if self.__gcodeFile.isFileLoaded():
        self.ui.frmCycle.setEnabled(True)
      else:
        self.ui.frmCycle.setEnabled(False)


  @pyqtSlot()
  def on_mnuBar(self):
    if self.__gcodeFile.isFileLoaded():
      self.ui.mnuAppSaveGcodeAs.setEnabled(True)
      self.ui.mnuAppCloseGcode.setEnabled(True)
      if self.__gcodeFile.gcodeChanged():
        self.ui.mnuAppSaveGcode.setEnabled(True)
      else:
        self.ui.mnuAppSaveGcode.setEnabled(False)
    else:
      self.ui.mnuAppSaveGcode.setEnabled(False)
      self.ui.mnuAppSaveGcodeAs.setEnabled(False)
      self.ui.mnuAppCloseGcode.setEnabled(False)
    if self.__connectionStatus:
      self.ui.mnu_MPos.setEnabled(True)
      self.ui.mnu_WPos.setEnabled(True)
      if self.__eStopON:
        self.ui.mnu_GrblConfig.setEnabled(True)
      else:
        self.ui.mnu_GrblConfig.setEnabled(False)
    else:
      self.ui.mnu_MPos.setEnabled(False)
      self.ui.mnu_WPos.setEnabled(False)
      self.ui.mnu_GrblConfig.setEnabled(False)

  @pyqtSlot()
  def on_mnuAppOpenStl(self):
    # Displays the opening dialog
    fileName = self.__stlFile.showFileOpen()
    if fileName[0] != "":
      self.setCursor(Qt.WaitCursor)
      RC = self.__stlFile.readFile(fileName[0])
      if RC:
        self.setWindowTitle(APP_NAME + " - " + self.__stlFile.filePath())
      else:
        # Select the console tab so that the error message is displayed except in case of debug
        if not self.ui.btnDebug.isChecked():
          self.ui.grpConsole.setCurrentIndex(2)
    # Restore mouse cursor
    self.setCursor(Qt.ArrowCursor)

  @pyqtSlot()
  def on_mnuAppOpenGcode(self):
    # Displays the opening dialog
    fileName = self.__gcodeFile.showFileOpen()
    if fileName[0] != "":
      # Reading the file
      # Hourglass slider
      self.setCursor(Qt.WaitCursor)
      RC = self.__gcodeFile.readFile(fileName[0])
      if RC:
        # Select the file tab except in case of debug
        if not self.ui.btnDebug.isChecked():
          self.ui.grpConsole.setCurrentIndex(1)
      else:
        # Select the console tab so that the error message is displayed except in case of debug
        if not self.ui.btnDebug.isChecked():
          self.ui.grpConsole.setCurrentIndex(2)
    # Enables or disables the cycle buttons
    self.setEnableDisableGroups()
    # Restore mouse cursor
    self.setCursor(Qt.ArrowCursor)


  @pyqtSlot()
  def on_mnuAppSaveGcode(self):
    if self.__gcodeFile.filePath != "":
      self.__gcodeFile.saveFile()


  @pyqtSlot()
  def on_mnuAppSaveGcodeAs(self):
    self.__gcodeFile.saveAs()


  @pyqtSlot()
  def on_mnuAppCloseGcode(self):
    self.__gcodeFile.closeFile()


  @pyqtSlot()
  def on_mnuAppQuit(self):
    self.close()


  def closeEvent(self, event):
    self.log(logSeverity.info.value, "Fermeture de l'application...")
    if self.__connectionStatus:
      self.__grblCom.stopCom()
    if not self.__gcodeFile.closeFile():
      self.log(logSeverity.info.value, "Fermeture du fichier annulee")
      event.setAccepted(False) # True accepte la fermeture, False annule la fermeture
    else:
      self.__statusText = "Bye-bye..."
      self.ui.statusBar.showMessage(self.__statusText)
      event.accept() # let the window close


  @pyqtSlot()
  def on_mnu_MPos(self):
    if self.ui.mnu_MPos.isChecked():
      param10 = 255 # Le bit 1 est a 1
      self.__grblCom.gcodeInsert("$10=" + str(param10))


  @pyqtSlot()
  def on_mnu_WPos(self):
    if self.ui.mnu_WPos.isChecked():
      param10 = 255 ^ 1 # Met le bit 1 a 0
      self.__grblCom.gcodeInsert("$10=" + str(param10))


  @pyqtSlot()
  def on_mnu_GrblConfig(self):
    ''' Configuration dialog'''
    self.__grblConfigLoaded = True
    dlgConfig = grblConfig(self.__grblCom, self.__nbAxis, self.__axisNames)
    dlgConfig.setParent(self)
    dlgConfig.sig_config_changed.connect(self.on_sig_config_changed)
    dlgConfig.showDialog()
    self.__grblConfigLoaded = False
    # Refreshed the config
    self.__grblCom.gcodeInsert(CMD_GRBL_GET_SETTINGS)

  @pyqtSlot()
  def on_mnu_ImplantRegistraion(self):
    ''' Implant registration dialog'''
    


  @pyqtSlot(str)
  def on_sig_config_changed(self, data: str):
    self.log(logSeverity.info.value, "Configuration de Grbl changee : {}".format(data) )


  @pyqtSlot()
  def on_setEmergency(self):
    if self.__eStopON:
      # If the emergency stop is active, you have to double click to deactivate it
      if not self.timerDblClick.isActive():
        # Double click timer: if it's a simple click, it will not unlock the emergency stop button. Upon first click, we activate the timer to see if the 2nd will be within the allotted time
        self.timerDblClick.setSingleShot(True)
        self.timerDblClick.start(QtWidgets.QApplication.instance().doubleClickInterval())
      else:
        # self.timerDblClick.remainingTime() > 0 # Double click detected
        self.timerDblClick.stop()
        self.__eStopON = False
        self.log(logSeverity.info.value, "Unlocking the Emergency Stop.")
    else:
      self.__grblCom.clearCom() # Empty the communication queue
      self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET) # dispatch Ctrl+X.
      self.__eStopON = True
      self.log(logSeverity.warning.value, "Emergency Stop ON!!!")

    # Updates the active / inactive state of the Grbl control groups
    self.setEnableDisableGroups()


  @pyqtSlot()
  def action_btnConnect(self):
    if not self.__connectionStatus:
      # Force l'onglet "Grbl output" sauf en cas de debug
      if not self.ui.btnDebug.isChecked():
        self.ui.grpConsole.setCurrentIndex(0)
      # Recupere les coordonnees et parametres du port a connecter
      serialDevice = self.ui.cmbPort.currentText()
      serialDevice = serialDevice.split("-")
      serialDevice = serialDevice[0].strip()
      baudRate = int(self.ui.cmbBauds.currentText())
      # Demarrage du communicator
      self.__grblCom.startCom(serialDevice, baudRate)
    else:
      # Arret du comunicator
      self.__grblCom.stopCom()
      self.__connectionStatus = self.__grblCom.isOpen()
      self.ui.btnConnect.setText("Connect")
      # Force l'onglet "Grbl output" sauf en cas de debug
      if not self.ui.btnDebug.isChecked():
        self.ui.grpConsole.setCurrentIndex(2)


  @pyqtSlot()
  def on_sig_connect(self):
    self.__connectionStatus = self.__grblCom.isOpen()
    if self.__connectionStatus:
      # Mise a jour de l'interface machine connectée
      self.ui.lblConnectStatus.setText("Connect to {}".format(self.ui.cmbPort.currentText().split("-")[0].strip()))
      self.ui.btnConnect.setText("Unconnect")
      self.setEnableDisableConnectControls()
      # Active les groupes de controles de pilotage de Grbl
      self.setEnableDisableGroups()
    else:
      # Mise a jour de l'interface machine non connectée
      self.ui.lblConnectStatus.setText("<Not Connected>")
      self.ui.btnConnect.setText("Connect")
      self.__statusText = ""
      self.ui.statusBar.showMessage(self.__statusText)
      self.setEnableDisableConnectControls()
      # Force la position de l'arret d'urgence
      self.__eStopON = True
      # Active les groupes de controles de pilotage de Grbl
      self.setEnableDisableGroups()
      # On redemandera les paramètres à la prochaine connection
      self.__firstGetSettings = False


  @pyqtSlot(int)
  def on_feedOverride(self, value: int):
    adjustFeedOverride(int(self.ui.lblAvancePourcent.text()[:-1]), value, self.__grblCom)
    self.ui.lblAvancePourcent.setText("{}%".format(value))
    if self.ui.btnLinkOverride.isChecked() and (value != self.ui.dialBroche.value()):
      self.ui.dialBroche.setValue(value)


  @pyqtSlot(int)
  def on_spindleOverride(self, value: int):
    adjustSpindleOverride(int(self.ui.lblBrochePourcent.text()[:-1]), value, self.__grblCom)
    self.ui.lblBrochePourcent.setText("{}%".format(value))
    if self.ui.btnLinkOverride.isChecked() and (value != self.ui.dialAvance.value()):
      self.ui.dialAvance.setValue(value)


  @pyqtSlot()
  def on_btnLinkOverride(self):
    if self.ui.btnLinkOverride.isChecked() and (self.ui.dialAvance.value() != self.ui.dialBroche.value()):
      newValue = (self.ui.dialAvance.value() + self.ui.dialBroche.value()) / 2
      self.ui.dialBroche.setValue(newValue)
      self.ui.dialAvance.setValue(newValue)


  @pyqtSlot()
  def on_btnResetAvance(self):
    self.ui.dialAvance.setValue(100)


  @pyqtSlot()
  def on_btnResetBroche(self):
    self.ui.dialBroche.setValue(100)


  @pyqtSlot()
  def on_cmbPort_changed(self):
    self.setEnableDisableConnectControls()


  @pyqtSlot(cnQPushButton, QtGui.QMouseEvent)
  def on_jog(self, cnButton, e):
    jogDistance = 0
    for qrb in [self.ui.rbtJog0000, self.ui.rbtJog0001, self.ui.rbtJog0010, self.ui.rbtJog0100, self.ui.rbtJog1000]:
      if qrb.isChecked():
        jogDistance = float(qrb.text().replace(' ', ''))

    if jogDistance != 0:
      self.__jogModContinue = False
      while cnButton.isMouseDown():  # on envoi qu'après avoir relâché le bouton
        # Process events to receive signals;
        QCoreApplication.processEvents()
      # envoi de l'ordre jog
      self.__jog.on_jog(cnButton, e, jogDistance)

    else:  # jogDistance == 0
      self.__jogModContinue = True
      # Recherche la course max de l'axe considéré
      axis = cnButton.name()[-1]   # L'axe est definit par le dernier caractere du nom du Bouton
      maxTravel = 0
      for I in range(self.__nbAxis):
        if axis == self.__axisNames[I]:
          maxTravel = self.__maxTravel[I]
          break
      # envoi de l'ordre jog
      self.__jog.on_jog(cnButton, e, jogDistance, maxTravel)


  @pyqtSlot(cnQPushButton, QtGui.QMouseEvent)
  def stop_jog(self, cnButton, e):
    if self.__jogModContinue:
      self.__jog.jogCancel()


  @pyqtSlot(float)
  def on_dsbJogSpeed_valueChanged(self, val: float):
    self.__jog.setJogSpeed(val)


  @pyqtSlot()
  def on_btnSpinM3(self):
    self.__grblCom.gcodeInsert("M3")
    self.ui.btnSpinM4.setEnabled(False) # Interdit un changement de sens de rotation direct


  @pyqtSlot()
  def on_btnSpinM4(self):
    self.__grblCom.gcodeInsert("M4")
    self.ui.btnSpinM3.setEnabled(False) # Interdit un changement de sens de rotation direct


  @pyqtSlot()
  def on_btnSpinM5(self):
    self.__grblCom.gcodeInsert("M5")
    self.ui.btnSpinM3.setEnabled(True)
    self.ui.btnSpinM4.setEnabled(True)


  @pyqtSlot(str, QtGui.QMouseEvent)
  def on_lblG5xClick(self, lblText, e):
    self.__grblCom.gcodeInsert(lblText)


  @pyqtSlot()
  def on_btnKillAlarm(self):
    self.__grblCom.gcodeInsert(CMD_GRBL_KILL_ALARM_LOCK)


  @pyqtSlot()
  def on_btnHomeCycle(self):
    self.__grblCom.gcodeInsert(CMD_GRBL_RUN_HOME_CYCLE)


  @pyqtSlot()
  def on_btnReset(self):
    self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET)


  @pyqtSlot()
  def sendCmd(self):
    if self.ui.txtGCode.text() != "":
      if self.ui.txtGCode.text() == REAL_TIME_REPORT_QUERY:
        self.decode.getNextStatus()
      if self.ui.txtGCode.text() == CMD_GRBL_GET_GCODE_PARAMATERS:
        self.decode.getNextGCodeParams()
      if self.ui.txtGCode.text() == CMD_GRBL_GET_GCODE_STATE:
        self.decode.getNextGCodeState()
    self.__grblCom.gcodePush(self.ui.txtGCode.text())
    self.ui.txtGCode.setSelection(0,len(self.ui.txtGCode.text()))
    self.ui.txtGCode.setFocus()
    if self.ui.txtGCode.text() != "":
      self.__gcodes_stack.insert(0, self.ui.txtGCode.text())
      self.__gcodes_stack_pos = 0
      self.__gcode_current_txt = ""


  @pyqtSlot()
  def txtGCode_on_Change(self):
    if self.ui.txtGCode.text() == REAL_TIME_REPORT_QUERY:
      self.logGrbl.append(REAL_TIME_REPORT_QUERY)
      self.decode.getNextStatus()
      self.__grblCom.realTimePush(REAL_TIME_REPORT_QUERY) # Envoi direct ? sans attendre le retour chariot.
      self.ui.txtGCode.setSelection(0,len(self.ui.txtGCode.text()))
    if not self.__gcode_recall_flag:
      self.__gcodes_stack_pos = -1
    else:
      self.__gcode_recall_flag = False


  @pyqtSlot(QtGui.QKeyEvent)
  def on_keyPressed(self, e):
    key = e.key()
    if QKeySequence(key+int(e.modifiers())) == QKeySequence("Ctrl+C"):
      pass
    elif QKeySequence(key+int(e.modifiers())) == QKeySequence("Ctrl+X"):
      self.logGrbl.append("Ctrl+X")
      self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET) # Envoi Ctrl+X.
    elif key == Qt.Key_Up:
      # Rappel des dernières commandes GCode
      if len(self.__gcodes_stack) > 0:
        if self.__gcode_current_txt == "":
          self.__gcode_current_txt = self.ui.txtGCode.text()
        self.__gcodes_stack_pos += 1
        if self.__gcodes_stack_pos >= 0 and self.__gcodes_stack_pos < len(self.__gcodes_stack):
          self.__gcode_recall_flag = True
          self.ui.txtGCode.setText(self.__gcodes_stack[self.__gcodes_stack_pos])
          self.ui.txtGCode.setSelection(0,len(self.ui.txtGCode.text()))
        elif self.__gcodes_stack_pos >= len(self.__gcodes_stack):
          self.__gcodes_stack_pos = len(self.__gcodes_stack) - 1
    elif key == Qt.Key_Down:
      # Rappel des dernières commandes GCode
      if len(self.__gcodes_stack) > 0:
        self.__gcodes_stack_pos -= 1
        if self.__gcodes_stack_pos >= 0 and self.__gcodes_stack_pos < len(self.__gcodes_stack):
          self.__gcode_recall_flag = True
          self.ui.txtGCode.setText(self.__gcodes_stack[self.__gcodes_stack_pos])
          self.ui.txtGCode.setSelection(0,len(self.ui.txtGCode.text()))
        elif self.__gcodes_stack_pos < 0:
          self.ui.txtGCode.setText(self.__gcode_current_txt)
          self.__gcodes_stack_pos = -1

  @pyqtSlot(int, str)
  def on_sig_log(self, severity: int, data: str):
    if severity == logSeverity.info.value:
      self.logCn5X.setTextColor(TXT_COLOR_GREEN)
      self.logCn5X.append(time.strftime("%Y-%m-%d %H:%M:%S") + " : Info    : " + data)
    elif severity == logSeverity.warning.value:
      self.logCn5X.setTextColor(TXT_COLOR_ORANGE)
      self.logCn5X.append(time.strftime("%Y-%m-%d %H:%M:%S") + " : Warning : " + data)
      if not self.ui.btnDebug.isChecked():
        self.ui.grpConsole.setCurrentIndex(2)
    elif severity == logSeverity.error.value:
      self.logCn5X.setTextColor(TXT_COLOR_RED)
      self.logCn5X.append(time.strftime("%Y-%m-%d %H:%M:%S") + " : Error   : " + data)
      if not self.ui.btnDebug.isChecked():
        self.ui.grpConsole.setCurrentIndex(2)
  def log(self, severity: int, data: str):
    self.on_sig_log(severity, data)

  @pyqtSlot(str)
  def on_sig_init(self, data: str):
    self.log(logSeverity.info.value, "cn5X++ : Grbl initialise.")
    self.logGrbl.append(data)
    self.__statusText = data.split("[")[0]
    self.ui.statusBar.showMessage(self.__statusText)
    # Interroge la config de grbl si la première fois
    if not self.__firstGetSettings:
      self.__grblCom.gcodeInsert(CMD_GRBL_GET_SETTINGS)
      self.__firstGetSettings = True


  @pyqtSlot()
  def on_sig_ok(self):
    self.logGrbl.append("ok")


  @pyqtSlot(int)
  def on_sig_error(self, errNum: int):
    self.logGrbl.append(self.decode.errorMessage(errNum))


  @pyqtSlot(int)
  def on_sig_alarm(self, alarmNum: int):
    self.logGrbl.append(self.decode.alarmMessage(alarmNum))


  @pyqtSlot(str)
  def on_sig_status(self, data: str):
    retour = self.decode.decodeGrblStatus(data)
    if retour != "":
      self.logGrbl.append(retour)


  @pyqtSlot(str)
  def on_sig_data(self, data: str):
    retour = self.decode.decodeGrblData(data)
    if retour is not None and retour != "":
      self.logGrbl.append(retour)


  @pyqtSlot(str)
  def on_sig_config(self, data: str):
    # Find the chain "[AXS: 5: XYZAB]" to retrieve the number of axes and their names
    if data[:5] == "[AXS:":
      self.__nbAxis           = int(data[1:-1].split(':')[1])
      self.__axisNames        = list(data[1:-1].split(':')[2])
      if len(self.__axisNames) < self.__nbAxis:
        # It is possible that there are fewer letters than the number of axes if Grbl implements the REPORT_VALUE_FOR_AXIS_NAME_ONCE option.
        self.__nbAxis = len(self.__axisNames);
      self.updateAxisNumber()
      self.decode.setNbAxis(self.__nbAxis)
    # Memorizes the maximum travel range
    elif data[:4] == "$130":
      self.__maxTravel[0] = float(data[5:])
    elif data[:4] == "$131":
      self.__maxTravel[1] = float(data[5:])
    elif data[:4] == "$132":
      self.__maxTravel[2] = float(data[5:])
    elif data[:4] == "$133":
      self.__maxTravel[3] = float(data[5:])
    elif data[:4] == "$134":
      self.__maxTravel[4] = float(data[5:])
    elif data[:4] == "$135":
      self.__maxTravel[5] = float(data[5:])

    if not self.__grblConfigLoaded:
      self.logGrbl.append(data)


  def updateAxisNumber(self):
      self.ui.label_grblPos_1.setText(self.__axisNames[0])
      self.ui.label_grblPos_2.setText(self.__axisNames[1])
      self.ui.label_grblPos_3.setText(self.__axisNames[2])

      if self.__nbAxis > 3:
        self.ui.label_grblPos_4.setText(self.__axisNames[3])
        self.ui.label_grblPos_4.setEnabled(True)
        self.ui.label_grblPos_4.setStyleSheet("")
        self.ui.grblPos_4.setEnabled(True)
        self.ui.grblPos_4.setStyleSheet("")
        self.ui.lblG5xA.setStyleSheet("")
        self.ui.lblG92A.setStyleSheet("")
        self.ui.lblWcoA.setStyleSheet("")
      else:
        self.ui.label_grblPos_4.setText("")
        self.ui.label_grblPos_4.setEnabled(False)
        self.ui.label_grblPos_4.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.grblPos_4.setEnabled(False)
        self.ui.grblPos_4.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG5xA.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG92A.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblWcoA.setStyleSheet("color: rgb(224, 224, 230);")

      if self.__nbAxis > 4:
        self.ui.label_grblPos_5.setText(self.__axisNames[4])
        self.ui.label_grblPos_5.setEnabled(True)
        self.ui.label_grblPos_5.setStyleSheet("")
        self.ui.grblPos_5.setEnabled(True)
        self.ui.grblPos_5.setStyleSheet("")
        self.ui.lblG5xB.setStyleSheet("")
        self.ui.lblG92B.setStyleSheet("")
        self.ui.lblWcoB.setStyleSheet("")
      else:
        self.ui.label_grblPos_5.setText("")
        self.ui.label_grblPos_5.setEnabled(False)
        self.ui.label_grblPos_5.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.grblPos_5.setEnabled(False)
        self.ui.grblPos_5.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG5xB.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG92B.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblWcoB.setStyleSheet("color: rgb(224, 224, 230);")

      if self.__nbAxis > 5:
        self.ui.label_grblPos_6.setText(self.__axisNames[5])
        self.ui.label_grblPos_6.setEnabled(True)
        self.ui.label_grblPos_6.setStyleSheet("")
        self.ui.grblPos_6.setEnabled(True)
        self.ui.grblPos_6.setStyleSheet("")
        self.ui.lblG5xC.setStyleSheet("")
        self.ui.lblG92C.setStyleSheet("")
        self.ui.lblWcoC.setStyleSheet("")
      else:
        self.ui.label_grblPos_6.setText("")
        self.ui.label_grblPos_6.setEnabled(False)
        self.ui.label_grblPos_6.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.grblPos_6.setEnabled(False)
        self.ui.grblPos_6.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG5xC.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblG92C.setStyleSheet("color: rgb(224, 224, 230);")
        self.ui.lblWcoC.setStyleSheet("color: rgb(224, 224, 230);")

      if 'A' in self.__axisNames:
        self.ui.btnJogMoinsA.setEnabled(True)
        self.ui.btnJogPlusA.setEnabled(True)
      else:
        self.ui.btnJogMoinsA.setEnabled(False)
        self.ui.btnJogPlusA.setEnabled(False)

      if 'B' in self.__axisNames:
        self.ui.btnJogMoinsB.setEnabled(True)
        self.ui.btnJogPlusB.setEnabled(True)
      else:
        self.ui.btnJogMoinsB.setEnabled(False)
        self.ui.btnJogPlusB.setEnabled(False)

      if 'C' in self.__axisNames:
        self.ui.btnJogMoinsC.setEnabled(True)
        self.ui.btnJogPlusC.setEnabled(True)
      else:
        self.ui.btnJogMoinsC.setEnabled(False)
        self.ui.btnJogPlusC.setEnabled(False)


  @pyqtSlot(str)
  def on_sig_emit(self, data: str):
    if data != "":
      self.logGrbl.append(data)
      if self.__cycleRun:
        # Recherche la ligne dans la liste du fichier GCode
        ligne = self.__gcodeFile.getGCodeSelectedLine()[0]
        while ligne < self.ui.gcodeTable.model().rowCount():
          idx = self.ui.gcodeTable.model().index(ligne, 0, QModelIndex())
          if self.ui.gcodeTable.model().data(idx) == data:
            self.__gcodeFile.selectGCodeFileLine(ligne)
            break
          else:
            ligne += 1


  @pyqtSlot(str)
  def on_sig_recu(self, data: str):
    pass


  @pyqtSlot(str)
  def on_sig_debug(self, data: str):
    if self.ui.mnuDebug_mode.isChecked():
      self.logDebug.append(data)


  @pyqtSlot()
  def on_mnuDebug_mode(self):
    ''' Set the debug button on the same status '''
    if self.ui.mnuDebug_mode.isChecked():
      if not self.ui.btnDebug.isChecked():
        self.ui.btnDebug.setChecked(True)
      self.ui.btnPausePolling.setEnabled(True)
    else:
      if self.ui.btnDebug.isChecked():
        self.ui.btnDebug.setChecked(False)
      # Ensure pooling in active when debug is off
      self.ui.btnPausePolling.setEnabled(False)
      self.ui.btnPausePolling.setChecked(False)
      self.__grblCom.startPolling()


  @pyqtSlot()
  def on_mnuA_propos(self):
    ''' Show up the About dialog box'''
    dlgAbout = cn5XAbout("Version {}".format(APP_VERSION_STRING), self.__licenceFile)
    dlgAbout.setParent(self)
    dlgAbout.showDialog()


  @pyqtSlot()
  def on_btnDebug(self):
    ''' Set the debug menu on the same status '''
    if self.ui.btnDebug.isChecked():
      if not self.ui.mnuDebug_mode.isChecked():
        self.ui.mnuDebug_mode.setChecked(True)
      self.ui.btnPausePolling.setEnabled(True)
      self.on_sig_debug("cn5X++ (v{}) : Starting debug.".format(APP_VERSION_STRING))
    else:
      self.on_sig_debug("cn5X++ (v{}) : Stop debugging.".format(APP_VERSION_STRING))
      if self.ui.mnuDebug_mode.isChecked():
        self.ui.mnuDebug_mode.setChecked(False)
      # Ensure pooling in active when debug is off
      self.ui.btnPausePolling.setEnabled(False)
      self.ui.btnPausePolling.setChecked(False)
      self.__grblCom.startPolling()


  @pyqtSlot()
  def on_btnPausePolling(self):
    self.log(logSeverity.info.value, "on_btnPausePolling({})".format(self.ui.btnPausePolling.isChecked()))
    if self.ui.btnPausePolling.isChecked():
      self.__grblCom.stopPolling()
    else:
      self.__grblCom.startPolling()


  @pyqtSlot()
  def clearDebug(self):
    self.logDebug.clear()


  def startCycle(self):
    self.log(logSeverity.info.value, "Cycle start...")
    self.__gcodeFile.selectGCodeFileLine(0)
    self.__cycleRun = True
    self.__cyclePause = False
    self.__gcodeFile.enQueue(self.__grblCom)
    self.ui.btnStart.setButtonStatus(True)
    self.ui.btnPause.setButtonStatus(False)
    self.ui.btnStop.setButtonStatus(False)


  def pauseCycle(self):
    if self.ui.lblEtat.text() == GRBL_STATUS_HOLD1:
      self.log(logSeverity.warning.value, "Hold in progress, impossible to leave now.")
    if self.ui.lblEtat.text() == GRBL_STATUS_HOLD0:
      self.log(logSeverity.info.value, "Resume cycle...")
      self.__grblCom.realTimePush(REAL_TIME_CYCLE_START_RESUME)
      self.__cyclePause = False
      self.ui.btnStart.setButtonStatus(True)
      self.ui.btnPause.setButtonStatus(False)
      self.ui.btnStop.setButtonStatus(False)
    else:
      self.log(logSeverity.info.value, "Pause cycle...")
      self.__grblCom.realTimePush(REAL_TIME_FEED_HOLD)
      self.__cyclePause = True
      self.ui.btnStart.setButtonStatus(False)
      self.ui.btnPause.setButtonStatus(True)
      self.ui.btnStop.setButtonStatus(False)


  def stopCycle(self):
    if self.ui.lblEtat.text() == GRBL_STATUS_HOLD0:
      # Deja en pause, on vide la file d'attente et on envoie un SoftReset
      self.log(logSeverity.info.value, "Arret du cycle...")
      self.__grblCom.clearCom() # Vide la file d'attente de communication
      self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET) # Envoi Ctrl+X.
    elif self.ui.lblEtat.text() == GRBL_STATUS_HOLD1:
      # Attente que le Hold soit termine
      self.log(logSeverity.info.value, "Pause en cours avant arret du cycle...")
      while self.ui.lblEtat.text() == GRBL_STATUS_HOLD1:
        QCoreApplication.processEvents()
      # Puis, vide la file d'attente et envoie un SoftReset
      self.log(logSeverity.info.value, "Arret du cycle...")
      self.__grblCom.clearCom() # Vide la file d'attente de communication
      self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET) # Envoi Ctrl+X.
    else:
      # Envoie une pause
      self.log(logSeverity.info.value, "Pause avant arret du cycle...")
      self.__grblCom.realTimePush(REAL_TIME_FEED_HOLD)
      # Attente que le Hold soit termine
      while self.ui.lblEtat.text() != GRBL_STATUS_HOLD0:
        QCoreApplication.processEvents()
      # Puis, vide la file d'attente et envoie un SoftReset
      self.log(logSeverity.info.value, "Arret du cycle...")
      self.__grblCom.clearCom() # Vide la file d'attente de communication
      self.__grblCom.realTimePush(REAL_TIME_SOFT_RESET) # Envoi Ctrl+X.
    self.__cycleRun = False
    self.__cyclePause = False
    self.ui.btnStart.setButtonStatus(False)
    self.ui.btnPause.setButtonStatus(False)
    self.ui.btnStop.setButtonStatus(True)
    self.log(logSeverity.info.value, "Cycle termine.")


  def on_gcodeTableContextMenu(self, event):
    if self.__gcodeFile.isFileLoaded():
      self.cMenu = QtWidgets.QMenu(self)
      editAction = QtWidgets.QAction("Editer la ligne", self)
      editAction.triggered.connect(lambda: self.editGCodeSlot(event))
      self.cMenu.addAction(editAction)
      insertAction = QtWidgets.QAction("Inserer une ligne", self)
      insertAction.triggered.connect(lambda: self.insertGCodeSlot(event))
      self.cMenu.addAction(insertAction)
      ajoutAction = QtWidgets.QAction("Ajouter une ligne", self)
      ajoutAction.triggered.connect(lambda: self.ajoutGCodeSlot(event))
      self.cMenu.addAction(ajoutAction)
      supprimeAction = QtWidgets.QAction("Supprimer la ligne", self)
      supprimeAction.triggered.connect(lambda: self.supprimeGCodeSlot(event))
      self.cMenu.addAction(supprimeAction)
      self.cMenu.popup(QtGui.QCursor.pos())

  def editGCodeSlot(self, event):
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.ui.gcodeTable.edit(idx[0])

  def insertGCodeSlot(self, event):
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.__gcodeFile.insertGCodeFileLine(idx[0].row())
    self.__gcodeFile.selectGCodeFileLine(idx[0].row())
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.ui.gcodeTable.edit(idx[0])

  def ajoutGCodeSlot(self, event):
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.__gcodeFile.addGCodeFileLine(idx[0].row())
    self.__gcodeFile.selectGCodeFileLine(idx[0].row()+1)
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.ui.gcodeTable.edit(idx[0])

  def supprimeGCodeSlot(self, event):
    idx = self.ui.gcodeTable.selectionModel().selectedIndexes()
    self.__gcodeFile.deleteGCodeFileLine(idx[0].row())


  def on_dialAvanceContextMenu(self):
    self.cMenu = QtWidgets.QMenu(self)
    resetAction = QtWidgets.QAction("Reinitialiser l'avance a 100%", self)
    resetAction.triggered.connect(lambda: self.ui.dialAvance.setValue(100))
    self.cMenu.addAction(resetAction)
    self.cMenu.popup(QtGui.QCursor.pos())


  def on_dialBrocheContextMenu(self):
    self.cMenu = QtWidgets.QMenu(self)
    resetAction = QtWidgets.QAction("Reinitialiser la vitesse de broche a 100%", self)
    resetAction.triggered.connect(lambda: self.ui.dialBroche.setValue(100))
    self.cMenu.addAction(resetAction)
    self.cMenu.popup(QtGui.QCursor.pos())


  def on_grblPos_6ontextMenu(self, axis: str):
    self.cMenu = QtWidgets.QMenu(self)
    resetX = QtWidgets.QAction("Reinitialiser l'axe {} a zero".format(self.__axisNames[axis]), self)
    resetX.triggered.connect(lambda: self.__grblCom.gcodePush("G10 P0 L20 {}0".format(self.__axisNames[axis])))
    self.cMenu.addAction(resetX)
    resetAll = QtWidgets.QAction("Reinitialiser tous les axes a zero", self)
    gcodeString = "G10 P0 L20 "
    for N in self.__axisNames:
      gcodeString += "{}0 ".format(N)
    resetAll.triggered.connect(lambda: self.__grblCom.gcodePush(gcodeString))
    self.cMenu.addAction(resetAll)
    self.cMenu.addSeparator()
    resetX = QtWidgets.QAction("Retour de {} a la position zero".format(self.__axisNames[axis]), self)
    cmdJog1 = CMD_GRBL_JOG + "G90G21F{}{}0".format(self.ui.dsbJogSpeed.value(), self.__axisNames[axis])
    resetX.triggered.connect(lambda: self.__grblCom.gcodePush(cmdJog1))
    self.cMenu.addAction(resetX)
    resetAll = QtWidgets.QAction("Retour de tous les axes en position zero", self)
    cmdJog = CMD_GRBL_JOG + "G90G21F{}".format(self.ui.dsbJogSpeed.value())
    for N in self.__axisNames:
      cmdJog += "{}0 ".format(N)
    resetAll.triggered.connect(lambda: self.__grblCom.gcodePush(cmdJog))
    self.cMenu.addAction(resetAll)
    self.cMenu.popup(QtGui.QCursor.pos())


  def on_lblGXXContextMenu(self, piece: int):
    self.cMenu = QtWidgets.QMenu(self)
    setOrigineAll = QtWidgets.QAction("Positionner l'origine piece {} (G{})".format(str(piece), str(piece + 53)), self)

  def on_lblPlanContextMenu(self):
    self.cMenu = QtWidgets.QMenu(self)
    planXY = QtWidgets.QAction("Plan de travail G17 - XY (Defaut)", self)
    planXY.triggered.connect(lambda: self.__grblCom.gcodePush("G17"))
    self.cMenu.addAction(planXY)
    planXZ = QtWidgets.QAction("Plan de travail G18 - XZ", self)
    planXZ.triggered.connect(lambda: self.__grblCom.gcodePush("G18"))
    self.cMenu.addAction(planXZ)
    planYZ = QtWidgets.QAction("Plan de travail G19 - YZ", self)
    planYZ.triggered.connect(lambda: self.__grblCom.gcodePush("G19"))
    self.cMenu.addAction(planYZ)
    self.cMenu.popup(QtGui.QCursor.pos())


  def on_lblUnitesContextMenu(self):
    self.cMenu = QtWidgets.QMenu(self)
    unitePouces = QtWidgets.QAction("G20 - Unites travail en pouces", self)
    unitePouces.triggered.connect(lambda: self.__grblCom.gcodePush("G20"))
    self.cMenu.addAction(unitePouces)
    uniteMM = QtWidgets.QAction("G21 - Unites travail en millimetres", self)
    uniteMM.triggered.connect(lambda: self.__grblCom.gcodePush("G21"))
    self.cMenu.addAction(uniteMM)
    self.cMenu.popup(QtGui.QCursor.pos())


  def on_lblCoordContextMenu(self):
    self.cMenu = QtWidgets.QMenu(self)
    unitePouces = QtWidgets.QAction("G90 - Deplacements en coordonnees absolues", self)
    unitePouces.triggered.connect(lambda: self.__grblCom.gcodePush("G90"))
    self.cMenu.addAction(unitePouces)
    uniteMM = QtWidgets.QAction("G91 - Deplacements en coordonnees relatives", self)
    uniteMM.triggered.connect(lambda: self.__grblCom.gcodePush("G91"))
    self.cMenu.addAction(uniteMM)
    self.cMenu.popup(QtGui.QCursor.pos())

  # Path Generation
  def deletePoint(self):
    if len(self.vtk_viewer.points) > 0:
      self.vtk_viewer.deleteLatestPoint()
    else:
      print("No points in the VTK viewer!")
  
  def showAxis(self):
      self.vtk_viewer.to_show_axis = not self.vtk_viewer.to_show_axis
      if self.vtk_viewer.to_show_axis:
        print("showing axis")
      else:
        print("stop showing axis")
  
  def find_cut_vector(self):
      if self.vtk_viewer.axis_vector:
        self.vtk_viewer.findCuttingVectors()
      else:
        print("No axis vector!")

"""******************************************************************"""

if __name__ == '__main__':
  import sys
  app = QtWidgets.QApplication(sys.argv)

  grbl_window = GrblMainwindow()
  grbl_window.show()
  sys.exit(app.exec_())
