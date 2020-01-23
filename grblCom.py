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

import sys, time
from math import *
from PyQt5.QtCore import QCoreApplication, QObject, QThread, QTimer, QEventLoop, pyqtSignal, pyqtSlot, QIODevice
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from cn5X_config import *
from grblComSerial import grblComSerial


class grblCom(QObject):
  '''
  Management of the serial communication thread with Grbl
  '''
  # Resumption of signals from the Com thread to forward
  sig_log     = pyqtSignal(int, str) # Operating message of the grblComSerial component, returns: logSeverity, message string
  sig_connect = pyqtSignal()         # Emits upon receipt of connection
  sig_init    = pyqtSignal(str)      # Emits upon receipt of the Grbl initialization chain, returns the complete chain
  sig_ok      = pyqtSignal()         # Emits at the reception of "ok" from Grbl
  sig_error   = pyqtSignal(int)      # Emits upon receipt of a Grbl error, returns the error number
  sig_alarm   = pyqtSignal(int)      # Emits upon receipt of a Grbl alarm, returns the alarm number
  sig_status  = pyqtSignal(str)      # Emits upon receipt of a status message ("<... |.>"), returns the complete line
  sig_config  = pyqtSignal(str)      # Emits upon receipt of a config value ($ XXX)
  sig_data    = pyqtSignal(str)      # Emits upon receipt of other Grbl data, returns the complete line
  sig_emit    = pyqtSignal(str)      # Emits when sending data on the serial port
  sig_recu    = pyqtSignal(str)      # Emits upon receipt of data on the serial port
  sig_debug   = pyqtSignal(str)      # Emits with each sending or reception

  # Control signals to send to the Grbl thread
  sig_abort        = pyqtSignal()
  sig_gcodeInsert  = pyqtSignal(str, object)
  sig_gcodePush    = pyqtSignal(str, object)
  sig_realTimePush = pyqtSignal(str, object)
  sig_clearCom     = pyqtSignal()
  sig_startPolling = pyqtSignal()
  sig_stopPolling  = pyqtSignal()

  def __init__(self):
    super().__init__()
    self.__threads       = None
    self.__Com           = None
    self.__connectStatus = False
    self.__grblInit      = False
    self.__polling       = True
    self.__grblVersion   = ""
    self.__grblStatus    = ""
    self.__threads = []


  def startCom(self, comPort: str, baudRate: int):
    '''
    Management of serial communications and timers in separate threads
    '''

    self.sig_debug.emit("grblCom.startCom(self, {}, {})".format(comPort, baudRate))

    self.sig_log.emit(logSeverity.info.value, 'grblCom: Starting grblComSerial thread.')
    newComSerial = grblComSerial(comPort, baudRate, self.__polling)
    thread = QThread()
    thread.setObjectName('grblComSerial')
    self.__threads.append((thread, newComSerial))  # need to store worker too otherwise will be gc'd
    newComSerial.moveToThread(thread)

    # Connect signals from grblComSerial
    newComSerial.sig_log.connect(self.sig_log.emit)
    newComSerial.sig_connect.connect(self.on_sig_connect)
    newComSerial.sig_init.connect(self.on_sig_init)
    newComSerial.sig_ok.connect(self.sig_ok.emit)
    newComSerial.sig_error.connect(self.sig_error.emit)
    newComSerial.sig_alarm.connect(self.sig_alarm.emit)
    newComSerial.sig_status.connect(self.on_sig_status)
    newComSerial.sig_config.connect(self.sig_config.emit)
    newComSerial.sig_data.connect(self.sig_data.emit)
    newComSerial.sig_emit.connect(self.sig_emit.emit)
    newComSerial.sig_recu.connect(self.sig_recu.emit)
    newComSerial.sig_debug.connect(self.sig_debug.emit)

    # Control signals to send to the thread
    self.sig_abort.connect(newComSerial.abort)
    self.sig_gcodeInsert.connect(newComSerial.gcodeInsert)
    self.sig_gcodePush.connect(newComSerial.gcodePush)
    self.sig_realTimePush.connect(newComSerial.realTimePush)
    self.sig_clearCom.connect(newComSerial.clearCom)
    self.sig_startPolling.connect(newComSerial.startPolling)
    self.sig_stopPolling.connect(newComSerial.stopPolling)


    # Start the thread...
    thread.started.connect(newComSerial.run)
    thread.start()  # this will emit 'started' and start thread's event loop

    # Memorize the communicator
    self.__Com = newComSerial


  @pyqtSlot(bool)
  def on_sig_connect(self, value: bool):
    self.sig_debug.emit("grblCom.on_sig_connect(self, {})".format(value))
    # Maintain connection status
    self.__connectStatus = value
    self.sig_connect.emit()


  @pyqtSlot(str)
  def  on_sig_init(self, buff: str):
    self.sig_debug.emit("grblCom.on_sig_init(self, {})".format(buff))
    self.__grblInit = True
    self.__grblVersion = buff.split("[")[0]
    self.sig_init.emit(buff)


  def grblVersion(self):
    return self.__grblVersion


  @pyqtSlot(str)
  def on_sig_status(self, buff: str):
    self.sig_debug.emit("grblCom.on_sig_status(self, {})".format(buff))
    # Memorize the status of Grbl each time we see a pass
    self.__grblStatus = buff[1:].split('|')[0]
    self.sig_status.emit(buff)


  def grblStatus(self):
    ''' Returns the last Grbl status seen '''
    return self.__grblStatus

  def stopCom(self):
    self.sig_debug.emit("grblCom.stopCom(self)")
    self.clearCom() #Empty the queue
    self.sig_log.emit(logSeverity.info.value, "Sig_abort signal sent to serial communications thread ...")
    self.sig_abort.emit()
    # Attente de la fin du (des) thread(s)
    for thread, worker in self.__threads:
        thread.quit()  # this will quit **as soon as thread event loop unblocks**
        thread.wait()  # <- so you need to wait for it to *actually* quit
    self.sig_log.emit(logSeverity.info.value, "Thread(s) enfant(s) termine(s).")
    self.__grblInit = False
    self.__threads = []


  def gcodeInsert(self, buff: str, flag=COM_FLAG_NO_FLAG):
    if self.__connectStatus and self.__grblInit:
      self.sig_gcodeInsert.emit(buff, flag)
    else:
      self.sig_log.emit(logSeverity.warning.value, "grblCom: Grbl not connected or not initialized, [{}] unable to send".format(buff) )


  def gcodePush(self, buff: str, flag=COM_FLAG_NO_FLAG):
    if self.__connectStatus and self.__grblInit:
      self.sig_gcodePush.emit(buff, flag)
    else:
      self.sig_log.emit(logSeverity.warning.value, "grblCom: Grbl not connected or not initialized, [{}] unable to send".format(buff) )


  def realTimePush(self, buff: str, flag=COM_FLAG_NO_FLAG):
    if self.__connectStatus and self.__grblInit:
      self.sig_realTimePush.emit(buff, flag)
    else:
      self.sig_log.emit(logSeverity.warning.value, "grblCom: Grbl not connected or not initialized, [{}] unable to send".format(buff) )


  def clearCom(self):
    self.sig_clearCom.emit()


  @pyqtSlot()
  def startPolling(self):
    self.__polling = True
    self.sig_startPolling.emit()


  @pyqtSlot()
  def stopPolling(self):
    self.__polling = False
    self.sig_stopPolling.emit()

  def isOpen(self):
    return self.__connectStatus

