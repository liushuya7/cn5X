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

import sys, time
from enum import Enum
from math import *
from PyQt5.QtCore import QCoreApplication, QObject, QThread, QTimer, QEventLoop, pyqtSignal, pyqtSlot, QIODevice
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from cn5X_config import *
from grblComStack import grblStack


class grblComSerial(QObject):
  '''
  QObject worker ensuring the management of two-way serial communication between cn5X ++ and grbl.
  Must be run in its own thread to not block the GUI.
  '''

  sig_log     = pyqtSignal(int, str) # Operating message of the grblComSerial component, returns: logSeverity, message string
  sig_connect = pyqtSignal(bool)     # Message sent on connection (value = True) and on disconnection or in case of connection error (value = False)
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


  def __init__(self, comPort: str, baudRate: int, polling: bool):
    super().__init__()

    self.__abort            = False
    self.__portName         = comPort
    self.__baudRate         = baudRate

    self.__realTimeStack    = grblStack()
    self.__mainStack        = grblStack()

    self.__initOK           = False
    self.__grblStatus       = ""

    self.__queryCounter     = 0
    self.__querySequence    = [
      REAL_TIME_REPORT_QUERY,
      REAL_TIME_REPORT_QUERY,
      CMD_GRBL_GET_GCODE_PARAMATERS + '\n',
      REAL_TIME_REPORT_QUERY,
      REAL_TIME_REPORT_QUERY,
      CMD_GRBL_GET_GCODE_STATE + '\n'
    ]
    self.__lastQueryTime    = time.time()
    self.__polling          = polling
    self.__okToSendGCode = True


  @pyqtSlot()
  def startPolling(self):
    self.__polling = True


  @pyqtSlot()
  def stopPolling(self):
    self.__polling = False


  @pyqtSlot()
  def abort(self):
    ''' Traitement du signal demandant l'arret de la communication '''
    self.sig_log.emit(logSeverity.info.value, self.tr("serialCommunicator : abort recu."))
    self.__abort = True


  @pyqtSlot()
  def clearCom(self):
    ''' Vide les files d'attente '''
    self.__realTimeStack.clear()
    self.__mainStack.clear()


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def realTimePush(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Addiing a GCode command in the realtime stack in FiFo mode '''
    self.__realTimeStack.addFiFo(buff, flag)


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def gcodePush(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Adding a GCode command in the stack in FiFo mode (normal functioning of the stack of a GCode program) '''
    self.__mainStack.addFiFo(buff, flag)


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def gcodeInsert(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Inserting a GCode command in the stack in LiFo mode (commands that must pass in front of the others) '''
    self.__mainStack.addLiFo(buff, flag)


  def __sendData(self, buff: str):
    ''' Sends data on the serial port '''
    # Debug signal for all data sent
    if buff[-1:] == "\n":
      self.sig_debug.emit(">>> " + buff[:-1] + "\\n")
    elif buff[-2:] == "\r\n":
      self.sig_debug.emit(">>> " + buff[:-2] + "\\r\\n")
    else:
      self.sig_debug.emit(">>> " + buff)
    # Formatting the buffer to send
    buffWrite = bytes(buff, sys.getdefaultencoding())
    # Time required for com (milliseconds), rounded up
    tempNecessaire = ceil(1000 * len(buffWrite) * 8 / self.__baudRate)
    timeout = 10 + (2 * tempNecessaire) # 2 times the time required + 10 milliseconds
    # Writing on the serial port
    self.__comPort.write(buffWrite)
    self.sig_debug.emit("grblComSerial.__sendData(), T = {} : self.__comPort.waitForBytesWritten({})".format(time.time() * 1000, timeout))
    if self.__comPort.waitForBytesWritten(timeout):
      ### remonte en amon dans la boucle principale self.sig_emit.emit(buff)
      self.sig_debug.emit(self.tr("grblComSerial : Donnees envoyees, T = {}".format(time.time() * 1000)))
      pass
    else:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Erreur envoi des donnees : timeout, err# = {}").format(self.__comPort.error()))
      self.sig_debug.emit(self.tr("grblComSerial : Erreur envoi des donnees : timeout, err# = {}, T = {}").format(self.__comPort.error(), time.time() * 1000))


  def __traileLaLigne(self, l, flag = COM_FLAG_NO_FLAG):
    ''' Emmet les signaux ad-hoc pour toutes les lignes recues  '''
    # Envoi de toutes les lignes dans le debug
    if l[-1:] == "\n":
      self.sig_debug.emit("<<< " + l[:-1] + "\\n")
    elif l[-2:] == "\r\n":
      self.sig_debug.emit("<<< " + l[:-2] + "\\r\\n")
    else:
      self.sig_debug.emit("<<< " + l)
    # Premier decodage pour envoyer le signal ah-hoc
    if l[:5] == "Grbl " and l[-5:] == "help]": # Init string : Grbl 1.1f ['$' for help]
      self.sig_init.emit(l)
    elif l == "ok":                            # Reponses "ok"
      if not flag & COM_FLAG_NO_OK:
        self.sig_ok.emit()
    elif l[:6] == "error:":                    # "error:X" => Renvoie X
      if not flag & COM_FLAG_NO_ERROR:
        errNum = int(l.split(':')[1])
        self.sig_error.emit(errNum)
    elif l[:6] == "ALARM:":                    # "ALARM:X" => Renvoie X
      alarmNum = int(l.split(':')[1])
      self.sig_alarm.emit(alarmNum)
    elif l[:1] == "<" and l[-1:] == ">":       # Real-time Status Reports
      self.__grblStatus = l[1:].split('|')[0]
      self.sig_status.emit(l)
    elif l[:1] == "$" or l[:5] == "[VER:" or l[:5] == "[AXS:" or l[:5] == "[OPT:": # Setting output
      self.sig_config.emit(l)
    else:
      self.sig_data.emit(l)


  def __openComPort(self):
    ''' Opening of the serial port and waiting for the initialization chain from Grbl '''

    self.sig_debug.emit("grblComSerial.__openComPort(self)")

    openResetTime = 1500  # Time for sending soft reset if init string is not receive from Grbl
    openMaxTime =   5000  # (ms) Timeout to receive the response from Grbl after opening the port = 5 seconds

    # Configuration port
    self.__comPort  = QSerialPort()
    self.__comPort.setPortName(self.__portName)
    self.__comPort.setBaudRate(self.__baudRate)
    self.__comPort.setDataBits(QSerialPort.Data8)
    self.__comPort.setStopBits(QSerialPort.OneStop)
    self.__comPort.setParity(QSerialPort.NoParity)

    # Open the port
    RC = False
    try:
      RC = self.__comPort.open(QIODevice.ReadWrite)
    except OSError as err:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Error openning port : {0}").format(err))
      self.sig_debug.emit(self.tr("grblComSerial : Error openning the port: {0}").format(err))
      self.sig_connect.emit(False)
      return False
    except:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Unexpected error : {}").format(sys.exc_info()[0]))
      self.sig_debug.emit(self.tr("grblComSerial : Unexpected error : {}").format(sys.exc_info()[0]))
      self.sig_connect.emit(False)
      return False
    if not RC:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Error opening serial port : err# = {0}").format(self.__comPort.error()))
      self.sig_debug.emit(self.tr("grblComSerial : Error opening serial port : err# = {0}").format(self.__comPort.error()))
      self.sig_connect.emit(False)
      return False

    # port opening OK
    self.sig_connect.emit(True)
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial : comPort {} opened, RC = {}").format(self.__comPort.portName(), RC))
    self.sig_debug.emit(self.tr("grblComSerial : comPort {} opened, RC = {}").format(self.__comPort.portName(), RC))

    # Waiting for initialisation Grbl
    tReset = False
    tDebut = time.time() * 1000
    self.sig_debug.emit(self.tr("grblComSerial : Wait for Grbl init... T = {:0.0f} ms...").format(tDebut))
    while True:
      serialData = ""
      while serialData == "" or serialData[-1] != '\n':
        time.sleep(0.25)
        if self.__comPort.waitForReadyRead(25):
          buff = self.__comPort.readAll()
          self.sig_debug.emit("grblComSerial.__openComPort() : Buffer recu : \"" + str(buff) + "\"")
          try:
            newData = buff.data().decode()
            serialData += newData
          except:
            self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Erreur decodage : {}").format(sys.exc_info()[0]))
            self.sig_debug.emit(self.tr("grblComSerial : Erreur decodage : {}").format(sys.exc_info()[0]))
        else:
          self.sig_debug.emit(self.tr("grblComSerial : Wait for Grbl init... T = {:0.0f} ms, Nothing to read on serial.").format(time.time() * 1000))
        now = time.time() * 1000
        if now > tDebut + (openResetTime) and not tReset:
          # Try to send Reset to Grbl at half time of timeout
          self.sig_debug.emit(self.tr("grblComSerial : No response from Grbl after {:0.0f}ms, sending soft reset...").format(openResetTime))
          self.__sendData(REAL_TIME_SOFT_RESET)
          tReset = True
        if now > tDebut + openMaxTime:
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Initialisation de Grbl : Timeout !"))
          self.sig_debug.emit(self.tr("grblComSerial : openMaxTime ({}ms) timeout elapsed !").format(openMaxTime))
          self.sig_debug.emit(self.tr("grblComSerial : Initialisation de Grbl : Timeout !"))
          self.__comPort.close()
          return False
      # On cherche la chaine d'initialisation dans les lignes du buffer
      for l in serialData.splitlines():
        if l[:5] == "Grbl " and l[-5:] == "help]": # Init string : Grbl 1.1f ['$' for help]
          self.sig_debug.emit(self.tr("grblComSerial : Grbl init string received in {:0.0f} ms, OK.").format(time.time()*1000 - tDebut))
          self.sig_init.emit(l)
          self.__initOK = True
        elif self.__initOK:
          self.sig_data.emit(l)
      if self.__initOK:
        # Appel de CMD_GRBL_GET_BUILD_INFO pour que l'interface recupere le nombre d'axes et leurs noms
        self.__sendData(CMD_GRBL_GET_BUILD_INFO + "\n")
        return True # On a bien recu la chaine d'initialisation de Grbl

  def __mainLoop(self):
    ''' Communication main loop: reads / writes on the serial port '''
    flag = COM_FLAG_NO_FLAG
    while True:
      # On commence par vider la file d'attente des commandes temps reel
      while not self.__realTimeStack.isEmpty():
        toSend, flag = self.__realTimeStack.pop()
        self.__sendData(toSend)
      if self.__okToSendGCode:
        # Envoi d'une ligne gcode si en attente
        if not self.__mainStack.isEmpty():
          # La pile n'est pas vide, on envoi la prochaine commande recuperee dans la pile GCode
          toSend, flag = self.__mainStack.pop()
          if toSend[-1:] != '\n':
            toSend += '\n'
          if not flag & COM_FLAG_NO_OK:
            if toSend[-2:] == '\r\n':
              self.sig_emit.emit(toSend[:-2])
            else:
              self.sig_emit.emit(toSend[:-1])
          self.__sendData(toSend)
          self.__okToSendGCode = False # On enverra plus de commande tant que l'on aura pas recu l'accuse de reception.
      else:
        # Process events to receive signals;
        QCoreApplication.processEvents()
      # Lecture du port serie
      serialData = ""
      while True:
        if self.__comPort.waitForReadyRead(25):
          buff = self.__comPort.readAll()
          try:
            newData = buff.data().decode()
            serialData += newData
          except:
            self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : Erreur decodage : {}").format(sys.exc_info()[0]))
          if serialData != "" and serialData[-1] == '\n':
            dataAvailable = True
            break # La derniere ligne recue est complette
        else:
          dataAvailable = False
          # Process events to receive signals;
          QCoreApplication.processEvents()
          break
      if dataAvailable:
        # Decoupe les donnees recues en lignes pour les envoyer une par une
        tblLines = serialData.splitlines()
        for l in tblLines:
          if l.find('ok') >= 0 or l.find('error') >= 0:
            self.__okToSendGCode = True # Accuse de reception ou erreur de la derniere commande GCode envoyee
          if l !='':
            self.__traileLaLigne(l, flag)
      if self.__abort:
        self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial : sig_abort recu, sortie du thread..."))
        break # Sortie de la boucle principale

      # Interrogations de Grbl a interval regulier selon la sequence definie par self.__querySequence
      if self.__polling:
        if (time.time() - self.__lastQueryTime) * 1000 > GRBL_QUERY_DELAY and self.__initOK:
          if len(self.__querySequence[self.__queryCounter]) == 1:
            self.realTimePush(self.__querySequence[self.__queryCounter])
          else:
            if self.__grblStatus == GRBL_STATUS_IDLE:
              self.gcodeInsert(self.__querySequence[self.__queryCounter], COM_FLAG_NO_OK | COM_FLAG_NO_ERROR)
          self.__lastQueryTime    = time.time()
          self.__queryCounter += 1
          if self.__queryCounter >= len(self.__querySequence):
            self.__queryCounter = 0

    # On est sorti de la boucle principale : fermeture du port.
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial : Fermeture du port serie."))
    self.sig_connect.emit(False)
    self.__comPort.close()
    self.__initOK = False
    # Emission du signal de fin
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial : Fin."))


  def run(self):
    ''' Start communication with the serial port in a separate thread '''

    thread_name = QThread.currentThread().objectName()
    thread_id = int(QThread.currentThreadId())  # cast to int() is necessary
    self.sig_log.emit(logSeverity.info.value, self.tr('grblComSerial running "{}" from thread #{}.').format(thread_name, hex(thread_id)))

    if self.__openComPort():
      self.__mainLoop()
    else:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial : impossible d'ouvrir le port serie !"))
      # Emission du signal de fin
      self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial : Fin."))

