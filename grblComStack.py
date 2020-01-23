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

from PyQt5.QtCore import QObject, QThread, QEventLoop, pyqtSignal, pyqtSlot
from cn5X_config import *

class grblStack():
  '''
  Serial port queue manager.
  Stores couples (CommandGrbl, flag), either in FiFo mode (addFiFo ()), or in LiFo mode (addLiFo ())
  and return them in the order chosen with the pop () function
  '''

  def __init__(self):
    self.__data = []

  def isEmpty(self):
    return len(self.__data) == 0

  def count(self):
    return len(self.__data)

  def addFiFo(self, item, flag = COM_FLAG_NO_FLAG):
    ''' Add an element in FiFO mode, the added element will be the last to exit '''
    self.__data.append((item, flag))

  def addLiFo(self, item, flag = COM_FLAG_NO_FLAG):
    ''' Add an element in LiFO mode, the added element will be the first to exit '''
    self.__data.insert(0, (item, flag))

  def next(self):
    ''' Returns the next element of the Queue without depilating (deleting it) or None if the list is empty. '''
    if len(self.__data) > 0:
      return self.__data[0]
    else:
      return None

  def pop(self):
    ''' Pop and return the first element of the list or None if the list is empty. '''
    if len(self.__data) > 0:
      return self.__data.pop(0)
    else:
      return None

  def clear(self):
    ''' Empty the entire stack '''
    self.__data.clear()
