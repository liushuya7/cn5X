import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QFileDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic

import cv2
import numpy as np

from CameraThread import CameraThread
from ImageWindowThread import ImageWindowThread

self_dir = os.path.dirname(os.path.realpath(__file__))

class CameraDialog(QDialog):
	''' Camera dialog '''
	def __init__(self):
		super().__init__()
		ui_dlgCamera = os.path.join(self_dir, '../ui/camera.ui')
		self.__dl = uic.loadUi(ui_dlgCamera, self)
		self.finished.connect(self.closeWindow)
		self.camera_thread = None
		self.threads = []
		self.show()

		self.__dl.pushButton_start.clicked.connect(self.startVideo)
		self.__dl.pushButton_stop.clicked.connect(self.closeWindow)
		self.__dl.pushButton_match.clicked.connect(self.matchFeatures)

	def startVideo(self):
		self.camera_thread = CameraThread()
		self.camera_thread.start()

	def closeWindow(self):
		if self.camera_thread and self.camera_thread.running:
			self.camera_thread.running = False
			self.camera_thread.quit()
			self.camera_thread.wait(3000)
			print("finished:  " + str(self.camera_thread.isFinished()))
			self.camera_thread.realsense.stop()

	def matchFeatures(self):
		file_filter = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
		file_name = QFileDialog()
		file_name.setFileMode(QFileDialog.ExistingFiles)
		names, _ = file_name.getOpenFileNames(self, "Open files", self_dir, file_filter)

		# open ImageWindow
		for name in names:
			self.threads.append()



		
