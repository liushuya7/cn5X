import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread, QTimer
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QFileDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic

import cv2
import numpy as np
import csv
from scipy.spatial.transform import Rotation
from Viewer import PathState

# debug
import pickle
import sys

self_dir = os.path.dirname(os.path.realpath(__file__))
IMAGE_PATH = os.path.join(self_dir, '../img')
IMAGE_FILTER = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
DATA_PATH = os.path.join(self_dir, '../data')
DATA_FILTER = "CSV files (*.csv);;Text files (*.txt)"

class PathGenerationDialog(QDialog):
	''' Path Generation dialog '''
	def __init__(self, viewer):
		super().__init__()
		ui_dlgPathGeneration = os.path.join(self_dir, '../ui/pathGeneration.ui')
		self.__dl = uic.loadUi(ui_dlgPathGeneration, self)

		self.viewer = viewer
		self.finished.connect(self.closeWindow)

		self.__dl.label_status.setText("Select Source and Target mesh, transform source mesh to target mesh.")
		self.__dl.pushButton_transform.clicked.connect(self.transform)
		self.__dl.pushButton_boolean_operation.clicked.connect(self.boolean_operation)
		self.__dl.pushButton_extraction_start.clicked.connect(self.startExtraction)
		self.__dl.pushButton_delete_point.clicked.connect(self.deletePoint)
		self.__dl.pushButton_show_axis.clicked.connect(self.showAxis)
		self.__dl.pushButton_cut_vector.clicked.connect(self.findCutVector)
		self.__dl.pushButton_save_path.clicked.connect(self.saveCutPath)
		self.__dl.pushButton_delete_all.clicked.connect(self.deleteAll)
		self.__dl.checkBox_show_normal.stateChanged.connect(self.setViewerNormal)
		self.__dl.pushButton_generate_path.clicked.connect(self.generatePath)

		self.show()


	def setViewerNormal(self):
		if self.__dl.checkBox_show_normal.isChecked():
			self.viewer.to_show_normal = True
		else:
			self.viewer.to_show_normal = False 

	def putSourceName(self, name):
		self.__dl.lineEdit_source.setText(name)
		# update label_status is both source and target names are assigned
		if self.__dl.lineEdit_target.text() != '':
			self.__dl.label_status.setText("Select a point on target mesh, will show selected point and its normal vector.")

	def putTargetName(self, name):
		self.__dl.lineEdit_target.setText(name)
		if self.__dl.lineEdit_source.text() != '':
			self.__dl.label_status.setText("Select a point on target mesh, will show selected point and its normal vector.")

	def putVolumeMeshName(self, name):
		self.__dl.lineEdit_flat_end.setText(name)

	def transform(self):
		source_name = self.lineEdit_source.text()
		target_name = self.lineEdit_target.text()
		self.viewer.transformSource(source_name, target_name)

	def boolean_operation(self):
		source_name = self.lineEdit_source.text()
		target_name = self.lineEdit_target.text()
		self.__dl.label_status.setText("Working on Boolean operation...")
		self.viewer.boolean_operation(source_name, target_name)
		self.__dl.label_status.setText("Finished Boolean operation.")

	def startExtraction(self):
		self.viewer.state = PathState['EXTRACT']
		self.viewer.setEnableDisableGroupActions()

	def generatePath(self):
		volume_mesh = self.__dl.lineEdit_flat_end.text()
		if volume_mesh == '':
			self.__dl.label_status.setText("Error: No volume mesh selected.")
		else:
			distance_between_layer = self.doubleSpinBox_dist_layer.value()
			distance_between_line = self.doubleSpinBox_dist_line.value()
			self.viewer.generatePath(volume_mesh, distance_between_layer, distance_between_line)

	def closeWindow(self):
		pass

	def deletePoint(self):
		if len(self.viewer.points) > 0:
			self.viewer.deleteLatestPoint()
		else:
			print("No points in the VTK viewer!")
	
	def showAxis(self):
		self.viewer.to_show_axis = not self.viewer.to_show_axis
		if self.viewer.to_show_axis:
			print("Showing axis")
		else:
			print("Stop showing axis")
	
	def findCutVector(self):
		if self.viewer.axis_vector:
			cut_angle = self.doubleSpinBox_angle.value()
			self.viewer.findCuttingVectors(cut_angle)
		else:
			print("No axis vector!")

	def saveCutPath(self):
		if self.viewer.cut_path:
			file_name = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', directory=self_dir, filter="CSV files (*.csv);;Text files (*.txt)")
			if file_name != '':
				file_name = file_name[0]
				with open(file_name, 'wt') as stream:
					writer = csv.writer(stream, lineterminator='\n')
					# Get path from Viewer
					cut_path = self.viewer.getCutPath()
					# save cut path 
					for path in cut_path:
						writer.writerow(path)
				print("File saved!")
			else:
				print("No file name given!")
		else:
			print("No cut path!")

	def deleteAll(self):
		self.viewer.deleteAllPoints()


	@staticmethod
	def convertToSE3(tvec, quat):
		"""
		Converts quaternion and translation vector into SE3 lie group

		Args:
			quat: quaternion representation of rotation (scalar last)
			tvec: translation vector (meters)

		Returns:
			tf: SE3 transformation (rotation + translation)
		"""
		r = Rotation.from_quat(quat)
		r_mat = r.as_matrix()
		tvec = tvec.reshape((3,1))
		tf = np.concatenate((r_mat, tvec), axis=1)
		last_row = [[0, 0, 0, 1]]
		tf = np.concatenate((tf, last_row), axis=0)
		return tf