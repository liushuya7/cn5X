import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread, QTimer
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QFileDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic

import cv2
import numpy as np
import csv
from scipy.spatial.transform import Rotation

# debug
import pickle
import sys
import roslibpy
import roslibpy.tf

from CameraThread import CameraThread
from ImageWindowThread import ImageWindowThread
from TwoViewEstimate import TwoViewEstimate
from ImageProcessing import ImageProcessing

self_dir = os.path.dirname(os.path.realpath(__file__))
IMAGE_PATH = os.path.join(self_dir, '../img')
IMAGE_FILTER = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
DATA_PATH = os.path.join(self_dir, '../data')
DATA_FILTER = "CSV files (*.csv);;Text files (*.txt)"

class CameraDialog(QDialog):
	''' Camera dialog '''
	def __init__(self, ros):
		super().__init__()
		ui_dlgCamera = os.path.join(self_dir, '../ui/camera.ui')
		self.__dl = uic.loadUi(ui_dlgCamera, self)
		self.finished.connect(self.closeWindow)
		self.camera_thread = None
		self.window_threads = []
		self.saved_file = 0
		self.imgs_key_to_id = [] # 0:base_frame
		self.imgs = {}  # dictionay of images
		self.poses = {} # dictionary of poses of image(camera)
		self.show()

		self.__dl.pushButton_start_camera.clicked.connect(self.startVideo)
		self.__dl.pushButton_capture.clicked.connect(self.captureImage)
		self.__dl.pushButton_close_camera.clicked.connect(self.closeWindow)
		self.__dl.pushButton_match.clicked.connect(self.matchFeatures)
		self.__dl.pushButton_close_windows.clicked.connect(self.closeCVWindows)
		self.__dl.pushButton_reconstruct.clicked.connect(self.reconstructPoints)
		self.__dl.pushButton_automatic.clicked.connect(self.automaticReconstruction)

		self.ros = ros
		self.ros.on_ready(lambda: print('Camera (ROS Connection):', self.ros.is_connected))
		self.tf_listener = roslibpy.tf.TFClient(self.ros, 'W', rate=100, angular_threshold=0.001, translation_threshold=0.0001)
		self.tf_listener.subscribe('camera_color_optical_frame', self.updateCameraPose)
		self.file_name_tf = None
		
	def updateCameraPose(self, data):
		# update camera realtime pose
		self.camera_pose = data

	def processTfData(self, data):
		quat = [data['rotation']['x'], data['rotation']['y'], data['rotation']['z'], data['rotation']['w']]
		tvec = [data['translation']['x'], data['translation']['y'], data['translation']['z']]
		camera_pose = []
		camera_pose.extend(tvec)
		camera_pose.extend(quat)
		print(camera_pose)

		return camera_pose

	def startVideo(self):
		self.camera_thread = CameraThread()
		self.camera_thread.start()

	def captureImage(self):

		# make dirs if not existed
		if not os.path.isdir(IMAGE_PATH):
			os.makedirs(IMAGE_PATH,exist_ok=True)
		if not os.path.isdir(DATA_PATH):
			os.makedirs(DATA_PATH,exist_ok=True)

		if self.__dl.checkBox_save.isChecked():
			if not self.file_name_tf: 
				self.file_name_tf = os.path.join(DATA_PATH, str("tf")+'.csv')
			# save camera pose
			# TODO: figure out a way to delete former tf file, since we are using "at" to append data
			camera_pose = self.camera_pose
			camera_pose = self.processTfData(camera_pose)
			with open(self.file_name_tf, 'at') as stream:
				writer = csv.writer(stream, lineterminator='\n')
				writer.writerow(camera_pose)

			file_name_img = os.path.join(IMAGE_PATH, str(self.saved_file).zfill(2)+'.png')
			self.camera_thread.saveImage(file_name_img)

			self.saved_file += 1

		# TODO: fix bug for saving tf data here
		else:
			file_filter = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
			file_name,_ = QFileDialog.getSaveFileName(self, 'Save File', directory=IMAGE_PATH,filter=file_filter)
			if file_name != '':
				self.camera_thread.saveImage(file_name)

	def closeCamera(self):
		if self.camera_thread and self.camera_thread.running:
			self.stopThreadSafely(self.camera_thread)

	def closeWindow(self):
		self.closeCamera()

	def matchFeatures(self):
		# get image file names
		file_name = QFileDialog()
		file_name.setFileMode(QFileDialog.ExistingFiles)
		image_names, _ = file_name.getOpenFileNames(self, "Open files", IMAGE_PATH, IMAGE_FILTER)
		tf_name, _ = QFileDialog.getOpenFileName(self, "Open files", DATA_PATH, DATA_FILTER)

		# process tf data
		camera_poses = []
		if tf_name != '' and os.path.splitext(tf_name)[1] in DATA_FILTER:
			with open(tf_name, 'r') as stream:
				reader = csv.reader(stream, delimiter=',')
				for row in reader:
					row = [float(value) for value in row]
					camera_poses.append(row)

		# store images TODO: a way to associate image, poses, image_id for two-view and multi-view neatly
		for i,name in enumerate(image_names):
			img_name = os.path.split(name)[1] # extract the tail part for window name, formatted as "name_ID.png"
			# extract id of image 
			img_id = int(img_name.split('.')[0].split('_')[-1])
			img = cv2.imread(name)
			self.imgs_key_to_id.append(img_id)
			self.imgs[img_id] = img
			self.poses[img_id] = CameraDialog.convertToSE3(np.array(camera_poses[img_id][:3]), np.array(camera_poses[img_id][3:]))
			thread = ImageWindowThread(img_name, img)
			self.window_threads.append(thread)

		# TODO: need to feed in the parameters in a neat way, currently hard code camera intrinsic parameters here
		mtx=np.array([[1.37347548e+03, 0, 9.61674092e+02],
		                                    [0, 1.37389297e+03, 5.21102993e+02],
		                                    [0, 0, 1]]);
		dist=np.array([[ 1.02888212e-01, -1.53687651e-01,  4.25168437e-05, -2.58183034e-03, -2.02461554e-01]]);

		self.two_view_estimate = TwoViewEstimate(self.imgs[self.imgs_key_to_id[0]], self.imgs[self.imgs_key_to_id[1]],\
			 									mtx, dist,\
												self.poses[self.imgs_key_to_id[0]], self.poses[self.imgs_key_to_id[1]])

		self.window_threads[0].window.img = self.two_view_estimate.img1_rectified
		self.window_threads[1].window.img = self.two_view_estimate.img2_rectified
		for thread in self.window_threads:
			thread.start()
		# self.windows_cleared = False
		# self.thread_manage_timer = QTimer()
		# self.thread_manage_timer.timeout.connect(self.manageThreads)
		# self.thread_manage_timer.start(1000)

	def stopThreadSafely(self, thread, wait_time=1000):

		thread.running = False
		thread.closeWindow()
		thread.quit()
		thread.wait(wait_time)
		print("thread is finished: " + str(thread.isFinished()))
		# if not thread.isFinished():
		# 	print("force to terminate thread: " + str(thread))
		# 	thread.terminate()
		# 	thread.wait()

	def closeCVWindows(self):
		for thread in self.window_threads:
			if thread.running:
				self.stopThreadSafely(thread, wait_time=500)

	def manageThreads(self):
		if not self.windows_cleared:
			windows_cleared = True
			for thread in self.window_threads:
				if not thread.running:
					self.stopThreadSafely(thread)
				windows_cleared = thread.isFinished and windows_cleared
			self.windows_cleared = windows_cleared
		else:
			self.thread_manage_timer.stop()
			print("Thread manager stopped!")

	def reconstructPoints(self, pts1=None, pts2=None):

		if pts1 is None or pts2 is None:
			self.two_view_estimate.estimate(self.window_threads[0].window.points_list, self.window_threads[1].window.points_list, scale=1000);
		else:
			self.two_view_estimate.estimate(pts1, pts2, scale=1000)
		print("Reconstructed points are saved in /data")

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

	def automaticReconstruction(self):

		# get image file names
		file_name = QFileDialog()
		file_name.setFileMode(QFileDialog.ExistingFiles)
		image_names, _ = file_name.getOpenFileNames(self, "Open files", IMAGE_PATH, IMAGE_FILTER)
		tf_name, _ = QFileDialog.getOpenFileName(self, "Open files", DATA_PATH, DATA_FILTER)

		# process tf data
		camera_poses = []
		if tf_name != '' and os.path.splitext(tf_name)[1] in DATA_FILTER:
			with open(tf_name, 'r') as stream:
				reader = csv.reader(stream, delimiter=',')
				for row in reader:
					row = [float(value) for value in row]
					camera_poses.append(row)

		# store images TODO: a way to associate image, poses, image_id for two-view and multi-view neatly
		for i,name in enumerate(image_names):
			img_name = os.path.split(name)[1] # extract the tail part for window name, formatted as "name_ID.png"
			# extract id of image 
			img_id = int(img_name.split('.')[0].split('_')[-1])
			img = cv2.imread(name)
			self.imgs_key_to_id.append(img_id)
			self.imgs[img_id] = img
			self.poses[img_id] = CameraDialog.convertToSE3(np.array(camera_poses[img_id][:3]), np.array(camera_poses[img_id][3:]))

		# return if no input files
		if len(self.imgs_key_to_id) == 0:
			prin("No files selected!")
			return 

		# TODO: need to feed in the parameters in a neat way, currently hard code camera intrinsic parameters here
		mtx=np.array([[1.37347548e+03, 0, 9.61674092e+02],
		                                    [0, 1.37389297e+03, 5.21102993e+02],
		                                    [0, 0, 1]]);
		dist=np.array([[ 1.02888212e-01, -1.53687651e-01,  4.25168437e-05, -2.58183034e-03, -2.02461554e-01]]);

		self.two_view_estimate = TwoViewEstimate(self.imgs[self.imgs_key_to_id[0]], self.imgs[self.imgs_key_to_id[1]],\
			 									mtx, dist,\
												self.poses[self.imgs_key_to_id[0]], self.poses[self.imgs_key_to_id[1]])
		image_processor_1 = ImageProcessing(self.two_view_estimate.img1_rectified)
		feature_points_1, img_with_keypoint_1 = image_processor_1.extractFeaturePoints()
		image_processor_2 = ImageProcessing(self.two_view_estimate.img2_rectified)
		feature_points_2, img_with_keypoint_2 = image_processor_2.extractFeaturePoints()
		feature_points_1 = np.array(feature_points_1)
		feature_points_2 = np.array(feature_points_2)

		# check to match features vertically or horizontally in images
		to_match_vertically = self.__dl.checkBox_match_vertically.isChecked()
		threshold = 5 # debug: need to tune for good and robust reconstruction

		if to_match_vertically:
			# feature_points should be sorted by pixel-x-coordinate in decreasing order by default
			# extract intersection points, set threshold to be 2 pixel for now
			points_for_triangulation_1 = []
			points_for_triangulation_2 = []
			for point in feature_points_1:
				difference = abs(feature_points_2 - point)[:,0]
				ret = np.where(difference < threshold)
				potential_correspondence_id = ret[0]
				if len(potential_correspondence_id) == 1:
					# TODO: need to handle case for multiple potential_correspondences	
					points_for_triangulation_1.append(point.tolist())
					points_for_triangulation_2.append(feature_points_2[potential_correspondence_id].flatten().tolist())
		else:
			# feature_points should be sorted by pixel-y-coordinate in decreasing order by default
			# extract intersection points, set threshold to be 2 pixel for now
			points_for_triangulation_1 = []
			points_for_triangulation_2 = []
			threshold = 5
			for point in feature_points_1:
				difference = abs(feature_points_2 - point)[:,1]
				ret = np.where(difference < threshold)
				potential_correspondence_id = ret[0]
				if len(potential_correspondence_id) == 1:
					# TODO: need to handle case for multiple potential_correspondences	
					points_for_triangulation_1.append(point.tolist())
					points_for_triangulation_2.append(feature_points_2[potential_correspondence_id].flatten().tolist())

		points_for_triangulation_1 = np.array(points_for_triangulation_1)
		points_for_triangulation_2 = np.array(points_for_triangulation_2)
		# debug
		print("pts1:")
		print(points_for_triangulation_1)
		print("pts2:")
		print(points_for_triangulation_2)

		self.reconstructPoints(points_for_triangulation_1, points_for_triangulation_2)
