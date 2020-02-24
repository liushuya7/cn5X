import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread, QTimer
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox, QFileDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic

import cv2
import numpy as np

# debug
import pickle
import sys
sys.path.append("calibration_tool/")
from Camera_Calibration import Camera_Calibration
import utils

from CameraThread import CameraThread
from ImageWindowThread import ImageWindowThread
from TwoViewEstimate import TwoViewEstimate
from ImageProcessing import ImageProcessing

self_dir = os.path.dirname(os.path.realpath(__file__))
SAVE_PATH = os.path.join(self_dir, '../img')

class CameraDialog(QDialog):
	''' Camera dialog '''
	def __init__(self):
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

	def startVideo(self):
		self.camera_thread = CameraThread()
		self.camera_thread.start()

	def captureImage(self):

		# make save dir if not existed
		if not os.path.isdir(SAVE_PATH):
			os.makedirs(SAVE_PATH,exist_ok=True)
		if self.__dl.checkBox_save.isChecked():
			file_name = os.path.join(SAVE_PATH, str(self.saved_file).zfill(2)+'.png')
			print(file_name)
			self.camera_thread.saveImage(file_name)
			self.saved_file += 1
		else:
			file_filter = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
			file_name,_ = QFileDialog.getSaveFileName(self, 'Save File', directory=SAVE_PATH,filter=file_filter)
			if file_name != '':
				self.camera_thread.saveImage(file_name)

	def closeCamera(self):
		if self.camera_thread and self.camera_thread.running:
			self.stopThreadSafely(self.camera_thread)

	def closeWindow(self):
		self.closeCamera()

	def matchFeatures(self):

		def load_calib(file_name):
			# for reading also binary mode is important 
			dbfile = open(file_name, 'rb')      
			db = pickle.load(dbfile) 
			dbfile.close() 
			return db

		def get_pose_calib(calibration_result, camera_id):

			R, _ = cv2.Rodrigues(calibration_result.rvecs[camera_id]);
			t = calibration_result.tvecs[camera_id];
			H_cam_to_board = np.concatenate((R, t.reshape((3,1))),axis=1);
			H_cam_to_board = np.concatenate((H_cam_to_board, np.array([0,0,0,1]).reshape((1,4))),axis=0);
			H_board_to_cam = np.linalg.inv(H_cam_to_board);

			return H_board_to_cam;

		# debug: use camera poses for previous case
		file_name = os.path.join(self_dir, '../my_calib')
		my_calib = load_calib(file_name)
		calibration_result = my_calib.cal_result; # R and t is from camera to chessboard, R is vector form, t is in mm

		# get image file names
		file_filter = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
		file_name = QFileDialog()
		file_name.setFileMode(QFileDialog.ExistingFiles)
		names, _ = file_name.getOpenFileNames(self, "Open files", SAVE_PATH, file_filter)

		# store images and open ImageWindow
		for name in names:
			img_name = os.path.split(name)[1] # extract the tail part for window name
			img = cv2.imread(name)
			# debug
			self.imgs.append(img)

			thread = ImageWindowThread(img_name, img)
			self.window_threads.append(thread)

		# preparation for two-view stereo
		img1 = self.imgs[0];
		img2 = self.imgs[1];
		img1_pose_id = 10;
		img2_pose_id = 11;
		# get camera poses from calibration result 
		H_board_to_cam_1 = get_pose_calib(calibration_result, img1_pose_id);
		H_board_to_cam_2 = get_pose_calib(calibration_result, img2_pose_id);

		self.two_view_estimate = TwoViewEstimate(img1, img2, calibration_result.mtx, calibration_result.dist, H_board_to_cam_1, H_board_to_cam_2);

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
			self.two_view_estimate.estimate(self.window_threads[0].window.points_list, self.window_threads[1].window.points_list);
		else:
			self.two_view_estimate.estimate(pts1, pts2)

	def automaticReconstruction(self):
		
		### Temporary test code
		def load_calib(file_name):
			# for reading also binary mode is important 
			dbfile = open(file_name, 'rb')      
			db = pickle.load(dbfile) 
			dbfile.close() 
			return db

		def get_pose_calib(calibration_result, camera_id):

			R, _ = cv2.Rodrigues(calibration_result.rvecs[camera_id]);
			t = calibration_result.tvecs[camera_id];
			H_cam_to_board = np.concatenate((R, t.reshape((3,1))),axis=1);
			H_cam_to_board = np.concatenate((H_cam_to_board, np.array([0,0,0,1]).reshape((1,4))),axis=0);
			H_board_to_cam = np.linalg.inv(H_cam_to_board);

			return H_board_to_cam;

		# debug: Temporary test code to extract camera poses 
		file_name = os.path.join(self_dir, '../my_calib')
		my_calib = load_calib(file_name)
		calibration_result = my_calib.cal_result # R and t is from camera to chessboard, R is vector form, t is in mm

		# get image file names
		file_filter = "png (*.png);;jpg (*.jpg);;PNG (*.PNG);;JPG (*.JPG)"
		file_name = QFileDialog()
		file_name.setFileMode(QFileDialog.ExistingFiles)
		names, _ = file_name.getOpenFileNames(self, "Open files", SAVE_PATH, file_filter)
		# store images TODO: a way to associate image, poses, image_id for two-view and multi-view neatly
		for i,name in enumerate(names):
			img_name = os.path.split(name)[1] # extract the tail part for window name, formatted as "name_ID.png"
			# extract id of image 
			img_id = int(img_name.split('.')[0].split('_')[-1])
			img = cv2.imread(name)
			thread = ImageWindowThread(img_name, img)
			self.window_threads.append(thread)
			self.imgs_key_to_id.append(img_id)
			self.imgs[img_id] = img
			self.poses[img_id] = get_pose_calib(calibration_result, img_id)

		self.two_view_estimate = TwoViewEstimate(self.imgs[self.imgs_key_to_id[0]], self.imgs[self.imgs_key_to_id[1]],\
			 									calibration_result.mtx, calibration_result.dist,\
												self.poses[self.imgs_key_to_id[0]], self.poses[self.imgs_key_to_id[1]])
		image_processor_1 = ImageProcessing(self.two_view_estimate.img1_rectified)
		feature_points_1, img_with_keypoint_1 = image_processor_1.extractFeaturePoints()
		image_processor_2 = ImageProcessing(self.two_view_estimate.img2_rectified)
		feature_points_2, img_with_keypoint_2 = image_processor_2.extractFeaturePoints()
		feature_points_1 = np.array(feature_points_1)
		feature_points_2 = np.array(feature_points_2)
		# feature_points should be sorted by pixel-y-coordinate in decreasing order by default
		# extract intersection points, set threshold to be 2 pixel for now
		points_for_triangulation_1 = []
		points_for_triangulation_2 = []
		threshold = 2
		for point in feature_points_1:
			difference = abs(feature_points_2 - point)
			potential_correspondence_id, _ = np.where(difference < threshold)
			if len(potential_correspondence_id) == 1:
				# TODO: need to handle case for multiple potential_correspondences	
				points_for_triangulation_1.append(point.tolist())
				points_for_triangulation_2.append(feature_points_2[potential_correspondence_id].flatten().tolist())
		points_for_triangulation_1 = np.array(points_for_triangulation_1)
		points_for_triangulation_2 = np.array(points_for_triangulation_2)
		print("pts1:")
		print(points_for_triangulation_1)
		print("pts2:")
		print(points_for_triangulation_2)
		self.reconstructPoints(points_for_triangulation_1, points_for_triangulation_2)

		self.window_threads[0].window.img = img_with_keypoint_1
		self.window_threads[1].window.img = img_with_keypoint_2
		for thread in self.window_threads:
			thread.start()