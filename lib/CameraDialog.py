import os
from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtWidgets import QDialog, QAbstractButton, QDialogButtonBox, QCheckBox
from PyQt5.QtGui import QStandardItemModel, QStandardItem, QValidator
from PyQt5 import uic
from cn5X_config import *
from grblCom import grblCom
from msgbox import *
from qweditmask import qwEditMask

import pyrealsense2 as rs
import cv2
import numpy as np
import pyqtgraph

from compilOptions import grblCompilOptions

self_dir = os.path.dirname(os.path.realpath(__file__))


class RealSense():
	def __init__(self, stream_type, width, height, fps):
		# Declare RealSense pipeline, encapsulating the actual device and sensors
		self.pipe = rs.pipeline()
		# Build config object and stream everything
		self.cfg = rs.config()
		self.cfg.enable_stream(stream_type, width, height, rs.format.rgb8, fps)
		self.pipe.start(self.cfg)
		print("start realsense!")

	def stop(self):
		print("close realsense!")
		self.pipe.stop()


class CameraThread(QThread):
	def __init__(self, window_name=None, image_view=None):
		super().__init__()
		self.running = True
		self.realsense = RealSense(rs.stream.color, 1920, 1080, 30)
		self.image_view = image_view

		# cv2 window
		# self.window_name = window_name
		# self.window = cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)


	def run(self):
		while self.running:
			try:
				# Retreive the stream and intrinsic properties for both cameras
				# profiles = self.pipe.get_active_profile()
				# stream = profiles.get_stream(rs.stream.color).as_video_stream_profile();
				# intrinsic = stream.get_intrinsics();

				frames = self.realsense.pipe.wait_for_frames()
				color_frame = frames.get_color_frame()
				if not color_frame:
					print("no frame!!!!")

				print("running")
				color_image = np.asanyarray(color_frame.get_data())
				self.image_view.setImage(color_image)
				# self.msleep(1)

				# cv2.imshow(self.window_name, color_image)
				# cv2.waitKey(1)

				#  # if the 'c' key is pressed, break from the loop
				# key = cv2.waitKey(1) & 0xFF
				# if key == ord("c"):
				#     break
			finally:
				pass
				# print("finished1")
		print("finished2")


class CameraDialog(QDialog):
	''' Camera dialog '''
	def __init__(self):
		super().__init__()
		ui_dlgCamera = os.path.join(self_dir, '../ui/camera.ui')
		self.image_view = pyqtgraph.ImageView()
		self.__dl = uic.loadUi(ui_dlgCamera, self)
		self.__dl.layout.addWidget(self.image_view)
		self.finished.connect(self.closeWindow)
		self.show()

		self.__dl.pushButton_start.clicked.connect(self.startVideo)
		self.__dl.pushButton_stop.clicked.connect(self.closeWindow)

	def startVideo(self):
		self.camera_thread = CameraThread(image_view=self.image_view)
		self.camera_thread.start()

	def closeWindow(self):
		if self.camera_thread.running:
			self.camera_thread.running = False
			self.camera_thread.quit()
			self.camera_thread.wait(3000)
			print("finished:  " + str(self.camera_thread.isFinished()))
			self.camera_thread.realsense.stop()
