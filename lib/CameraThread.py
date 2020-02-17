from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread
import pyrealsense2 as rs
import cv2
import numpy as np
import csv

class RealSense():
	def __init__(self, stream_type, width, height, fps):
		# Declare RealSense pipeline, encapsulating the actual device and sensors
		self.pipe = rs.pipeline()
		# Build config object and stream everything
		self.cfg = rs.config()
		self.cfg.enable_stream(stream_type, width, height, rs.format.rgb8, fps)
		self.pipe.start(self.cfg)
		self.running = True
		print("start realsense!")

	def stop(self):
		if self.running:
			print("close realsense!")
			self.pipe.stop()
			self.running = False
		else:
			print("realsense is closed already!")


class CameraThread(QThread):
	def __init__(self):
		super().__init__()
		self.running = True
		self.realsense = RealSense(rs.stream.color, 1920, 1080, 30)
		self.window_size = (1920, 1080)
		self.window_name = "RealSense Live Video"
		self.img = None
		self.window = cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
		cv2.resizeWindow(self.window_name, self.window_size[0], self.window_size[1])
	
	def run(self):
		while self.running:
			try:
				frames = self.realsense.pipe.wait_for_frames()
				color_frame = frames.get_color_frame()
				if not color_frame:
					print("no frame!!!!")

				color_image = np.asanyarray(color_frame.get_data())
				color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
				self.img = color_image

				cv2.imshow(self.window_name, color_image)
				cv2.waitKey(1)
				# # if the 'c' key is pressed, break from the loop ### NOT WORKING
				# key = cv2.waitKey(1) & 0xFF
				# if key == ord("c"):
				# 	self.running = False
				# 	break

				# catch the event that window is closed by user ### NOT WORKING!!
				if cv2.getWindowProperty(self.window_name,1) < 0:
					print("manually close window!!!")
					self.running = False
					break
			finally:
				pass
		self.closeWindow()
		print("finished!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

	def closeWindow(self):
		cv2.destroyWindow(self.window_name)
		self.realsense.stop()

	def saveImage(self, name):
		if self.img is not None: # cannot use self.img directly because it may be a numpy array, which needs a.any() or a.all() for "if"
			cv2.imwrite(name, self.img)
		else:
			print("No image available to save!")
