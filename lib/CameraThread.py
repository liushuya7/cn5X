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
		print("start realsense!")

	def stop(self):
		print("close realsense!")
		self.pipe.stop()


class CameraThread(QThread):
	def __init__(self,image_view=None):
		super().__init__()
		self.running = True
		self.realsense = RealSense(rs.stream.color, 1920, 1080, 30)
		self.image_view = image_view

		# cv2 window
		self.window_names = {"realsense":"RealSense Live Stream", "multiview":"MultiView Reconstruction Window"}

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

				color_image = np.asanyarray(color_frame.get_data())
				self.image_view.setImage(color_image)
				# self.msleep(1)

				# cv2.imshow(self.window_name, color_image)

				#  # if the 'c' key is pressed, break from the loop
				# key = cv2.waitKey(1) & 0xFF
				# if key == ord("c"):
				#     break
			finally:
				pass
				# print("finished1")
		print("finished2")