from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot, QThread
import cv2
import numpy as np
import csv

class ImageWindow(object):
	def __init__(self, name, img, multiview_estimate=None):
		self.name = name;
		self.img = img;
		window_size = np.array(img.shape[:2]).T
		self.points_list = [];
		self.color = (0,0,255);#None;
		self.multiview_estimate = multiview_estimate;
		cv2.namedWindow(self.name, cv2.WINDOW_NORMAL);
		cv2.resizeWindow(self.name, window_size[0], window_size[1])
		cv2.setMouseCallback(self.name,self.click);

	def click(self, event, x, y, flags, param):
		# if the left mouse button was clicked, record 
		# (x, y) coordinates 
		if event == cv2.EVENT_LBUTTONDOWN:
			point = (x,y)
			self.points_list.append(point);
			color = np.random.randint(0, 256, 3);
			self.color = (int(color[0]), int(color[1]), int(color[2]))
			cv2.circle(self.img, (x,y), 5, self.color , thickness= 2);
			cv2.putText(self.img, str(len(self.points_list)), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color)
			cv2.imshow(self.name, self.img);
			if self.multiview_estimate:
				self.multiview_estimate.stereoMatching(point, self.color);
				# self.multiview_estimate.computeEpipolarLineRectified(point, self.color);

	def showWindow(self):
		cv2.imshow(self.name, self.img);
		cv2.waitKey(1)

	def savePoints(self):
		clicked_points_file = self.name+ "_points_2d.csv"
		with open(clicked_points_file, 'wt') as stream:
			writer = csv.writer(stream, lineterminator='\n')
			for point in self.points_list:
				writer.writerow(point);


class ImageWindowThread(QThread):
	def __init__(self, window_name, image):
		super().__init__()
		self.running = True
		self.window_name = window_name
		self.img_raw = image
		self.window = ImageWindow(name=self.window_name, img=self.img_raw)

	def run(self):
		while self.running:
			try:
				self.window.showWindow()
			finally:
				pass
		print("finished: " + self.window_name)

	def closeWindow(self):
		cv2.destroyWindow(self.window_name)