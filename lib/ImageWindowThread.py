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
            cv2.circle(self.img, (x,y), 10, self.color , thickness= 5);
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


class ImageWindowThread(object):
    def __init__(self):
        super().__init__(window_name)

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