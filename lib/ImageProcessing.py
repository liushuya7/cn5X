import cv2
import numpy as np
import time

class ImageProcessing(object):
    def __init__(self, img):
        self.img = img 

    def extractROI(self, img):
        
        ret, labels = cv2.connectedComponents(img)
        array = np.matrix.flatten(labels)
        # debug later
        # labels_counts_order_decreasing = np.flip(np.argsort(np.bincount(array)[1::], kind='stable'))
        # max_label = labels_counts_order_decreasing[1] + 1
        max_label = np.bincount(array)[1::].argmax() + 1
        for label in range(1, ret):
            if (label == max_label):
                mask = np.array(labels, dtype=np.uint8)
                mask[labels == label] = 255
                mask[labels != label] = 0
        return mask

    def thresholding(self, img):
        
        ret1, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_TOZERO)
        # ret1, thresh = cv2.threshold(img, 170, 255, cv2.THRESH_TOZERO)
        gray = thresh
        binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        return binary

    def blobDetect(self, img):
        #img = self.img
        # set up for blob detector
        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 50
        # self.params.maxThreshold = 225

        #filter to find bright dot
        params.filterByColor=True
        params.blobColor=0

        #filter by area
        params.filterByArea = True
        params.minArea = 1 #30 
        params.maxArea = 500

        # Set up the detector with default parameters
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(img)
        center_list = []
        for points in keypoints:
            center_list.append([points.pt[0], points.pt[1]])
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        img_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return center_list, img_with_keypoints


    def extractFeaturePoints(self):

        gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        t0 = time.time()
        binay_img = self.thresholding(gray_img)
        t1 = time.time()
        print("time for thresholding: " + str(t1-t0))

        t0 = time.time()
        roi_img = self.extractROI(binay_img)
        t1 = time.time()
        print("time for extracting ROI: " + str(t1-t0))

        t0 = time.time()
        feature_pionts, img_with_keypoints = self.blobDetect(roi_img)
        t1 = time.time()
        print("time for blob detection: " + str(t1-t0))

        return feature_pionts, img_with_keypoints


def main():
    file_name = "../rectified2.png"
    img = cv2.imread(file_name)
    image_processor = ImageProcessing(img)
    feature_pionts, img_with_keypoints = image_processor.extractFeaturePoints()
    cv2.imshow("", img_with_keypoints)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()