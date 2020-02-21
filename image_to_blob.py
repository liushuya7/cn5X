import cv2
import numpy as np

img = cv2.imread("img_test/color24.png", cv2.IMREAD_GRAYSCALE)
ret1, thresh = cv2.threshold(img, 170, 255, cv2.THRESH_TOZERO)

gray = thresh
binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

ret, labels = cv2.connectedComponents(binary)
countlist = [0 for i in range(ret)]
for column in labels:
    for element in column:
        if element > 0:
            countlist[element - 1] += 1
for label in range(1, ret):
    if (countlist[label - 1] > 30000):
        mask = np.array(labels, dtype=np.uint8)
        mask[labels == label] = 255
        mask[labels != label] = 0

im = mask
 
params=cv2.SimpleBlobDetector_Params();
# Change thresholds
params.minThreshold = 50
# self.params.maxThreshold = 225;

#filter to find bright dot
params.filterByColor=True;
params.blobColor=0;

#filter by area
params.filterByArea = True;
params.minArea = 30 
params.maxArea = 500

# # Filter by Circularity
# params.filterByCircularity = True
# params.minCircularity = 0.9
# # Filter by Convexity
# params.filterByConvexity = True
# params.minConvexity = 0.87
# #filter by Inertia
# params.filterByInertia = True
# params.minInertiaRatio = 0.5

# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector_create(params)

 
# Detect blobs.
keypoints = detector.detect(im)
center_list = []
for points in keypoints:
    center_list.append([points.pt[0], points.pt[1]])
print (center_list)
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
cv2.destroyAllWindows()