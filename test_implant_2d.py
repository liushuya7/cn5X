# Standard imports
import cv2
import numpy as np;
 
# Read image
im = cv2.imread("/home/parto/Desktop/img/color12.png", cv2.IMREAD_GRAYSCALE)
 
params=cv2.SimpleBlobDetector_Params();
# Change thresholds
params.minThreshold = 80
# self.params.maxThreshold = 225;

#filter to find bright dot
params.filterByColor=True;
params.blobColor=0;

#filter by area
params.filterByArea = True;
params.minArea = 3 
# self.params.maxArea = 60

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
print("here")
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
cv2.destroyAllWindows()