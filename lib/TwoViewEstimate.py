import cv2
import os
import numpy as np
import csv

self_dir = os.path.dirname(os.path.realpath(__file__))
SAVE_PATH = os.path.join(self_dir, '../data')

class TwoViewEstimate(object):
    def __init__(self, img1, img2, K, dist, camera_pose_1, camera_pose_2):
        self.img1 = img1;
        self.img2 = img2;
        self.camera_pose_1 = camera_pose_1;
        self.camera_pose_2 = camera_pose_2;
        self.K = K;
        self.dist = dist;

        shape = self.img1.shape;
        self.img_size = (shape[1],shape[0]);
        self.P1, self.R1, self.P2, self.R2, self.img1_rectified, self.img2_rectified = self.stereoRectify();
        self.img1_matching = self.img1_rectified.copy();
        self.img2_matching = self.img2_rectified.copy();

        self.matched_points = [];


    def findRandT(self):
        # R and T are rotation and translation from camera2 to camera1
        # H = np.matmul(np.linalg.inv(self.camera_pose_1), self.camera_pose_2);
        H = np.matmul(np.linalg.inv(self.camera_pose_2), self.camera_pose_1);
        R = H[:3, :3];
        T = H[:3, 3];

        return (R, T);


    def findFundamentalMatrix(self):
        (R, T) = self.findRandT();
        E = np.matmul(self.skew(T), R);

        K_temp_2 = np.linalg.inv(self.K).T;
        K_temp_1 = np.linalg.inv(self.K);
        F = np.matmul(np.matmul(K_temp_2, E), K_temp_1);

        return F;

    def computeEpipolarLine(self, F, point2D):

        point = list(point2D)
        point.append(1);
        point = np.array(point)

        L = np.matmul(F, point);
        y_intercept = int(-L[2] / L[1]);
        x_intercept = int(-L[2] / L[0]);
        point1 = (x_intercept, 0);
        point2 = (0, y_intercept);
        if x_intercept < 0:
            print("special case")
            slope = -L[0] / L[1];

        cv2.line(self.img2, point1, point2, (0,0,255), thickness=5);

        return (L, point1, point2)

    def computeEpipolarLineRectified(self, point2D, color):
        cv2.line(self.img2_rectified, (0,point2D[1]), (self.img_size[0], point2D[1]), color, thickness=10);
        cv2.line(self.img1_rectified, (0,point2D[1]), (self.img_size[0], point2D[1]), color, thickness=10);
        location = self.findMatching(point2D);

    def findMatching(self, point2D):
        mask_window_radius = 10; 
        match_window_radius = 50; 
        img1_gray = cv2.cvtColor(self.img1_matching, cv2.COLOR_BGR2GRAY);
        img2_gray = cv2.cvtColor(self.img2_matching, cv2.COLOR_BGR2GRAY);
        template = img1_gray[point2D[1] - match_window_radius : point2D[1] + match_window_radius, point2D[0] - match_window_radius:point2D[0] + match_window_radius];# imread reverse width and height
        cv2.imwrite("template.png", template)

        # create mask of ROI for feature detection
        mask = np.zeros(img2_gray.shape, dtype=np.uint8);
        mask[point2D[1] - mask_window_radius : point2D[1] + mask_window_radius, :] = 1;
        # good feature to track
        corners = cv2.goodFeaturesToTrack(img2_gray, 0, 0.05, 10, mask=mask);
        print("number of corners:" + str(len(corners)))
        corners = np.int0(corners);
        scores =[];
        # find best matching corner
        for i, corner in enumerate(corners):
            x,y = corner.ravel();
            to_match = img2_gray[y-match_window_radius:y+match_window_radius, x-match_window_radius:x+match_window_radius];
            score = self.computeScore(template, to_match);
            scores.append(score);
            # debug
            # if to_match.shape == template.shape:
            #     cv2.imwrite(str(i)+'.png', to_match)
            # else:
            #     print("fail to crop images")
        print(scores)
        scores = np.array(scores);
        best_match = np.argmax(scores);
        print("max:" + str(scores[best_match]))
        location = corners[best_match];
        x,y = location.ravel()
        matched_img = img2_gray[y-match_window_radius:y+match_window_radius, x-match_window_radius:x+match_window_radius];
        cv2.imwrite("matched_img.png", matched_img);

        # # ORB
        # orb = cv2.ORB_create();
        # kp = orb.detect(img2_gray, None);
        # kp, des = orb.compute(img2_gray, kp);
        # img2_keypt = cv2.drawKeypoints(img2_gray, kp, None, color=(0,255,0), flags=0);
        # cv2.imwrite("img2_kp.png", img2_keypt)

        return location;

    def stereoMatching(self, point2D, color):
        # self.computeEpipolarLineRectified(point2D, color);
        location = self.findMatching(point2D);
        print(location)
        x,y = location.ravel()
        cv2.circle(self.img2_rectified, (x,y), 10, color , thickness= 5);
        self.matched_points.append((x,y));
    
    def computeScore(self, temp1, temp2):

        ## normalized cross-correlation coefficient
        # product = np.mean(temp1-temp1.mean() * temp2-temp2.mean());
        # stds = temp1.std()*temp2.std();
        # if stds == 0:
        #     return 0;
        # else:
        #     product /= stds;
        #     return product;
        if temp1.shape != temp2.shape:
            score = 0;
        else:
            score = ssim(temp1, temp2);
        return score

    def triangulation(self, P1, P2, pts_1, pts_2):
        # reconstruct the points from 2D point sets, P1 and P2 are projection matrices
        # note that dtype of P1, P2, pts_1, and pts_2 should be float64 (numpy)

        pts_1 = pts_1.astype(np.float);
        pts_2 = pts_2.astype(np.float);

        pts_3d_reconstr = cv2.triangulatePoints(P1, P2, pts_1.T, pts_2.T);
        # reconstructed 3d points homogeneous coordinates
        pts_3d_reconstr /= pts_3d_reconstr[3, :];

        return pts_3d_reconstr.T;

    def transformToCamera1(self, pts):
        # 3D pts is in homogeneous coordinage of shape (n,4)
        R_homogeneous = np.concatenate((self.R1, np.array([0,0,0]).reshape((3,1))), axis=1)
        R_homogeneous = np.concatenate((R_homogeneous, np.array([0,0,0,1]).reshape((1,4))), axis=0)
        transformed_pts = np.matmul(np.linalg.inv(R_homogeneous), pts.T)

        return transformed_pts.T


    def projectPoints(self, pts):
        # 3D pts is in homogeneous coordinage of shape (n,4)
        camera_matrix = np.concatenate((self.K, np.array([0,0,0]).reshape((3,1))), axis=1)
        points2D = np.matmul(camera_matrix, pts.T)
        # points2D = np.matmul(self.P1, pts.T)
        points2D /= points2D[2, :]

        return points2D

    def estimate(self, pts_1, pts_2):
        # F = self.findFundamentalMatrix();
        # (L, point1, point2) = self.computeEpipolarLine(F, pts_1);
        # return (L, point1, point2)

        pts_3d = self.triangulation(self.P1, self.P2, np.array(pts_1), np.array(pts_2));
        pts_3d = self.transformToCamera1(pts_3d)
        points_file = os.path.join(SAVE_PATH, "points_3d_stereo.csv")
        # check save path
        if not os.path.isdir(SAVE_PATH):
            os.makedirs(SAVE_PATH, exist_ok=True)
        with open(points_file, 'wt') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            for point in pts_3d:
                writer.writerow(point[:3])
        
        # # project points for verification
        pts_2d = self.projectPoints(pts_3d)
        img_projected = self.img1.copy()
        for pt_2d in pts_2d.T:
            pt_2d = pt_2d[:2] 
            img = cv2.circle(img_projected, tuple(pt_2d.astype(np.int32)),3, color=(255,0,0), thickness=5, lineType=cv2.FILLED)
        cv2.imwrite('img1.png', self.img1)
        cv2.imwrite('project_to_img1.png', img)


    def stereoRectify(self):
        (R, T) = self.findRandT();

        R1, R2, P1, P2, Q, ROI1, ROI2 = cv2.stereoRectify(self.K, self.dist, self.K, self.dist, self.img_size, R, T)#, alpha=0);
        map_x_1, map_y_1 = cv2.initUndistortRectifyMap(self.K, self.dist, R1, P1, self.img_size, cv2.CV_16SC2);
        map_x_2, map_y_2 = cv2.initUndistortRectifyMap(self.K, self.dist, R2, P2, self.img_size, cv2.CV_16SC2);
        img1_rectified = cv2.remap(self.img1, map_x_1, map_y_1, cv2.INTER_LANCZOS4);
        img2_rectified = cv2.remap(self.img2, map_x_2, map_y_2, cv2.INTER_LANCZOS4);

        return (P1, R1, P2, R2, img1_rectified, img2_rectified);

    def skew(self, vector):
        vector_temp = vector.flatten()
        return np.array([[0, -vector_temp[2], vector_temp[1]], 
                     [vector_temp[2], 0, -vector_temp[0]], 
                     [-vector_temp[1], vector_temp[0], 0]])