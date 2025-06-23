#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import glob
import os

def calibrate_camera(image_folder, checkerboard_dims):
	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# prepare object points (0,0,0), (1,0,0), ..., (9,6,0) for 10x7 board
	objp = np.zeros((checkerboard_dims[0]*checkerboard_dims[1], 3), np.float32)
	objp[:,:2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1, 2)

	objpoints = []  # 3d point in real world space
	imgpoints = []  # 2d points in image plane

	images = glob.glob(os.path.join(image_folder, '*.png'))
	
	cnt = 1
	for fname in images:
		img = cv2.imread(fname)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)

		# If found, add object points, image points (after refining them)
		if ret:
			objpoints.append(objp)
			corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners2)

			# Draw and display the corners
			cv2.drawChessboardCorners(img, checkerboard_dims, corners2, ret)
			cv2.imshow('Chessboard', img)
			print(cnt, fname)
			cnt += 1
			cv2.waitKey(0)

	cv2.destroyAllWindows()

	if len(objpoints) > 0:
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
		h, w = 480, 640
		new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
		print("Calibration successful!\n")
		print("Camera matrix:")
		print(mtx)
		print("Distortion coefficients:")
		print(dist)
		print("New camera matrix:")
		print(new_camera_mtx)
		print("roi:")
		print(roi)
		return mtx, dist, rvecs, tvecs, new_camera_mtx, roi
	else:
		print("No corners found. Calibration failed.")
		return None, None, None, None, None, None

if __name__ == "__main__":
	folder_path = "/home/nvidia/xycar_ws/src/final/src/calibration/chess_images/"
	checkerboard_size = (10, 7)  # columns, rows
	mtx, dist, rvecs, tvecs, new_camera_mtx, roi = calibrate_camera(folder_path, checkerboard_size)