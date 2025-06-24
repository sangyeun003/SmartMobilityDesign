#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np

from xycar_msgs.msg import xycar_motor

# Publish motor_msg
def motor_control(car, angle, speed=10):
	motor_msg = xycar_motor()
	motor_msg.angle = angle
	motor_msg.speed = speed

	car.motor_pub.publish(motor_msg)

# Image preprocess
def image_preprocess(car):
	car.img = car.image.copy()
	car.display_img = car.img		# to show
	car.img_ready = False

	# Calibration
	camera_matrix = np.array([[352.15, 0, 315.03],
						[0, 351.72, 216.93],
						[0, 0, 1]])
	dist_coeffs = np.array([-0.36, 0.2, 0.002, -0.002, 0])
	new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (car.WIDTH, car.HEIGHT), 0.4, (car.WIDTH, car.HEIGHT))
	car.img = cv2.undistort(car.img, camera_matrix, dist_coeffs, None, new_camera_mtx)

	# Gray scale
	car.gray = cv2.cvtColor(car.img, cv2.COLOR_BGR2GRAY)
	# Gaussian Blur
	car.blur_gray = cv2.GaussianBlur(car.gray, (5, 5), 0)
	# Canny edge detection
	car.edge_img = cv2.Canny(np.uint8(car.blur_gray), 50, 100)
	# Bird Eye View
	car.bev_img, car.bev_edge_img = bev(car)

# Bird Eye View
def bev(car):
	h, w = (480, 640)
	src = np.float32([
		[140, 240],	# LEFT UP
		[500, 240],	# RIGHT UP
		[40, 420],	# LEFT DOWN
		[600, 420]	# RIGHT DOWN
	])
	dst = np.float32([
		[0, 0],
		[640, 0],
		[0, 480],
		[640, 480]
	])
	M = cv2.getPerspectiveTransform(src, dst)

	return cv2.warpPerspective(car.img, M, (w, h)), cv2.warpPerspective(car.edge_img, M, (w, h))

# Show display_img
def show_img(car):
	cv2.imshow("car-Driving", car.display_img)
	cv2.waitKey(1)

# PID angle
def PID(car, input_data, target_data):
	car.end_time = time.time()
	dt = car.end_time - car.start_time
	car.start_time = car.end_time

	error = target_data - input_data
	derror = error - car.prev_error

	p_error = car.p * error
	car.i_error = car.i_error + car.i * error * dt
	d_error = car.d * derror / dt if dt != 0 else 0

	output = p_error + car.i_error + d_error
	car.prev_error = error

	if output > 50:
		output = 50
	elif output < -50:
		output = -50

	return -output