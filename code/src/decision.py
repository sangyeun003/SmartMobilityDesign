#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import math

# LANE
def detect_lane(car, row_offset=140):
	# ROI
	ROI_HIGH=320
	ROI_LOW=460
	ROI_LEFT=0
	ROI_RIGHT=640
	ROI_HEIGHT = ROI_LOW - ROI_HIGH
	LANE_ROW = 20	# row for position detection

	# ROI image
	roi_img = car.img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]
	roi_edge_img = car.edge_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]

	# Find lines
	all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi / 180, 20, 50, 20)
	if all_lines is None:
		return False

	# Slope filtering
	slopes = []
	filtered_lines = []

	for line in all_lines:
		x1, y1, x2, y2 = line[0]

		if (x2 == x1):
			slope = 1000.0
		else:
			slope = float(y2 - y1) / float(x2 - x1)

		if 0.17 < abs(slope):
			slopes.append(slope)
			filtered_lines.append(line[0])

	# Separate LEFT & RIGHT lines
	left_lines = []
	right_lines = []
	for j in range(len(slopes)):
		line = filtered_lines[j]
		slope = slopes[j]

		x1, y1, x2, y2 = line
		# slope 0 -> 0.2 (for stoparea detection fail)
		if (slope < -0.2) and (x2 < car.WIDTH / 2):
			left_lines.append(line.tolist())
		elif (slope > 0.2) and (x1 > car.WIDTH / 2):
			right_lines.append(line.tolist())

	# LEFT: Red
	# RIGHT: Yellow
	line_draw_img = roi_img.copy()
	for line in left_lines:
		x1, y1, x2, y2 = line
		cv2.line(line_draw_img, (x1, y1), (x2, y2), (0, 0, 255), 2)	# RED
	for line in right_lines:
		x1,y1, x2, y2 = line
		cv2.line(line_draw_img, (x1, y1), (x2, y2), (0, 255, 255), 2)	# YELLOW

	# Major LEFT lines
	m_left, b_left = 0.0, 0.0
	x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
	size = len(left_lines)
	if size != 0:
		for line in left_lines:
			x1, y1, x2, y2 = line
			x_sum += x1 + x2
			y_sum += y1 + y2
			if (x2 != x1):
				m_sum += float(y2 - y1) / float(x2 - x1)
			else:
				m_sum += 0
		x_avg = x_sum / (size * 2)
		y_avg = y_sum / (size * 2)
		m_left = m_sum / size
		b_left = y_avg - m_left * x_avg

		if m_left != 0.0:
			x1 = int((0.0 - b_left) / m_left)
			x2 = int((ROI_HEIGHT - b_left) / m_left)
			cv2.line(line_draw_img, (x1, 0), (x2, ROI_HEIGHT), (255, 0, 0), 2)# BLUE

	# Major RIGHT lines
	m_right, b_right = 0.0, 0.0
	x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
	size = len(right_lines)
	if size != 0:
		for line in right_lines:
			x1, y1, x2, y2 = line
			x_sum += x1 + x2
			y_sum += y1 + y2
			if (x2 != x1):
				m_sum += float(y2 - y1) / float(x2 - x1)
			else:
				m_sum += 0
		x_avg = x_sum / (size * 2)
		y_avg = y_sum / (size * 2)
		m_right = m_sum / size
		b_right = y_avg - m_right * x_avg

		if m_right != 0.0:
			x1 = int((0.0 - b_right) / m_right)
			x2 = int((ROI_HEIGHT - b_right) / m_right)
			cv2.line(line_draw_img, (x1, 0), (x2, ROI_HEIGHT), (255, 0, 0), 2)	# BLUE

	# calculate x_left
	if m_left == 0.0:		# Detection fail
		car.x_left = car.prev_x_left
	else:
		car.x_left = int((LANE_ROW - b_left) / m_left)
		car.prev_x_left = car.x_left

	# calculate x_left
	if m_right == 0.0:		# Detection fail
		car.x_right = car.prev_x_right
	else:
		car.x_right = int((LANE_ROW - b_right) / m_right)
		car.prev_x_right = car.x_right

	car.prev_x_left = car.x_left
	car.prev_x_right = car.x_right

	x_midpoint = (car.x_left + car.x_right) // 2
	view_center = car.WIDTH // 2

	# Draw
	cv2.line(line_draw_img, (0, LANE_ROW), (car.WIDTH, LANE_ROW), (0, 255, 255), 2)
	cv2.rectangle(line_draw_img, (car.x_left - 5, LANE_ROW - 5), (car.x_left + 5, LANE_ROW + 5), (0,255,0), 4)
	cv2.rectangle(line_draw_img, (car.x_right - 5, LANE_ROW - 5), (car.x_right + 5, LANE_ROW + 5), (0,255,0), 4)
	cv2.rectangle(line_draw_img, (x_midpoint - 5, LANE_ROW - 5), (x_midpoint + 5, LANE_ROW + 5), (255,0,0), 4)
	cv2.rectangle(line_draw_img, (view_center - 5, LANE_ROW - 5), (view_center + 5, LANE_ROW + 5), (0,0,255), 4)

	# Update display_img
	car.display_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT] = line_draw_img

	return True

# STOP LINE
def is_stopline(car):
	# ROI
	ROI_HIGH=360
	ROI_LOW=460
	ROI_LEFT=0
	ROI_RIGHT=480

	# ROI Bird Eye View image
	roi_img = car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]
	roi_edge_img = car.bev_edge_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]

	# Find lines
	all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi / 180, threshold=20, minLineLength=50, maxLineGap=20)
	if all_lines is None:
		return False

	# Slope filtering
	filtered_lines = []
	for line in all_lines:
		x1, y1, x2, y2 = line[0]
		if (x2 == x1):
			slope = 1000.0
		else:
			slope = float(y2 - y1) / float(x2 - x1)

		if -0.5 <= slope <= 0.03:
			filtered_lines.append(line[0])

	# Draw lines
	line_draw_img = roi_img.copy()
	for line in filtered_lines:
		x1, y1, x2, y2 = line
		cv2.line(line_draw_img, (x1,y1), (x2, y2), (0, 0, 255), 5)	# RED

	# Decide if STOPLINE exists
	if len(filtered_lines) >= 15:
		# Update display_img
		car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT] = line_draw_img
		car.display_img = car.bev_img
		print("----STOPLINE----")
		return True
	else:
		return False

# CROSSWALK
def is_crosswalk(car):
	# ROI
	ROI_HIGH=240
	ROI_LOW=460
	ROI_LEFT=0
	ROI_RIGHT=640

	# ROI Bird Eye View image
	roi_img = car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]
	roi_edge_img = car.bev_edge_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]

	# Find contours for CROSSWALK
	_, contours, _ = cv2.findContours(roi_edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if contours is None:
		return False

	# Draw CROSSWALK
	crosswalk_mark = 0
	line_draw_img = roi_img.copy()
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)	# find polygon
		if len(approx) == 4 and cv2.isContourConvex(approx) and cv2.contourArea(cnt) > 10:	# find rectangle
			x, y, w, h = cv2.boundingRect(approx)
			aspect_ratio = float(w) / h
			cv2.drawContours(line_draw_img, [approx], -1, (0, 255, 0), 5)
			cv2.rectangle(line_draw_img, (x, y), (x + w, y + h), (255, 0, 0), 10)
			crosswalk_mark += 1

	# Decide if CROSSWALK exists
	if crosswalk_mark >= 5:
		# Update display_img
		car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT] = line_draw_img
		car.display_img = car.bev_img
		print("----CROSSWALK----")
		return True
	else:
		return False

# STOPAREA
def is_stoparea(car):
	# ROI
	ROI_HIGH=110
	ROI_LOW=400
	ROI_LEFT=80
	ROI_RIGHT=360

	# ROI Bird Eye View image
	roi_img = car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]
	roi_edge_img = car.bev_edge_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT]

	# Find lines
	all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi / 180, threshold=20, minLineLength=100, maxLineGap=30)
	if all_lines is None:
		return False

	# Slope filtering
	filtered_lines = []
	for line in all_lines:
		x1, y1, x2, y2 = line[0]
		if (x2 == x1):
			slope = 1000.0
		else:
			slope = float(y2 - y1) / float(x2 - x1)

		if 0.05 < slope < 0.7:
			filtered_lines.append(line[0])

	# Draw lines
	line_draw_img = roi_img.copy()
	for line in filtered_lines:
		x1, y1, x2, y2 = line
		cv2.line(line_draw_img, (x1,y1), (x2, y2), (0, 0, 255), 3)	# RED

	# Decide if STOPAREA exists
	if len(filtered_lines) >= 10:
		# Update display_img
		car.bev_img[ROI_HIGH:ROI_LOW, ROI_LEFT:ROI_RIGHT] = line_draw_img
		car.display_img = car.bev_img
		print("----STOP AREA----")
		return True
	else:
		return False

# TUNNEL
def is_tunnel(car):
	# ROI for LiDAR
	ANGLE_START = 20			# degree
	ANGLE_END = 80				# degree
	DISTANCE_THRESHOLD = 0.5	# meter
	COUNT_THRESHOLD = 10

	l_count = 0
	r_count = 0
	car.tunnel_left = 0			# left avg distance
	car.tunnel_right = 0		# right avg distance

	# Count VALID lidar_points
	for degree in range(2 * ANGLE_START, 2 * ANGLE_END):
		if not math.isnan(car.lidar_points[degree]) and not math.isinf(car.lidar_points[degree]) and (0.01 < car.lidar_points[degree] <= DISTANCE_THRESHOLD):
			car.tunnel_right += car.lidar_points[degree]
			r_count += 1
		if not math.isnan(car.lidar_points[719 - degree]) and not math.isinf(car.lidar_points[719 - degree]) and (0.01 < car.lidar_points[719 - degree] <= DISTANCE_THRESHOLD):
			car.tunnel_left += car.lidar_points[719 - degree]
			l_count += 1

	# Avg distance
	car.tunnel_right = car.tunnel_right / r_count if r_count > 0 else 0
	car.tunnel_left = car.tunnel_left / l_count if l_count > 0 else 0

	# Decide if TUNNEL exists
	if l_count > COUNT_THRESHOLD and r_count > COUNT_THRESHOLD:
		print("----TUNNEL----")
		return True
	else:
		return False

# OBSTACLE
def is_obstacle(car):
	# no obstacle: return False
	# LEFT/RIGHT obstacle: return True

	# ROI for LiDAR
	ANGLE_START = 20
	ANGLE_END = 60
	DISTANCE_THRESHOLD = 0.4
	BIG_COUNT_THRESHOLD = 20
	SMALL_COUNT_THRESHOLD = 10

	l_count = 0
	r_count = 0
	l_avg = 0			# left avg distance
	r_avg = 0			# right avg distance

	# Count VALID lidar_points
	for degree in range(2 * ANGLE_START, 2 * ANGLE_END):
		if not math.isnan(car.lidar_points[degree]) and not math.isinf(car.lidar_points[degree]) and 0.01 < car.lidar_points[degree] <= DISTANCE_THRESHOLD:
			r_count += 1
			r_avg += car.lidar_points[degree]
		if not math.isnan(car.lidar_points[719 - degree]) and not math.isinf(car.lidar_points[719 - degree]) and 0.01 < car.lidar_points[719 - degree] <= DISTANCE_THRESHOLD:
			l_count += 1
			l_avg += car.lidar_points[719 - degree]

	# Decide if OBSTACLE exists
	if l_count > BIG_COUNT_THRESHOLD and r_count < SMALL_COUNT_THRESHOLD:
		car.obstacle = -1
		print("LEFT OBSTACLE!")
		return True
	elif r_count > BIG_COUNT_THRESHOLD and l_count < SMALL_COUNT_THRESHOLD:
		car.obstacle = 1
		print("RIGHT OBSTACLE!")
		return True
	else:
		car.obstacle = 0
		return False