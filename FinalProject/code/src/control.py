#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy

from decision import *
from utils import *

# Track lane
def track_lane(car):
	# Show display_img
	show_img(car)

	# Calculate Midpoint
	x_midpoint = (car.x_left + car.x_right) // 2
	view_center = car.WIDTH // 2

	# Control car
	angle = PID(car, x_midpoint, view_center)
	motor_control(car, angle, car.speed)

# Wait for 5 sec on CROSSWALK
def wait_crosswalk(car):
	print("-------WAIT CROSSWALK-------")
	# Wait for 5 sec
	for i in range(32):
		# Update image
		image_preprocess(car)
		is_crosswalk(car)
		show_img(car)
		# Control car
		motor_control(car, angle=0, speed=0)
		time.sleep(0.1)
	print("-------CROSSWALK END-------")

	# Go straight for 0.8 sec not to detect CROSSWALK again
	for i in range(8):
		# Update image
		image_preprocess(car)
		show_img(car)
		# Control car
		motor_control(car, angle=0)
		time.sleep(0.1)

# Avoid obstacles
def avoid_obstacle(car):
	# Avoid angle
	ANGLE = 15
	angle = ANGLE if car.obstacle == -1 else -ANGLE
	print("AVOID OBSTACLE : " + ("LEFT" if car.obstacle == -1 else "RIGHT"))

	# ANGLE
	for i in range(4):
		# Update image
		image_preprocess(car)
		is_stopline(car)
		show_img(car)
		# Control car
		motor_control(car, angle)
		time.sleep(0.1)
	# STRAIGHT
	for i in range(2):
		# Update image
		image_preprocess(car)
		is_stopline(car)
		show_img(car)
		# Control car
		motor_control(car, 0)
		time.sleep(0.1)
	# -ANGLE
	for i in range(4):
		# Update image
		image_preprocess(car)
		is_stopline(car)
		show_img(car)
		# Control car
		motor_control(car, -angle)
		time.sleep(0.1)

# Drive in TUNNEL
def tunnel_drive(car):
	# CONSTANT
	K = 200

	print("-------TUNNEL START-------")
	while is_tunnel(car):
		# Update image
		image_preprocess(car)
		detect_lane(car)
		show_img(car)
		# Control car
		angle = int((-car.tunnel_left + car.tunnel_right) * K)	# Compare LEFT avg distance to RIGHT avg distance
		angle = PID(car, angle, 0)	# PID
		motor_control(car, angle=angle)

		print("angle: ", angle)
	print("-------TUNNEL END-------")

	for i in range(5):
		detect_lane(car)
		track_lane(car)
		time.sleep(0.1)

# Finish driving on STOPAREA
def finish_driving(car):
	# Go straight for 1 sec not to detect STOPLINE again
	for i in range(10):
		# Update image
		image_preprocess(car)
		is_stoparea(car)
		show_img(car)
		# Control car
		motor_control(car, angle=-5)
		time.sleep(0.1)
	# Go until find STOPLINE
	while not is_stopline(car):
		# Update image
		image_preprocess(car)
		is_stoparea(car)
		show_img(car)
		# Control car
		motor_control(car, angle=-3)
	# Stop & Wait 3 sec
	for i in range(30):
		# Update image
		image_preprocess(car)
		is_stopline(car)
		show_img(car)
		# Control car
		motor_control(car, angle=0, speed=0)
		time.sleep(0.1)
	# EXIT
	rospy.signal_shutdown("FINISH DRIVING")