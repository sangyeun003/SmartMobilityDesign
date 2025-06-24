#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import numpy as np
from functools import partial
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from perception import *
from decision import *
from control import *
from utils import *

class Car():
	def __init__(self, speed=5, p=0.8, i=0, d=0):
		# ROS init
		rospy.init_node('Xycar')

		# perception
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, partial(img_callback, car=self))
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, partial(lidar_callback, car=self))

		# control
		self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
		self.speed = speed
		self.angle = None

		# PID
		self.p = p
		self.i = i
		self.d = d
		self.i_error = 0.0
		self.prev_error = 0.0

		# image
		self.image = np.empty(shape=[0])
		self.img_ready = None
		self.bridge = CvBridge()

		self.img = None
		self.display_img = None
		self.gray = None
		self.blur_gray = None
		self.binary = None
		self.edge_img = None
		self.bev_img = None
		self.bev_edge_img = None

		self.WIDTH = 640
		self.HEIGHT = 480

		# lidar
		self.lidar_points = None

		# line detection
		self.prev_x_left = 0
		self.prev_x_right = self.WIDTH
		self.x_left = 0
		self.x_right = self.WIDTH

		# obstacle
		self.obstacle = 0	# None: 0, Left: -1, Right: 1

		# tunnel
		self.tunnel_left = 0
		self.tunnel_right = 0

		# time
		self.start_time = time.time()
		self.end_time = time.time()

	def drive(self):
		print("Self-driving start!")
		motor_control(car, speed=0, angle=0)

		while not rospy.is_shutdown():
			while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
				continue
			while self.img_ready == False:
				continue
			while self.lidar_points is None:
				continue

			# Image preprocessing
			image_preprocess(car)

			# STOPLINE
			if is_stopline(self):
			 	# CROSSWALK
				if is_crosswalk(self):
					wait_crosswalk(self)
			 	# STOP AREA
				elif is_stoparea(self):
					finish_driving(self)
			# TUNNEL
			elif is_tunnel(self):
				tunnel_drive(self)
			# OBSTACLE
			elif is_obstacle(self) != 0:
				avoid_obstacle(self)
			# LANE
			detect_lane(self)
			track_lane(self)

if __name__ == "__main__":
	car = Car(speed=5)
	car.drive()
	print("EXIT")