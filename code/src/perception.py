#!/usr/bin/env python
# -*- coding: utf-8 -*-
def img_callback(data, car):
	car.image = car.bridge.imgmsg_to_cv2(data, "bgr8")
	car.img_ready = True

def lidar_callback(data, car):
	car.lidar_points = data.ranges