#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

def img_callback(data):
	global image, img_ready, bridge

	image = bridge.imgmsg_to_cv2(data, "bgr8")
	img_ready = True

def main():
	global image, img_ready, bridge

	rospy.init_node("image_saver_node")
	rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

	image = np.empty(shape=[0])
	img_ready = None
	bridge = CvBridge()

	save_path = "/home/nvidia/xycar_ws/src/final/src/calibration/chess_images/"

	print("Press 's' to save image. Press 'q' to quit.")
	
	count = 0
	while not rospy.is_shutdown():
		if img_ready:
			cv2.imshow("Image", image)
			key = cv2.waitKey(1) & 0xFF

			if key == ord('s'):
				filename = os.path.join(save_path, "image" + str(count) + ".png")
				success = cv2.imwrite(filename, image)
				if success:
					print("Saved ", filename)
					count += 1
				else:
					print("Fail...")
			elif key == ord('q'):
				print("Exiting...")
				break
	
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()