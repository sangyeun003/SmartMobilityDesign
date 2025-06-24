#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point

target_x, target_y, target_z = 50.0, 50.0, 50.0
threshold = 8.0

def callback(data):
	distance = math.sqrt((target_x - data.x) ** 2 + (target_y - data.y) ** 2 + (target_z - data.z) ** 2)
	if distance <= threshold:
		rospy.loginfo("Drone arrived (%.2f, %.2f, %.2f)" % (data.x, data.y, data.z))
	else:
		rospy.loginfo(" Currently, drone is at (%.2f, %.2f, %.2f)" % (data.x, data.y, data.z))

def monitor():
	rospy.init_node('monitor', anonymous=True)

	rospy.Subscriber('/drone_position', Point, callback)
	
	rospy.loginfo("Monitoring center is ready!")
	
	rospy.spin()

if __name__ == "__main__":
	try:
		monitor()
	except rospy.ROSInterruptException:
		pass