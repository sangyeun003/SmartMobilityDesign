#!/usr/bin/env python
import rospy
import time

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

Imu_msg = None

def imu_callback(data):
	global Imu_msg
	Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

def determine_direction(current_yaw, prev_yaw):
	diff = current_yaw - prev_yaw

	if diff > 3.14:
		diff -= 6.28
	elif diff < -3.14:
		diff += 6.28

	if abs(diff) < 0.02:
		return "Straight"
	elif diff > 0:
		return "Left"
	else:
		return "Right"

def send_direction():
	rospy.init_node("imu_direction_monitor")
	rospy.Subscriber("/imu", Imu, imu_callback)
	pub = rospy.Publisher("/direction", String, queue_size=10)

	prev_yaw = None
	rate = rospy.Rate(1)  # 1Hz
	while not rospy.is_shutdown():
		if Imu_msg == None:
			continue

		(roll, pitch, yaw) = euler_from_quaternion(Imu_msg)

		if prev_yaw is None:
			prev_yaw = yaw
			continue

		direction = determine_direction(yaw, prev_yaw)
		prev_yaw = yaw

		rospy.loginfo("Yaw: %.4f, Direction: %s" % (yaw, direction))
		pub.publish(String(data=direction))

		rate.sleep()

if __name__ == "__main__":
	try:
		send_direction()
	except rospy.ROSInterruptException:
		pass