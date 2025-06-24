#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Point

def report_position():
	rospy.init_node('drone', anonymous=True)

	pub = rospy.Publisher('/drone_position', Point, queue_size=10)
	rate = rospy.Rate(1)
	
	rospy.loginfo("Drone driving is ready!")

	msg = Point()
	while not rospy.is_shutdown():
		msg.x = random.uniform(40, 60)
		msg.y = random.uniform(40, 60)
		msg.z = random.uniform(40, 60)
		
		rospy.loginfo("current position: %.2f, %.2f, %.2f" % (msg.x, msg.y, msg.z))
		pub.publish(msg)
		rate.sleep()

if __name__ == "__main__":
	try:
		report_position()
	except rospy.ROSInterruptException:
		pass