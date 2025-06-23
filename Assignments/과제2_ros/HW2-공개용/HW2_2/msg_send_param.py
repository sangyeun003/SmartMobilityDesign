#!/usr/bin/env python
import rospy
from my_msg.msg import my_msg

def send():
	rospy.init_node('msg_sender', anonymous=True)
	pub = rospy.Publisher('msg_to_xycar', my_msg, queue_size=10)

	msg = my_msg()

	msg.first_name = rospy.get_param('~firstname')
	msg.last_name = rospy.get_param('~lastname')
	msg.id_number = rospy.get_param('~id')
	msg.phone_number = rospy.get_param('~phone')

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		pub.publish(msg)
		print("sending message")
		rate.sleep()

if __name__ == "__main__":
	try:
		send()
	except rospy.ROSInterruptException:
		pass
