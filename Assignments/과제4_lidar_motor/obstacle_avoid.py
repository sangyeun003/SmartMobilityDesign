#!/usr/bin/env python
import rospy, time, random
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

# CONSTANT
SCAN_ANGLE_RANGE = 23		# 23degree
DISTANCE_THRESHOLD = 0.65	# 0.6m

SPEED = 3				# 0~5
ANGLE = 35					# 0~50(absolute value)

class Car():
	def __init__(self):
		rospy.init_node('lidar_driver')
		rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1)
		self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

		self.motor_msg = xycar_motor()
		self.lidar_points = None

	def callback(self, data):
		self.lidar_points = data.ranges

	def drive(self, speed, angle):
		self.motor_msg.speed = speed
		self.motor_msg.angle = angle
		self.pub.publish(self.motor_msg)

	def obstacle_detected(self):
		count = 0
		for degree in range(0, 2 * SCAN_ANGLE_RANGE):
			if 0.01 < self.lidar_points[degree] <= DISTANCE_THRESHOLD:
				count += 1
			if 0.01 < self.lidar_points[719 - degree] <= DISTANCE_THRESHOLD:
				count += 1
		return count > 5

	def avoid_obstacle(self):
		direction = random.choice(["left", "right"])
		angle = ANGLE if direction == "right" else -ANGLE
		
		rospy.loginfo("[Avoiding] direction: " + direction)
		
		# 1. L or R
		for i in range(15):
			self.drive(SPEED, angle)
			time.sleep(0.1)
		
		# 2. straight
		for i in range(10):
			self.drive(SPEED, 0)
			time.sleep(0.1)
		
		# 3. R or L
		for i in range(30):
			self.drive(SPEED, -angle)
			time.sleep(0.1)
		
		# 4. straight
		for i in range(10):
			self.drive(SPEED, 0)
			time.sleep(0.1)
		
		# 5. L or R
		for i in range(15):
			self.drive(SPEED, angle)
			time.sleep(0.1)
		
		rospy.loginfo("Avoid complete")



def main():
	car = Car()
    
	while car.lidar_points is None:
		continue

	cleared = None
	while not rospy.is_shutdown():
		if car.obstacle_detected():
			rospy.loginfo("Obstacle detected! Stop!")
			car.drive(0, 0)

			start_time = time.time()
			cleared = False

			while time.time() - start_time < 5:
				if not car.obstacle_detected():
					cleared = True
					rospy.loginfo("Obstacle eliminated! Go straight!")
					car.drive(SPEED, 0)
					break

			if not cleared:
				rospy.loginfo("5sec over. Avoid")
				car.avoid_obstacle()
		else:
			rospy.loginfo("No obstacles! Go straight!")
			car.drive(SPEED, 0)


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
