#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class MoveToGoal:
	
	def __init__(self):
		rospy.init_node('topic_publisher')
		self.pub = rospy.Publisher('phrases', String, queue_size=10)
		self.rate = rospy.Rate(2)
		
	def run(self):	
		msg_str = String()
		msg_str = "Hello World - ROS"

		while not rospy.is_shutdown():
			self.pub.publish(msg_str)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		publisher=MoveToGoal()
		publisher.run()
	except rospy.ROSInterruptException:
		pass
