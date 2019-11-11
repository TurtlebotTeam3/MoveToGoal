#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class Subscriber:

	def __init__(self):
		rospy.init_node('topic_subscriber') # needs to be the first call
		self.sub = rospy.Subscriber('/phrases', String, self.callback)

		rospy.spin() # needs to be the last in the constructor

	def callback(self, msg):
		print(msg.data)

if __name__ == '__main__':
	try:
		subscriber=Subscriber()
	except rospy.ROSInterruptException:
		pass