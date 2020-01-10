#! /usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import String
from math import pow, atan2, sqrt
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._Pose import Pose
from nav_msgs.msg._Odometry import Odometry
from std_msgs.msg._Bool import Bool
from sensor_msgs.msg._LaserScan import LaserScan
import tf


class MoveToGoal:

	def __init__(self):
		rospy.on_shutdown(self._shutdown)
		rospy.init_node('move_to_goal')
		self.pub = rospy.Publisher('phrases', String, queue_size=10)
		
		self.velocity_publisher = rospy.Publisher('cmd_vel', 
													Twist, queue_size=10)

		self.pose_subscriber = rospy.Subscriber('/odom',
												Odometry, self._update_pose)

		self.goal_subscriber = rospy.Subscriber('/move_to_goal/goal',
												Pose, self._update_goal)
		
		self.goal_reached_publisher = rospy.Publisher('move_to_goal/reached', 
													Bool, queue_size=1)
		self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

		self.stop = False
		self.distance_tolerance = 0.03
		self.pose = Pose()
		self.rate = rospy.Rate(20)
		self.obstacle = False
		self.oldtheta = 0
		print('--- ready ---')
		rospy.spin()

	def _shutdown(self):
		self.stop = True
		vel_msg = Twist()
		# Linear velocity in the x-axis.
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		# Angular velocity in the z-axis.
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()

	def _scan_callback(self, scan):
		if scan.ranges[0] < 0.2:
			self.obstacle = True
		else:
			self.obstacle = False

	def _update_pose(self, data):
		"""
		Update current pose of robot
		"""
		self.pose = data.pose.pose
		self.pose.position.x = round(data.pose.pose.position.x, 4)
		self.pose.position.y = round(data.pose.pose.position.y, 4)
		self.robot_yaw = self._robot_angle()

	def _robot_angle(self):
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
		return yaw

	def _update_goal(self, goal_pose):
		"""
		New goal to approach
		"""
		
		print('New goal' + str(goal_pose.position.x) + ' | ' + str(goal_pose.position.y))

		vel_msg = Twist()

		cancle = False

		while self._euclidean_distance(goal_pose) >= self.distance_tolerance and not self.stop and not cancle:
			# Linear velocity in the x-axis.
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0

			if abs(self._steering_angle(goal_pose) - self.robot_yaw) > 0.1:
				print('rotate')
				vel_msg.angular.z = self._angular_vel(goal_pose)
			else:
				print('Forward')
				if not self.obstacle:
					vel_msg.linear.x = self._linear_vel(goal_pose)
				else:
					cancle = True
					vel_msg.linear.x = 0
					vel_msg.angular.z = 0
					self.velocity_publisher.publish(vel_msg)

			# Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)

			# Publish at the desired rate.
			self.rate.sleep()
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()
		if not cancle:
			self.goal_reached_publisher.publish(True)
		else:
			self.goal_reached_publisher.publish(False)

	def _euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
					pow((goal_pose.position.y - self.pose.position.y), 2))

	def _linear_vel(self, goal_pose, constant=0.4):
		return constant * self._euclidean_distance(goal_pose)

	def _steering_angle(self, goal_pose):
		y = goal_pose.position.y - self.pose.position.y
		x = goal_pose.position.x - self.pose.position.x
		
		theta = atan2(y, x)
		
		return theta

	def _angular_vel(self, goal_pose, constant=0.3):
		yaw = self._robot_angle()
		steer_angle = self._steering_angle(goal_pose)
		#return constant * (steer_angle - yaw)
		
		yaw_2 = (yaw + 2*math.pi) % (2*math.pi)
		steer_angle_2 = (steer_angle + 2*math.pi) % (2*math.pi)
		angle_2pi = (steer_angle_2 - yaw_2) % (2*math.pi)
		
		if angle_2pi < math.pi:
			# rotate robot to the left
			return constant * (2*math.pi - angle_2pi)
		else:
			# rotate robot to the right
			return constant * (angle_2pi - 2*math.pi)


if __name__ == '__main__':
	try:
		moveToGoal=MoveToGoal()
	except rospy.ROSInterruptException:
		pass
