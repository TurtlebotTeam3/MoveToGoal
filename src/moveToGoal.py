#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from math import pow, atan2, sqrt
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._Pose import Pose
from nav_msgs.msg._Odometry import Odometry
from std_msgs.msg._Bool import Bool
import tf


class MoveToGoal:

	def __init__(self):
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

		self.distance_tolerance = 0.03
		self.pose = Pose()
		self.rate = rospy.Rate(20)
		rospy.spin()


	def _update_pose(self, data):
		"""
		Update current pose of robot
		"""
		self.pose = data.pose.pose
		self.pose.position.x = round(data.pose.pose.position.x, 4)
		self.pose.position.y = round(data.pose.pose.position.y, 4)
		self.robot_yaw = self._robot_angle()

	def _update_goal(self, goal_pose):
		"""
		New goal to approach
		"""
		
		print('New goal' + str(goal_pose.position.x) + ' | ' + str(goal_pose.position.y))

		vel_msg = Twist()

		while self._euclidean_distance(goal_pose) >= self.distance_tolerance:
			if abs(self._steering_angle(goal_pose) - self.robot_yaw) > 0.1:
				print('rotate')
				# Linear velocity in the x-axis.
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				if self._angular_vel(goal_pose) > 0:
					vel_msg.angular.z = 0.2
				else:
					vel_msg.angular.z = -0.2
				
				# vel_msg.angular.z = self._angular_vel(goal_pose)

			else:
				print('Forward')
				# Linear velocity in the x-axis.
				vel_msg.linear.x = self._linear_vel(goal_pose)
				#vel_msg.linear.x = 0.2
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

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
		self.rate.sleep()
		self.rate.sleep()
		self.goal_reached_publisher.publish(True)

	def _euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
					pow((goal_pose.position.y - self.pose.position.y), 2))

	def _linear_vel(self, goal_pose, constant=0.4):
		return constant * self._euclidean_distance(goal_pose)

	def _steering_angle(self, goal_pose):
		return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)

	def _robot_angle(self):
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
		return yaw

	def _angular_vel(self, goal_pose, constant=0.3):
		yaw = self._robot_angle()
		steer_angle = self._steering_angle(goal_pose)
		return constant * (steer_angle - yaw)

if __name__ == '__main__':
	try:
		moveToGoal=MoveToGoal()
	except rospy.ROSInterruptException:
		pass
