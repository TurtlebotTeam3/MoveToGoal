#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
import copy
import time

from simple_odom.msg import PoseConverted, CustomPose
from move_to_goal.srv import Move, MoveResponse

from math import pow, atan2, sqrt

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan


class MoveToGoal:

	def __init__(self):
		rospy.on_shutdown(self._shutdown)
		rospy.init_node('move_to_goal')

		self.stop = False

		self.rotation_speed = 0.75
		self.forward_speed = 0.175
		self.distance_tolerance = 0.02

		self.pose = Pose()
		self.pose_converted = PoseConverted()

		self.rate = rospy.Rate(10)
		self.obstacle_in_front = False
		self.obstacle_in_back = False
		self.oldtheta = 0
		self.pause_action = False
		self.send_paused_update = False
		self.goal_pose = Pose()
		self.goal_to_approach = False

		self.last_pose_time = None

		self.avoid_obstacle_enabled = rospy.get_param("~avoid_obstacle",default=False)
		
		# --- publisher ---
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.goal_reached_publisher = rospy.Publisher('move_to_goal/reached', Bool, queue_size=1)
		self.paused_publisher = rospy.Publisher('move_to_goal/paused',	Bool, queue_size=1)
		
		# --- subscriber ---
		self.pose_subscriber = rospy.Subscriber('simple_odom_pose',	CustomPose, self._handle_update_pose)
		self.scanSub = rospy.Subscriber('scan', LaserScan, self._handle_scan_data)
		self.goal_subscriber = rospy.Subscriber('move_to_goal/goal', Pose, self._update_goal)
		self.pause_subscriber = rospy.Subscriber('move_to_goal/pause_action',
												Bool, self._pause_action)
		# --- service ---
		self.drive_back_and_rotate_service = rospy.Service('drive_back_and_rotate', Move, self._drive_back_and_rotate_360)

		rospy.loginfo('ready')

	def _shutdown(self):
		"""
		Shut down procedure. Stops the motors -> makes it safer
		"""
		self.stop = True
		self._set_motor_speed(0,0)

	def _handle_scan_data(self, scan):
		"""
		Handles the data received on the laser scan topic.
		"""
		self._check_obstacle_infront(scan.ranges)

	def _check_obstacle_infront(self, scan_data):
		"""
		Checks if there is an obstacle in front of the robot
		"""
		range_front = []
		min_front = 0
		# in front of the robot (between 20 to -20 degrees)
		range_front[:20] = scan_data[-20:]
		range_front[20:] = scan_data[:20]
		range_front = list(filter(lambda num: num != 0, range_front))
		min_front = min(range_front)

		range_back = []
		min_back = 0
		# in front of the robot (between 20 to -20 degrees)
		range_back[:20] = scan_data[-20:]
		range_back[20:] = scan_data[:20]
		range_back = list(filter(lambda num: num != 0, range_back))
		min_back = min(range_back)

		if self.avoid_obstacle_enabled:
			if min_front < 0.3 and min_front != 0.0:
				if not self.obstacle_in_front:
					rospy.loginfo('Obstacle in Front')
				self.obstacle_in_front = True
			else:
				self.obstacle_in_front = False
		else:
			if min_front < 0.25 and min_front != 0.0:
				if not self.obstacle_in_front:
					rospy.loginfo('Obstacle in Front')
				self.obstacle_in_front = True
			else:
				self.obstacle_in_front = False

		if min_back < 0.3 and min_back != 0.0:
			if not self.obstacle_in_back:
				rospy.loginfo('Obstacle in Back')
			self.obstacle_in_back = True
		else:
			self.obstacle_in_back = False


	def _handle_update_pose(self, data):
		"""
		Update current pose of robot
		"""
		try:	
			self.last_pose_time = int(round(time.time() * 1000))

			self.pose_converted = data.pose_converted
			self.pose = data.pose
		except:
			rospy.loginfo('transform not ready')
		
	def _pause_action(self, data):
		"""
		Pause / Continue current action
		"""
		# True when to pause and False when to continue
		self.send_paused_update = True
		self.pause_action = data.data

	def _update_goal(self, goal_pose):
		"""
		New goal to approach
		"""
		print('New goal' + str(goal_pose.position.x) + ' | ' + str(goal_pose.position.y))
		self.goal_pose = goal_pose
		self.goal_to_approach = True


	def _move_to_goal(self):
		"""
		Drive towards the currently set goal
		"""
		cancel = False
		goal_reached = self._euclidean_distance(self.goal_pose) <= self.distance_tolerance

		while not goal_reached and not self.stop and not cancel:

			current_time_ms = int(round(time.time() * 1000))
			# check if network is working properly
			if not self.pause_action and self.last_pose_time != None and (current_time_ms - self.last_pose_time) < 50:
				
				if self.send_paused_update:
					self.paused_publisher.publish(False)
					self.send_paused_update = False
				
				# Forward speed
				v_forward = 0
				# Rotation speed
				v_rotate = 0

				# rotate towards goal
				if abs(self._steering_angle(self.goal_pose) - self.pose_converted.yaw) > 0.2:
					v_rotate = self._angular_vel(self.goal_pose)
				# drive forward towards goal
				else:
					if not self.obstacle_in_front:
						v_forward = self._linear_vel(self.goal_pose)
					else:
						self._stop_motors()
						if self.avoid_obstacle_enabled:
							# try to avoid obstacle by 90 rotation to right -> move 1,5 robot width forward -> 90 rotation to left -> move 1,5 robot width forward
							self._avoid_obstacle()
						# cancel to trigger recalculation
						cancel = True
						rospy.loginfo('cancel')

				# Set speed
				self._set_motor_speed(v_forward, v_rotate)

				goal_reached = self._euclidean_distance(self.goal_pose) <= self.distance_tolerance
			else:
				if self.send_paused_update:
					self._stop_motors()
					self.paused_publisher.publish(True)
					self.send_paused_update = False

		# Stopping our robot after the movement is over.
		self._stop_motors()
		if not cancel:
			self.goal_reached_publisher.publish(True)
			self.goal_to_approach = False
		else:
			self.goal_reached_publisher.publish(False)
			self.goal_to_approach = False

	def _avoid_obstacle(self):
		"""
		Avoids an obstacle by driving around it
		"""
		rospy.loginfo('Avoiding obstacle')
		# move back word
		self._move_straight_x(-0.15)
		# Rotate 90 to the left
		while self.obstacle_in_front:
			self._rotate_x_degrees(self.rotation_speed, 90, False)
		# Drive 1,5  robot widt forward
		self._move_straight_x(0.15)
		rospy.loginfo('Avoiding obstacle finished')

	def _rotate_x_degrees(self, speed_rad_sec, rotate_in_degrees, clockwise):
		"""
		Rotate robot

		Parameters:
		speed_rad_sec (float): Rotation speed in rad/sec
		rotate_in_degrees (int): Rotation angle in degree
		clockwise (bool): True = Clockwise, False = counterclockwise

		Returns:
		success (bool): True if successful executed
		"""
		#Converting from angles to radians
		relative_angle = rotate_in_degrees*2*math.pi/360

		v_rotate = 0

		# Checking if our movement is CW or CCW
		if clockwise:
			v_rotate = -abs(speed_rad_sec)
		else:
			v_rotate = abs(speed_rad_sec)

		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while current_angle < relative_angle and not self.pause_action:
			current_time_ms = int(round(time.time() * 1000))
			if self.last_pose_time != None and (current_time_ms - self.last_pose_time) < 50:
				self._set_motor_speed(0,v_rotate)
				t1 = rospy.Time.now().to_sec()
				current_angle = speed_rad_sec*(t1-t0)
			else:
				self._stop_motors()

		#Forcing our robot to stop
		self._stop_motors()

		if current_angle < relative_angle:
			return False
		else:
			return True

	def _move_straight_x(self, distance):
		"""
		Move forward a definded distance:

		Parameters:
		distance (float): Distance in meters

		Returns:
		success (bool): True if successful executed
		"""
		# Calculate end points
		next_x = round(self.pose.position.x, 4) + math.cos(self.pose_converted.yaw) * distance
		next_y = round(self.pose.position.y, 4) + math.sin(self.pose_converted.yaw) * distance

		# create copy and only change the x and y position to keep rotation
		goal_pose = copy.deepcopy(self.pose)
		goal_pose.position.x = next_x
		goal_pose.position.y = next_y

		# drive until specified distance is driven
		while not (self._euclidean_distance(goal_pose) <= self.distance_tolerance) and not self.pause_action:
			current_time_ms = int(round(time.time() * 1000))
			if self.last_pose_time != None and (current_time_ms - self.last_pose_time) < 50:
				if distance < 0:
					if not self.obstacle_in_back:
						self._set_motor_speed(-self._linear_vel(),0)
					else:
						break
				else:
					if not self.obstacle_in_front:
						self._set_motor_speed(self._linear_vel(),0)
					else:
						break
			else:
				self._stop_motors()

		self._stop_motors()

		if (self._euclidean_distance(goal_pose) <= self.distance_tolerance):
			return True
		else:
			return False

	def _drive_back_and_rotate_360(self, data):
		"""
		Handle service request. Drive back 20cm and rotate 360 degree
		"""
		rospy.loginfo('Start drive back and rotate')
		success = True
		success &= self._move_straight_x(-0.2)
		# Rotate 360 to the right
		success &= self._rotate_x_degrees(self.rotation_speed, 360, True)

		return_vel = Bool()
		return_vel.data =  success

		if self.send_paused_update and self.pause_action and not success:
			self._stop_motors()
			self.paused_publisher.publish(True)
			self.send_paused_update = False

		rospy.loginfo('Finished drive back and rotate - success: ' + str(success))

		return MoveResponse(return_vel)

	def _stop_motors(self):
		"""
		Stops activ rotation and forward movement
		"""
		self._set_motor_speed(0,0)

	def _set_motor_speed(self, forward_speed, rotation_speed):
		"""
		Publish speeds to robot.
		"""
		vel_msg = Twist()
		# Linear velocity in the x-axis.
		vel_msg.linear.x = forward_speed
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		# Angular velocity in the z-axis.
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = rotation_speed
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()

	def _euclidean_distance(self, goal_pose):
		"""
		Calculates the distance between robot and goal
		"""
		return sqrt(pow((goal_pose.position.x - round(self.pose.position.x, 4)), 2) +
					pow((goal_pose.position.y - round(self.pose.position.y, 4)), 2))

	def _linear_vel(self, goal_pose=None, constant=0.4):
		"""
		Forward speed
		"""
		return self.forward_speed

	def _steering_angle(self, goal_pose):
		"""
		Calculate the angle between goal and robot
		"""
		y = goal_pose.position.y - round(self.pose.position.y, 4)
		x = goal_pose.position.x - round(self.pose.position.x, 4)
		
		theta = atan2(y, x)
		
		return theta

	def _angular_vel(self, goal_pose, constant=0.3):
		"""
		Rotation speed
		"""
		steer_angle = self._steering_angle(goal_pose)
		
		yaw_2 = (self.pose_converted.yaw + 2*math.pi) % (2*math.pi)
		steer_angle_2 = (steer_angle + 2*math.pi) % (2*math.pi)
		angle_2pi = (steer_angle_2 - yaw_2) % (2*math.pi)
		
		if angle_2pi < math.pi:
			# rotate robot to the left
			return self.rotation_speed
		else:
			# rotate robot to the right
			return -1 * self.rotation_speed

	def run(self):
		"""
		Runs in an endless loop and checks if there is a goal to approach until shutdown
		"""
		while not rospy.is_shutdown():
			if self.goal_to_approach:
				self._move_to_goal()

if __name__ == '__main__':
	try:
		moveToGoal = MoveToGoal()
		moveToGoal.run()
	except rospy.ROSInterruptException:
		pass
