#! /usr/bin/env python3
import rospy
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rospy.init_node('drive')

initialize = False # Run this once to initizalize PID parameters
if initialize is False:
	Kp = 4.6
	Ki = 0.1
	Kd = 0
	integral_clamp = .2# Anti windup clamp
	Desired = 0.27 # Zero crosstrack error is the target value
	integral_error = 0 # Initialize integral error
	previous_error = 0 # Initialize previous error for derivative control
	t_lapsed = 0
	print("initialized")
	initialize is True
	
	
def PID(msg, rightStick, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1):
	
	bot_error = Desired - rightStick
	
	integral_error = integral_error + bot_error * (t_1 - t_0)
	
	deriv_error = (bot_error - previous_error) / (t_1 - t_0)
	previous_error = bot_error
		
	controller_output = (Kp * bot_error) + (Ki * integral_error) + (Kd * deriv_error)
	print("Controller output: %f" %controller_output)
	velocity.angular.z = controller_output # Output of controller will be angular velocity command
	pub.publish(velocity)
	
	if controller_output > integral_clamp or controller_output < -integral_clamp: # Anti-windup trick
		integral_error = 0
	
		print("CLAMPED")
	return
def END(msg, DirectLeft, DirectRight, SecondLeft, SecondRight, leftStick, rightStick):
	velocity.linear.x = 0.6 # Keep linear velocity constant and publish to /cmd_vel
	velocity.angular.z = 0
	pub.publish(velocity)
	
	if DirectLeft == float("inf") and DirectRight == float("inf") and SecondLeft == float("inf") and SecondRight == float("inf") and leftStick == float("inf") and rightStick == float("inf"):
		velocity.linear.x = 0 # Keep linear velocity constant and publish to /cmd_vel
		velocity.angular.z = 0
		pub.publish(velocity)
		print("End reached!")
		rospy.spin()
	return
	
	
def callback(msg):

	#if msg.ranges[60] or msg.ranges[300] == int("inf"):
	#	velocity.linear.x = 0.15
	#	pub.publish(velocity)
		
	t_0 = time.time()
	rightStick = msg.ranges[315] # Find the position of the robot wrt the right wall via LiDAR data.
	leftStick = msg.ranges[45]
	frontDist = msg.ranges[0]
	SecondLeft = msg.ranges[10]
	SecondRight = msg.ranges[350]
	DirectRight = msg.ranges[270]
	DirectLeft = msg.ranges[90] 
	#DesiredPieSlice = (1/12) * 3.1415 * 0.3**2
	print("Right Stick: %f" %rightStick)

	t_1 = time.time()
	
	#realPieSlice = (1/12) * 3.1415 * frontDist**2
	if leftStick and rightStick != float("inf"):
		if rightStick <= 0.4 and frontDist < 0.4:
			velocity.linear.x = 0.0
			pub.publish(velocity)
			PID(msg, leftStick, -Kp, -Ki, -Kd, Desired, integral_error, previous_error, t_0, t_1)
			print("AVOIDING")
	
		if frontDist and SecondRight > 0.4:
			velocity.linear.x = 0.35 # Keep linear velocity constant and publish to /cmd_vel
			pub.publish(velocity)
			PID(msg, rightStick, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1)
	else:
		END(msg, DirectLeft, DirectRight, SecondLeft, SecondRight, leftStick, rightStick)
		
	
	#if leftStick and rightStick and frontDist == float("inf"):
	#	if leftStick and rightStick and frontDist == float("inf"):
	#		velocity.linear.x = 0 # Keep linear velocity constant and publish to /cmd_vel
	#		velocity.angular.z = 0
	#		pub.publish(velocity)
	
		

	#if frontDist < 0.25 and FiveLeft < 0.25 and leftStick < 0.20:
	#	velocity.linear.x = 0.0
	#	pub.publish(velocity)
	#	PID(msg, rightStick, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1)
	
	
	
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
velocity = Twist()

rate = rospy.Rate(50)
while not rospy.is_shutdown():
	rate.sleep()
