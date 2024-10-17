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
	Kp = 1.75
	Ki = 0.001
	Kd = 0
	t_lapstart = time.time()
	integral_clamp = .01# Anti windup clamp
	Desired = 0.2 # Zero crosstrack error is the target value
	integral_error = 0 # Initialize integral error
	previous_error = 0 # Initialize previous error for derivative control
	t_lapsed = 0
	LookaheadDist = .465
	print("initialized")
	initialize is True
	
	
def PID(msg, rightStick, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1):
	
	bot_error = Desired - rightStick
	
	integral_error = integral_error + bot_error * (t_1 - t_0)
	
	deriv_error = (bot_error - previous_error) / (t_1 - t_0)
	previous_error = bot_error
		
	controller_output = (Kp * bot_error) + (Ki * integral_error) + (Kd * deriv_error)

	velocity.angular.z = controller_output # Output of controller will be angular velocity command
	pub.publish(velocity)
	
	if controller_output > integral_clamp or controller_output < -integral_clamp: # Anti-windup trick
		integral_error = 0
	return
def END(msg, DirectLeft, DirectRight, SecondLeft, SecondRight, leftStick, rightStick):
	velocity.linear.x = 3.8 # Keep linear velocity constant and publish to /cmd_vel
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
		
	t_0 = time.time()
	rightStick = msg.ranges[315] # Find the position of the robot wrt the right wall via LiDAR data.
	leftStick = msg.ranges[45]
	frontDist = msg.ranges[0]
	SecondLeft = msg.ranges[20]
	SecondRight = msg.ranges[340]
	ThirdLeft = msg.ranges[35]
	FourthLeft = msg.ranges[30]
	DirectRight = msg.ranges[270]
	DirectLeft = msg.ranges[90] 

	t_1 = time.time()
	t_lap = time.time() - t_lapstart
	
	print("Lap Time: %f" %t_lap)
	
	if leftStick != float("inf"):
		if t_lap < 19:
			velocity.linear.x = 0.67 # Keep linear velocity constant and publish to /cmd_vel
			pub.publish(velocity)
			LookaheadDist = .465 
			if leftStick >= (Desired*1) and frontDist > LookaheadDist and SecondLeft > LookaheadDist and ThirdLeft > LookaheadDist and FourthLeft > LookaheadDist:
				print("First Sector Speed")
				PID(msg, leftStick, (-Kp * 1.58), -Ki, -Kd, (Desired * 1), integral_error, previous_error, t_0, t_1)
			else:
				print("First Sector Speed")
				PID(msg, rightStick, (Kp * 1.58), Ki, Kd, (Desired * 1), integral_error, previous_error, t_0, t_1)
				
		if t_lap >= 19 and t_lap < 24:
			velocity.linear.x = 0.52 # Keep linear velocity constant and publish to /cmd_vel
			pub.publish(velocity)
			LookaheadDist = .465 *.80
			if leftStick >= (Desired) and frontDist > LookaheadDist and SecondLeft > LookaheadDist and ThirdLeft > LookaheadDist and FourthLeft > LookaheadDist:
				PID(msg, leftStick, (-Kp*1.5), -Ki, -Kd, Desired, integral_error, previous_error, t_0, t_1)
			else:
				PID(msg, rightStick, (Kp*1.5), Ki, Kd, Desired, integral_error, previous_error, t_0, t_1)
		if t_lap > 24:
			velocity.linear.x = 0.638 # Keep linear velocity constant and publish to /cmd_vel
			pub.publish(velocity)
			LookaheadDist = .465 * 1.1
			if leftStick >= Desired and frontDist > LookaheadDist and SecondLeft > LookaheadDist and ThirdLeft > LookaheadDist and FourthLeft > LookaheadDist:
				PID(msg, leftStick, (-Kp*1.13), -Ki, -Kd, Desired, integral_error, previous_error, t_0, t_1)
			else:
				PID(msg, rightStick, (Kp*1.13), Ki, Kd, Desired, integral_error, previous_error, t_0, t_1)
	else:
		END(msg, DirectLeft, DirectRight, SecondLeft, SecondRight, leftStick, rightStick)
		
	
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
velocity = Twist()

rate = rospy.Rate(200)
while not rospy.is_shutdown():
	rate.sleep()
