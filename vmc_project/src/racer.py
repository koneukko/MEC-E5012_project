#! /usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rospy.init_node('drive')

initialize = False # Run this once to initizalize PID parameters
if initialize is False:
	Kp = 1.7
	Ki = 0.2
	Kd = 0
	integral_clamp = .05 # Anti windup clamp
	Desired = 0 # Zero crosstrack error is the target value
	integral_error = 0 # Initialize integral error
	previous_error = 0 # Initialize previous error for derivative control
	initialize is True
	
def PID(msg, ct_diff, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1):
	
	bot_error = Desired - ct_diff
	
	integral_error = integral_error + bot_error * (t_1 - t_0)
	
	deriv_error = (bot_error - previous_error) / (t_1 - t_0)
	previous_error = bot_error
		
	controller_output = (Kp * bot_error) + (Ki * integral_error) + (Kd * deriv_error)
	print("Controller output: %f" %controller_output)
	velocity.angular.z = -controller_output # Output of controller will be angular velocity command
	pub.publish(velocity)
	
	if controller_output > integral_clamp or controller_output < -integral_clamp: # Anti-windup trick
		integral_error = 0
	
	return
		
def callback(msg):
	
	t_0 = time.time()
	ct_diff = msg.ranges[90] - msg.ranges[270] # Find the position of the robot wrt the walls via LiDAR data. 
	
	print("Crosstrack difference: %f" %ct_diff)

	
	if msg.ranges[90] != float("inf") and msg.ranges[270] != float("inf"):
		velocity.linear.x = 0.2 # Keep linear velocity constant and publish to /cmd_vel
		pub.publish(velocity)
		t_1 = time.time()
		PID(msg, ct_diff, Kp, Ki, Kd, Desired, integral_error, previous_error, t_0, t_1)
	else:
		velocity.linear.x = 0
		velocity.angular.z = 0
		pub.publish(velocity)
		print("End of track reached!")
		rospy.spin()


sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
velocity = Twist()

rate = rospy.Rate(50)
while not rospy.is_shutdown():
	rate.sleep()
