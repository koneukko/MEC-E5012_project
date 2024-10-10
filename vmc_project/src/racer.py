#! /usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# ham hammy
class Driving(): # main class

	def __init__(self):
		global trackdrive
		trackdrive = Twist()
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10) # Publish velocity command
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback) # Subscribe to Lidar data
			

	def callback(self, msg):
		print("The front LiDAR reading is: %f" %msg.ranges[0])
		print("The left LiDAR reading is : %f" %msg.ranges[90])
		print("The right LiDAR reading is: %f" %msg.ranges[270])
		
		
		self.distance = 0.198
		#LidarRangeRight = [1] * 45
		#for i in range(44):
			#rangeval = int(i + 315)
			#LidarRangeRight[i] = msg.ranges[rangeval]
			# i =+ 1
			
		# LidarRangeRight = msg.ranges[345,346,347,348,349,350,351,352,353,354,355,356,357,358,359]
		if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance:
		# clear path
			trackdrive.linear.x = 0.2 # Forward through the track
			trackdrive.angular.z = 0 # No rotation
		if msg.ranges[270] < self.distance:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.1
			trackdrive.angular.z = 1.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[315] > self.distance:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.1
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[323] < self.distance:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.1
			trackdrive.angular.z = 1.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[330] < self.distance:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.1
			trackdrive.angular.z = 1.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		#if LidarRangeRight < self.distance + 0.1:
		if msg.ranges[337] < self.distance:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.04
			trackdrive.angular.z = 1.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[345] < self.distance + 0.15:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[347] < self.distance + 0.12:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[350] < self.distance + 0.1:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[353] < self.distance + 0.1:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[355] < self.distance + 0.1:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[356] < self.distance + 0.1:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[357] < self.distance + 0.12:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[358] < self.distance + 0.12:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[359] < self.distance + 0.12:
		# path blocked by track wall at 275 deg
			trackdrive.linear.x = 0.01
			trackdrive.angular.z = 2.8 # Rotation to the left (ccw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[90] < self.distance:
		#path blocked by track wall at 90 deg
			trackdrive.linear.x = 0.1
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[45] > self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = 1.8 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[41] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[37] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[30] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[23] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[17] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.06
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[15] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.04
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[12] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.04
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[9] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.04
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[7] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[5] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[3] < self.distance:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		if msg.ranges[1] < self.distance - 0.02:
		#path blocked by track wall at 45 deg
			trackdrive.linear.x = 0.02
			trackdrive.angular.z = -1.0 # Rotation to the right (cw)
			if msg.ranges [0] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
				# path is clear again
				trackdrive.linear.x = 0.2
				trackdrive.angular.z = 0 # No rotation
		#if msg.ranges[90] == float("inf") and msg.ranges[270] == float("inf"):
		#	trackdrive.linear.x = 0 # End of track
		#	trackdrive.angular.z = 0 # End of track
		self.pub.publish(trackdrive) # Publish command for forward movement
		
		
if __name__ == '__main__':
	rospy.init_node('trackdriving_node') #initialise
	rate = rospy.Rate(100)
	Driving()
	rospy.spin()
			
			




#sub = rospy.Subscriber('/scan', LaserScan, callback)
#pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

#vel = Twist() 
#vel.linear.x = 0.1
#rate = rospy.Rate(100)
#while not rospy.is_shutdown():
#	pub.publish(vel)
#	rate.sleep()
