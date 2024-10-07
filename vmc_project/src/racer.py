#! /usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        

        self.distance = 0.19
        #All of the below is left as comments for now
#        if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance:
#        # clear path
#            trackdrive.linear.x = 0.4 # Forward through the track
#            trackdrive.angular.z = 0 # No rotation
#        if msg.ranges[275] < self.distance:
#        # path blocked by track wall at 275 deg
#            trackdrive.linear.x = 0.35
#            trackdrive.angular.z = 1.2 # Rotation to the left (ccw)
#            if msg.ranges [0] > self.distance and msg.ranges[30] > self.distance and msg.ranges[330] > self.distance:
#                # path is clear again
#                trackdrive.linear.x = 0.4
#                trackdrive.angular.z = 0 # No rotation
#        if msg.ranges[85] < self.distance:
#        #path blocked by track wall at 85 deg
#            trackdrive.linear.x = 0.35
#            trackdrive.angular.z = -1.2 # Rotation to the right (cw)
#            if msg.ranges [0] > self.distance and msg.ranges[30] > self.distance and msg.ranges[330] > self.distance:
#                # path is clear again
#                trackdrive.linear.x = 0.4
#                trackdrive.angular.z = 0 # No rotation

        #initalize the distance
        print(type(msg.ranges))
        Gamma=np.argmin(msg.ranges)+180
        print("Smallest distance index (DEG):",Gamma)
        theta=Gamma-90

        #Smallest distandce to the wall and it's opposite
        Least = msg.ranges[np.argmin(msg.ranges)]
        Opposite = msg.ranges[Gamma]

        #Initalize vehicle coordinate system orgin in world coordinate system accordding to palnned drawings
        y_vehicle = (Least + Opposite)/2 - Least
        x_vehicle = 0 # The world coordinate system is at this place

        #Lookahead point in world coordinate system
        x_g=0.5
        y_g=0

        #coordinate transformation
        x_gv = (x_g - x_vehicle)*np.cos(np.deg2rad(theta)) + (y_g - y_vehicle)*np.sin(np.deg2rad(theta))
        y_gv = (x_g - x_vehicle)*np.sin(np.deg2rad(theta)) + (y_g - y_vehicle)*np.cos(np.deg2rad(theta))


        if msg.ranges[90] == float("inf") and msg.ranges[270] == float("inf"):
            trackdrive.linear.x = 0 # End of track
            trackdrive.angular.z = 0 # End of track

        self.pub.publish(trackdrive) # Publish command for forward movement


if __name__ == '__main__':
    rospy.init_node('trackdriving_node') #initialise
    Driving()
    rospy.spin()
