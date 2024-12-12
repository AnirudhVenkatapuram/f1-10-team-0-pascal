#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from race.msg import pid_input, AngleDistList, AngleDist

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 3  # d`istance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.96 # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
car_length = 0.5 #0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.2
tolerance = 0.45
# deep: tolerance = 0.5
#tolerance = 0.05

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data):
	nan_dist = 0.01
	angle_min_rad = -math.pi / 2
	angle_max_rad = math.pi / 2
	kmin = int((angle_min_rad - data.angle_min) / data.angle_increment)
	kmax = int((angle_max_rad - data.angle_min) / data.angle_increment)
	angler_dists = []
		
	for i in range(kmin , kmax): # iterating -90 deg to 90 deg
		angler = i * data.angle_increment + data.angle_min
		dist = data.ranges[i]
		dist = nan_dist if math.isnan(dist) else dist
		angler_dists.append((angler, dist, i))
	
	print(angler_dists)
	
def callback(data):
	global forward_projection
	getRange(data) # obtain the ray distance for theta
	# msg = pid_input()

	# # this is the error that you want to send to the PID for steering correction.
	# msg.pid_error = theta_rad
	# msg.pid_vel = 0              # velocity error can also be sent.
	# pub.publish(msg)

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan", LaserScan, callback)
	rospy.spin()