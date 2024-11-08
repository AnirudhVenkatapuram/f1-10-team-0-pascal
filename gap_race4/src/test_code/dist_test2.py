#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 3  # distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.96 # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)
prev_distance = 0

def find_adj_pairs(angle_dists, thresh):
	res = []
	for i in range(len(angle_dists) - 1):
		a1, d1 = angle_dists[i]
		a2, d2 = angle_dists[i+1]
		if abs (d1 - d2) > thresh: res.append((i, i+1))
	return res

def getRange(data):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	global prev_distance
	thresh = 0.5
	max_dist = 4
	thresh_dist = 2

	angle_min_rad = -math.pi / 2
	angle_max_rad = math.pi / 2
	kmin = int((angle_min_rad - data.angle_min) / data.angle_increment)
	kmax = int((angle_max_rad - data.angle_min) / data.angle_increment)
	angler_dists = []
	overall_dists = []
	for i in range(kmin, kmax):
		angle = i * data.angle_increment + data.angle_min
		dist = data.ranges[i]
		dist = 0 if math.isnan(dist) else dist
		angler_dists.append((angle, dist))
		overall_dists.append(dist)
	pairs_list = find_adj_pairs(angler_dists, thresh)
	best_angle = None
	best_dist = -1
	for p in pairs_list:
		i1, i2 = p
		if angler_dists[i1][1] >= thresh_dist:
			if best_dist < angler_dists[i1][0] or best_dist < angler_dists[i2][0]:
				best_angle = angler_dists[i1][0]
			print(str(angler_dists[i1]) + "  " + str(angler_dists[i2]))
#	print("distance: " + str(np.max(angler_dists)) + " rad " + str((np.argmax(angler_dists) + kmin) * data.angle_increment + data.angle_min))
	print('kmin' + str(data.angle_min + data.angle_increment * kmin))
	print('kmax' + str(data.angle_min + data.angle_increment * kmax))
	kmin_angle = data.angle_min + data.angle_increment * kmin
	angle = np.argmax(overall_dists) * data.angle_increment + kmin_angle
	print(angle)
	return np.max(overall_dists), angle


def callback(data):
	global forward_projection

        a, theta_rad = getRange(data) # obtain the ray distance for theta

        msg = pid_input()       # An empty msg is created of the type pid_input
        # this is the error that you want to send to the PID for steering correction.
        msg.pid_error = theta_rad
        msg.pid_vel = 0              # velocity error can also be sent.
        pub.publish(msg)

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan",LaserScan,callback)
	rospy.spin()