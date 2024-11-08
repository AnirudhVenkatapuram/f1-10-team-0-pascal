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
car_width = 0.25
tolerance = 0.05

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)
pub_lid = rospy.Publisher('angle_dist', AngleDist, queue_size=10)
pub_arrow = rospy.Publisher('arrow', AngleDist, queue_size=10)

prev_distance = 0

def find_adj_pairs(angle_dists, thresh):
	res = []
	for i in range(len(angle_dists) - 1):
		a1, d1, _ = angle_dists[i]
		a2, d2, _ = angle_dists[i+1]
		if abs(d1 - d2) > thresh:
			#print('in disparity finder', d1, d2) 
			res.append((i, i+1))
	return res

def idx_to_angle(angle, data):
    return int((angle - data.angle_min) / data.angle_increment)

def getRange(data):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    ## Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	global prev_distance
	thresh = 0.5
	nan_dist = 5 # 0.1

	angle_min_rad = -math.pi / 2
	angle_max_rad = math.pi / 2
	kmin = int((angle_min_rad - data.angle_min) / data.angle_increment)
	kmax = int((angle_max_rad - data.angle_min) / data.angle_increment)
	angler_dists = []
	overall_dists = []
        to_skip = 0
	for i in range(kmin + to_skip, kmax - to_skip):
		angle = i * data.angle_increment + data.angle_min
		dist = data.ranges[i]
		if dist < 0.01:
			print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa')
		dist = nan_dist if math.isnan(dist) else dist
		angler_dists.append((angle, dist, i))
		overall_dists.append(dist)

                msg = AngleDist(angle, dist, i, "black")
                pub_lid.publish(msg)


        pairs_list = find_adj_pairs(angler_dists, thresh)
	
        for pair_idx in pairs_list:
                pair = [angler_dists[pair_idx[0]], angler_dists[pair_idx[1]]]
                (angle1, dist1, i1), (angle2, dist2, i2) = pair
		if dist1 < 0.01 or dist2 < 0.01:
			continue
                #print(pair)
		alpha, beta = get_alpha_beta(0.2, dist1, dist2)
		#print(alpha, beta)
		if dist1 >= dist2:
			left = idx_to_angle(angle2 - beta, data) - kmin
			left = max(left, 0)
			right = i1 - kmin + 1
			print('if', dist1, dist2, angle1, angle2, alpha, beta, left, right, len(overall_dists))
		else:
			left = i2 - kmin
			right = idx_to_angle(angle1 + alpha, data) - kmin + 1
			right = min(right, len(overall_dists))
			print('else', dist1, dist2, angle1, angle2, alpha, beta, left, right, len(overall_dists))
		for i in range(left, right):
			overall_dists[i] = min(overall_dists[i], min(dist1, dist2))
			#angler_dists[i] = (angler_dists[i][0], overall_dists[i], angler_dists[i][2])
	                
                        msg = AngleDist(angler_dists[i][0], overall_dists[i], len(overall_dists)+i, "blue")
                        #pub_lid.publish(msg)


#	print('kmax' + str(data.angle_min + data.angle_increment * kmax))

# navigating to the center of the widest gap
	gap_thresh = 1

	greater_than_thresh = [kmin - 1]
	for i in range(kmin, kmax):
		angle = data.angle_min + data.angle_increment * i
		dist = overall_dists[i - kmin]

		if dist < gap_thresh:
			greater_than_thresh.append(i)
	greater_than_thresh.append(kmax)
	largest_len = 0
	largest_len_center = 0
	prev = greater_than_thresh[0]	
	for i in range(1, len(greater_than_thresh)):
		len_gap = greater_than_thresh[i] - greater_than_thresh[i - 1]
		if len_gap > largest_len:
			largest_len = len_gap
			largest_len_center = (greater_than_thresh[i] + greater_than_thresh[i - 1]) // 2
	mode = "gap"
        if mode == "gap":
            angle = data.angle_min + data.angle_increment * largest_len_center
            msg = AngleDist(angle, 2, 0, "red")
            pub_arrow.publish(msg)
#	    if angle < 0:
#	    	print(angle)
#		print(overall_dists.index(max(overall_dists)) * data.angle_increment + kmin)
#		print(kmin, largest_len_center, largest_len, kmax)
#		print(data.ranges[largest_len_center])
	    print(largest_len)
	    return data.ranges[largest_len_center], angle
        else:
            # navigating to the deepest point available
            kmin_angle = data.angle_min + data.angle_increment * kmin
    	    angle = np.argmax(overall_dists) * data.angle_increment + kmin_angle
            print(angle, np.max(overall_dists))
            print(nan_dist in overall_dists)

            msg = AngleDist(angle, 2, 0, "red")
            pub_arrow.publish(msg)

            return np.max(overall_dists), angle
	
	

def get_alpha_beta(car_width, dist_1, dist_2):
#    x1= dist_1 * np.cos(angle_1)
    #x2 = dist_2 * np.cos(angle_2)
    #y1 = dist_1 * np.sin(angle_1)
    #y2 = dist_2 * np.sin(angle_2)
    half_width = car_width/2
    #print('distances', dist_1, dist_2)
    acos_inp1 = ((2 * dist_1**2) - half_width**2) / (2*dist_1)
    acos_inp2 = ((2 * dist_2**2) - half_width**2) / (2*dist_2)
    print(acos_inp2)
    acos_inp1 = max(-1, min(1, acos_inp1))
    acos_inp2 = max(-1, min(1, acos_inp2))
    #print(acos_inp1, acos_inp2, "\n")
    final_angle_1 = math.acos( acos_inp1 )
    final_angle_2 = math.acos( acos_inp2 )
    print(final_angle_1, final_angle_2)
    if final_angle_1 < 0 or final_angle_2 < 0:
	print('error')
    return abs(final_angle_1), abs(final_angle_2)

    #gap_dist = np.sqrt((x2 - x1)**2 + (y2-y1)**2)
    #gap_angle = np.arctan2(y2 - y1, x2-x1)

    #proj_gap = gap_dist * np.sin(gap_angle)
    #if proj_gap >= car_width:
    #    return True
    #else:
    #    return False

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
