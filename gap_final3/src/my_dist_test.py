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
# deep: tolerance = 0.5
#tolerance = 0.05
ignore = 0.05

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)
pub_lid = rospy.Publisher('angle_dist', AngleDist, queue_size=10)
pub_arrow = rospy.Publisher('arrow', AngleDist, queue_size=10)

prev_distance = 0
prev_angle = 0

def find_adj_pairs(angle_dists, thresh):
	res = []
	for i in range(len(angle_dists) - 1):
		a1, d1, i1 = angle_dists[i]
		a2, d2, i2 = angle_dists[i+1]
		if abs(d1 - d2) > thresh:
			#print('in disparity finder', d1, d2) 
			res.append((i1, i2))
	return res

def angle_to_idx(angle, data):
    return int((angle - data.angle_min) / data.angle_increment)

def idx_to_angle(idx, data, kmin=0):
    return int(data.angle_increment * idx + data.angle_min) - kmin

def getRange(data):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    ## Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
        global prev_angle
        
        thresh = 0.1# if two distances are separated by this gap or more, it's a disparity
	mode = "deep"
    	nan_dist = 4.21000019

	angle_min_rad = -math.pi / 2
	angle_max_rad = math.pi / 2
	kmin = int((angle_min_rad - data.angle_min) / data.angle_increment)
	kmax = int((angle_max_rad - data.angle_min) / data.angle_increment)
	angler_dists = []
	overall_dists = []
        to_skip = 0
	for ik in range(kmin + to_skip, kmax - to_skip): # iterating -90 deg to 90 deg
		angle = ik * data.angle_increment + data.angle_min
		dist = data.ranges[ik]
		dist = nan_dist if math.isnan(dist) else dist
		angler_dists.append((angle, dist, ik))
		overall_dists.append(dist)

                msg = AngleDist(angle, dist, ik, "black")
                pub_lid.publish(msg)


        pairs_list = find_adj_pairs(angler_dists, thresh)
        min_dist = 0.75	
        for ik in [item for pairs in pairs_list for item in pairs]: # iterating over list of disparities
		angle, dist, ik = angler_dists[ik - kmin]
                if dist < ignore or abs(dist - nan_dist) <= 0.001: # ignoring disparities where one distance is too small
                    #if abs(dist - nan_dist) <= 0.001: print("skip!" +  str(ik))
                    continue
		
                alpha = get_alpha(car_width / 2 + tolerance, dist)
                
                modew = "both"
                if modew == "both":
                    left = min(
                            max(0, angle_to_idx(angle - alpha, data) - kmin),
                            ik - kmin
                            )

                    right = max(
                            ik - kmin + 1,
                            min(len(overall_dists), angle_to_idx(angle + alpha, data) - kmin + 1)
                            )
                else:
    		    if dist1 >= dist2:
    			    left = idx_t
                            _angle(angle2 - beta, data) - kmin
    			    left = max(left, 0)
		            right = i1 - kmin + 1
		            print('if', dist1, dist2, angle1, angle2, alpha, beta, left, right, len(overall_dists))
		    else:
		    	    left = i2 - kmin
		    	    right = angle_to_idx(angle1 + alpha, data) - kmin + 1
		    	    right = min(right, len(overall_dists))
		    	    print('else', dist1, dist2, angle1, angle2, alpha, beta, left, right, len(overall_dists))
                for i in range(left, right):
			ik = i + kmin
                        if dist < min_dist:
                            continue
                        overall_dists[i] = min(overall_dists[i], dist)
                        angler_dists[i] = (angler_dists[i][0], overall_dists[i], ik)
                        #print("NEW DIST:" + str(overall_dists[i]))
                        msg = AngleDist(angler_dists[i][0], overall_dists[i], 2 * len(overall_dists)+i, "cyan")
                        pub_lid.publish(msg)
                        pass
        final_pairs_list = find_adj_pairs(angler_dists, 0.075)
        gaps = [] # idx1, idx2, dist

        min_width = car_width
        min_depth = .2
        max_angle_changer = math.pi / 6
        for i in range(len(final_pairs_list)-1):
            ik1 = final_pairs_list[i][1]
            ik11 = final_pairs_list[i][0]
            ik2 = final_pairs_list[i+1][0]
            ik22 = final_pairs_list[i+1][1]
            ikmid = (ik1 + ik2) // 2
            dist = angler_dists[ikmid - kmin][1]
            angle = angler_dists[ikmid - kmin][0]
            
            dist1 = overall_dists[ik1-kmin]
            dist2 = overall_dists[ik2-kmin]
            angle_diff = abs(angler_dists[ik1-kmin][0] - angler_dists[ik2-kmin][0])
            width = get_opp_side(dist1, dist2, angle_diff)
            
            if width < min_width or dist < min_depth or (overall_dists[ik1 - kmin] == nan_dist and overall_dists[ik2 - kmin] == nan_dist):
                continue

            msg = AngleDist(angler_dists[ik11 - kmin][0], overall_dists[ik11 - kmin], ik11, "magenta")
            #pub_arrow.publish(msg)
            msg = AngleDist(angler_dists[ik22 - kmin][0], overall_dists[ik22 - kmin], ik22, "magenta")
            #pub_arrow.publish(msg)


            msg = AngleDist(angler_dists[ik1 - kmin][0], overall_dists[ik1 - kmin], ik1, "magenta")
            pub_arrow.publish(msg)
            msg = AngleDist(angler_dists[ik2 - kmin][0], overall_dists[ik2 - kmin], ik2, "magenta")
            pub_arrow.publish(msg)

            
            msg = AngleDist(angle, width, ikmid, "green")
            pub_arrow.publish(msg)

            gaps.append((angle, dist, ikmid, width))
            gaps.append((angler_dists[ik1-kmin][0], angler_dists[ik1 - kmin][1], 0, 0))
            gaps.append((angler_dists[ik2-kmin][0], angler_dists[ik2 - kmin][1], 0, 0))


        print("N Gaps:" + str(len(gaps)))
        print(gaps)

        # 1 - depth, 3 - width
        key_idx = 1
        
        #gaps = []        
        if len(gaps) != 0:
            key_vals = [sublist[key_idx] for sublist in gaps]
            max_val = max(key_vals)
            max_idx = key_vals.index(max_val)

            angle, dist, _, width = gaps[max_idx]
        else:
            # max depth
            kmin_angle = data.angle_min + data.angle_increment * kmin
            dist = np.max(overall_dists)
            angle = (np.argmax(overall_dists)) * data.angle_increment + kmin_angle

        print("final angle:" + str(angle) + " best dist: " + str(dist))
        msg = AngleDist(angle, 3, 99, "red")
        pub_arrow.publish(msg)
        
        prev_anagle = angle
        return dist, angle
	
def get_opp_side(d1, d2, theta):
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))

def get_alpha(half_width, dist_1):
    acos_inp1 = ((2 * dist_1**2) - half_width**2) / (2*dist_1 ** 2)
    acos_inp1 = max(-1, min(1, acos_inp1))
    final_angle_1 = math.acos( acos_inp1 )
    if final_angle_1 < 0:
	print('error')
    return abs(final_angle_1)


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
