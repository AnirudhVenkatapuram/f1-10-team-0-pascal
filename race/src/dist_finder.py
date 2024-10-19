#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.0	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 1.3	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)
prev_distance = 0

def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	global prev_distance
	adjusted_angle = (angle - 90)*(math.pi/180)
        #print(data)
	k = (adjusted_angle - data.angle_min) / data.angle_increment
	dist = data.ranges[int(k)]
	count = 1
	if math.isnan(dist):
		print('using prev distance')
		dist=prev_distance
	print("Angle_incr:", str(data.angle_increment))
	print("Count: ", str(count))
        print("Angle:{}  Distance: {} ".format(adjusted_angle, dist))
	prev_distance = dist
	return dist



def callback(data):
	global forward_projection

	theta = 70 # you need to try different values for theta
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	swing = math.radians(theta)
	alpha = math.atan2(a*math.cos(swing) - b, a * math.sin(swing))
	print("opp", a*math.cos(swing) - b)
        print("adf", a * math.sin(swing))
        print('alpha =' + str(alpha))
	AB = b * math.cos(alpha)
	print('AB', AB)
	CD = AB + forward_projection * math.sin(alpha)

	error = desired_distance - CD
        print("Error:", str(error))
	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan",LaserScan,callback)
	rospy.spin()
