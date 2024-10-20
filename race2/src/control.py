#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker

# PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 0.0	#TODO
min_speed = 5

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_0/offboard/command', AckermannDrive, queue_size = 1)
marker_pub = rospy.Publisher('steering_arrow', Marker, queue_size=10)

def create_steering_arrow(x, y, z, yaw):
	marker = Marker()
	marker.header.frame_id = 'car_0_laser'
	marker.header.stamp = rospy.Time.now()

	marker.ns = 'steering_arrow'
	marker.id = 0
	marker.type = Marker.ARROW
	marker.action = Marker.ADD

	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = z

	marker.pose.orientation.x = 0
	marker.pose.orientation.y = 0
	marker.pose.orientation.z = yaw
	marker.pose.orientation.w = 1

	marker.scale.x = 1
	marker.scale.y = 0.1
	marker.scale.z = 0.1

	marker.color.r = 1.0
	marker.color.g = 0
	marker.color.b = 0
	marker.color.a = 1

        return marker

def sigmoid(x):
        return 1 / (1 + math.exp(-x))

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	angle = 0.0

	print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	print("Kd's multiplier", str(prev_error - data.pid_error))
	# 1. Scale the error
	v_theta = kp * data.pid_error + kd * (prev_error - data.pid_error)
	# 2. Apply the PID equation on error to compute steering
	angle -= v_theta
	if angle > 100:
		angle = 100
	elif angle < -100:
		angle = -100

	print(angle)
	prev_error = data.pid_error

	#if abs(angle) <= 10:
	#	vel = vel_input
	#elif abs(angle) <= 20:
	#	vel = 10
	#else:
	#	vel = 1
	vel = math.tanh(1 - (abs(angle) / 100)) * (vel_input - min_speed) + min_speed

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	command.steering_angle = angle

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = vel

	# Move the car autonomously
	command_pub.publish(command)

	arrow_marker = create_steering_arrow(0, 0, 0, angle * math.pi / 180)
	marker_pub.publish(arrow_marker)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
