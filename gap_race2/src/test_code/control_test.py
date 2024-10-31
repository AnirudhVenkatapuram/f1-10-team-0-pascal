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
min_speed = 1

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_0/offboard/command', AckermannDrive, queue_size = 1)
marker_pub = rospy.Publisher('steering_arrow', Marker, queue_size=10)

def create_steering_arrow(x, y, z, yaw):
	marker = Marker()
	marker.header.frame_id = "car_0_laser"
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

	print("PID Control Node is Listening to error")
	
	angle_rad = data.pid_error
	print(angle_rad * 180 / math.pi)
	arrow_marker = create_steering_arrow(0, 0, 0, angle_rad)
	marker_pub.publish(arrow_marker)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
        kp = -50
        kd = -200
        vel_input = 35
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#ki = input("Enter Ki Value: ")
	#vel_input = input("Enter velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
