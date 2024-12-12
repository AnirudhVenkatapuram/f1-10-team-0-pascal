#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
import tf
import numpy as np

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from utils import *
from sensor_msgs.msg import LaserScan
import numpy as np

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)
marker_pub = rospy.Publisher('steering_arrow', Marker, queue_size=10)
line_pub = rospy.Publisher('race_line', Marker, queue_size=10)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon
global speed

min_speed = 35
max_speed = 73

min_lookahead = 1.7
max_lookahead = 2.7

wp_seq          = 0
control_polygon = PolygonStamped()
speed = min_speed

published = False

SAFETY_DISTANCE = 0.5 # meters
FRONT_ANGLE_RANGE = 20  # degrees to check in front of the car
previous_speed = 0

def get_forward_distance(scan_data):
    # Convert front angle range to array indices
    angles = np.arange(len(scan_data.ranges)) * scan_data.angle_increment + scan_data.angle_min
    front_indices = np.where(np.abs(angles) <= np.radians(FRONT_ANGLE_RANGE))[0]
    
    # Get the minimum distance in the front cone
    front_distances = np.array(scan_data.ranges)[front_indices]
    # Filter out invalid readings
    valid_distances = front_distances[np.isfinite(front_distances)]
    print(valid_distances)
    
    if len(valid_distances) > 0:
        return np.min(valid_distances)
    return float('inf')

latest_forward_distance = float('inf')
def scan_callback(scan_data):
    global latest_forward_distance
    latest_forward_distance = get_forward_distance(scan_data)

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/nvidia/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))


# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN = 0.325

def get_lookahead_value(speed, min_speed, max_speed, min_lookahead, max_lookahead):
    prop = (speed - min_speed) / (max_speed - min_speed)
    return (max_lookahead - min_lookahead) * prop + min_lookahead

def get_speed(steering_angle):
    s = max_speed
    steering_angle = abs(steering_angle)

    mode = ''

    if mode == 'prabhath':
        if steering_angle < 10:
            s = max_speed
        else:
            s = (max_speed)/(1+0.02778*(steering_angle-5))
    else:
        if steering_angle < 5:
            s = max_speed
        elif steering_angle < 40:
            s = (max_speed + min_speed) / 2
        else:
            s = min_speed


    #s = max_speed*np.exp(-0.1*steering_angle)
    #if abs(steering_angle) > 10:
        #s = min_speed
    return s

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon
    global speed
    global published

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    min_dist_i = -1
    min_dist = float('inf')

    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Your code here
    for i, pos in enumerate(plan):
        d = dist(pos, (odom_x, odom_y))
        if d < min_dist:
            min_dist = d
            min_dist_i = i
    
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = get_lookahead_value(speed, min_speed, max_speed, min_lookahead, max_lookahead)

    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    # Your code here
    i = min_dist_i
    running_dist = 0
    while True:
        cur_pos = plan[i]
        next_pos = plan[(i + 1) % len(plan)]
        running_dist += dist(cur_pos, next_pos)
        i = (i + 1) % len(plan)
        if running_dist >= lookahead_distance:
            break

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here
    heading_vector = np.array([np.cos(heading), np.sin(heading)])
    target_vector = np.array([plan[i][0] - odom_x, plan[i][1] - odom_y])
    y_t = np.cross(target_vector, heading_vector)
    print('y_t', y_t)

    # alpha = math.asin(y_t / lookahead_distance)
    delta = -math.atan2(2 * WHEELBASE_LEN * y_t, lookahead_distance ** 2)
    print(delta)
    print(lookahead_distance)

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # scale radians to degrees
    command.steering_angle = delta / (math.pi / 6) * 100
    command.steering_angle = max(command.steering_angle, -STEERING_RANGE)
    command.steering_angle = min(command.steering_angle, STEERING_RANGE)
    # if delta < STEERING_RANGE and delta > -1*STEERING_RANGE:
    #     command.steering_angle = delta

    print("Steering Angle:\t\t" + str(math.degrees(delta)))
    print("True Steering Angle:" + str(command.steering_angle))

    vel_lookahead = 0.8

    i = min_dist_i
    running_dist = 0
    while True:
        cur_pos = plan[i]
        next_pos = plan[(i + 1) % len(plan)]
        running_dist += dist(cur_pos, next_pos)
        i = (i + 1) % len(plan)
        if running_dist >= vel_lookahead:
            break

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    command.speed = plan[i][4] * (max_speed - min_speed) + min_speed # get_speed(command.steering_angle)

    speed = command.speed

    global latest_forward_distance, previous_speed
    
    # Calculate desired speed based on steering angle
    desired_speed = get_speed(command.steering_angle)
    
    # Adjust speed based on forward distance
    if latest_forward_distance < SAFETY_DISTANCE:
        # Stop the car if it's too close to an obstacle
        print('Emergency STOP: TOO CLOSE TO OBSTACLE')
        command.speed = 0
    else:
        # Gradually reduce speed as we approach obstacles
        distance_factor = min(latest_forward_distance / SAFETY_DISTANCE, 1.0)  # Cap at 1.0
        adjusted_speed = desired_speed * distance_factor
        command.speed = adjusted_speed
    
    # Smooth acceleration (prevent sudden speed changes)
    acceleration_limit = 2.0  # maximum speed change per update
    speed_difference = command.speed - previous_speed
    if abs(speed_difference) > acceleration_limit:
        command.speed = previous_speed + acceleration_limit * np.sign(speed_difference)

    previous_speed = command.speed
    speed = command.speed
    command_pub.publish(command)    

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    # These are set to zero only so that the template code builds. 
    pose_x=plan[min_dist_i][0]    
    pose_y=plan[min_dist_i][1] 
    target_x=plan[i][0]
    target_y=plan[i][1]

    marker_pub.publish(create_steering_arrow(0, 0, 0, delta))
    line_pub.publish(create_race_line(plan))
    # if not published:
    #     line_pub.publish(create_race_line(plan))
    #     published = True

    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        
        # Add the LiDAR subscriber
        rospy.Subscriber('/{}/scan'.format(car_name), LaserScan, scan_callback)
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), 
                        PoseStamped, purepursuit_control_node)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass