#!/usr/bin/env python

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
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from utils import *

# Global variables for storing the path, path resolution, frame ID, and car details
plan = []
path_resolution = []
frame_id = 'map'
car_name = str(sys.argv[1])
trajectory_name = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
marker_pub = rospy.Publisher('steering_arrow', Marker, queue_size=10)
line_pub = rospy.Publisher('race_line', Marker, queue_size=10)

# Global variables for waypoint sequence and current polygon
wp_seq = 0
control_polygon = PolygonStamped()
speed = 35  # Initial speed
published = False

# Speed and lookahead parameters
min_speed = 20
max_speed = 73
min_lookahead = 1.7
max_lookahead = 2.7

# Threshold for obstacle detection
obstacle_distance_threshold = 1.5  # Meters

def construct_path():
    file_path = os.path.expanduser('/home/nvidia/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(len(plan)):
        for point in range(len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
        dx = plan[index][0] - plan[index-1][0]
        dy = plan[index][1] - plan[index-1][1]
        path_resolution.append(math.sqrt(dx*dx + dy*dy))

def get_lookahead_value(speed, min_speed, max_speed, min_lookahead, max_lookahead):
    prop = (speed - min_speed) / (max_speed - min_speed)
    return (max_lookahead - min_lookahead) * prop + min_lookahead

def lidar_callback(data):
    global speed

    # Check for obstacles within the threshold distance in front of the car
    angle_min = -15  # Degrees
    angle_max = 15   # Degrees
    front_ranges = []

    for i in range(len(data.ranges)):
        angle = math.degrees(data.angle_min + i * data.angle_increment)
        if angle_min <= angle <= angle_max:
            front_ranges.append(data.ranges[i])

    min_distance = min(front_ranges) if front_ranges else float('inf')
    print("min distance: ", min_distance)
    if min_distance < obstacle_distance_threshold:
        speed = max(min_speed, speed - 5)  # Slow down
        print("SLOWING DOWN AT SPEED: ", speed)
        
    else:
        speed = min(max_speed, speed + 1)  # Gradually increase speed

def purepursuit_control_node(data):
    command = AckermannDrive()
    global wp_seq, control_polygon, speed, published

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    min_dist_i = -1
    min_dist = float('inf')

    for i, pos in enumerate(plan):
        d = math.sqrt((pos[0] - odom_x) ** 2 + (pos[1] - odom_y) ** 2)
        if d < min_dist:
            min_dist = d
            min_dist_i = i

    heading = tf.transformations.euler_from_quaternion((
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w))[2]

    lookahead_distance = get_lookahead_value(speed, min_speed, max_speed, min_lookahead, max_lookahead)

    i = min_dist_i
    running_dist = 0
    while True:
        cur_pos = plan[i]
        next_pos = plan[(i + 1) % len(plan)]
        running_dist += math.sqrt((next_pos[0] - cur_pos[0]) ** 2 + (next_pos[1] - cur_pos[1]) ** 2)
        i = (i + 1) % len(plan)
        if running_dist >= lookahead_distance:
            break

    heading_vector = np.array([np.cos(heading), np.sin(heading)])
    target_vector = np.array([plan[i][0] - odom_x, plan[i][1] - odom_y])
    y_t = np.cross(target_vector, heading_vector)

    delta = -math.atan2(2 * 0.325 * y_t, lookahead_distance ** 2)

    command.steering_angle = max(min(delta / (math.pi / 6) * 100, 100.0), -100.0)
    command.speed = speed

    command_pub.publish(command)

    pose_x = plan[min_dist_i][0]
    pose_y = plan[min_dist_i][1]
    target_x = plan[i][0]
    target_y = plan[i][1]

    marker_pub.publish(create_steering_arrow(0, 0, 0, delta))
    if not published:
        line_pub.publish(create_race_line(plan))
        published = True

    base_link = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x = odom_x
    base_link.y = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    wp_seq += 1
    polygon_pub.publish(control_polygon)

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        if not plan:
            rospy.loginfo('Obtaining trajectory')
            construct_path()

        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.Subscriber('/{}/scan'.format(car_name), LaserScan, lidar_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
