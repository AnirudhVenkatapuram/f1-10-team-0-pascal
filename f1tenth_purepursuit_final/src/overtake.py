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
from geometry_msgs.msg import Point
import numpy as np

# Add to global variables
is_car_ahead = False
safe_distance = 2.0  # meters
overtaking_state = "following"  # States: "following", "preparing_overtake", "overtaking", "returning"
original_path = []
overtake_path = []


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

def laser_callback(scan_msg):
    global is_car_ahead
    
    # Convert laser scan to cartesian coordinates
    # Add a small epsilon to ensure we include the last angle
    epsilon = scan_msg.angle_increment * 0.1
    angles = np.arange(scan_msg.angle_min, 
                      scan_msg.angle_max + epsilon, 
                      scan_msg.angle_increment)
    
    # Ensure angles array matches the length of ranges
    angles = angles[:len(scan_msg.ranges)]
    points = []
    
    for i, distance in enumerate(scan_msg.ranges):
        # Skip invalid measurements
        if not np.isfinite(distance) or distance < scan_msg.range_min or distance > scan_msg.range_max:
            continue
            
        angle = angles[i]
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        points.append((x, y))
    
    # Check for car ahead within a narrow forward-facing cone
    forward_points = [p for p in points if abs(np.arctan2(p[1], p[0])) < np.pi/6]  # 30-degree cone
    if forward_points:
        min_distance = min([np.sqrt(p[0]**2 + p[1]**2) for p in forward_points])
        is_car_ahead = min_distance < safe_distance


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
    global overtaking_state, plan, original_path
    
    if not original_path:
        original_path = plan[:]

    
    # State machine for overtaking
    if overtaking_state == "following" and is_car_ahead:
        overtaking_state = "preparing_overtake"
        # Generate overtake path
        plan = generate_overtake_path(data.pose.position, original_path)
    
    elif overtaking_state == "preparing_overtake":
        if not is_car_ahead:
            overtaking_state = "overtaking"
    
    elif overtaking_state == "overtaking":
        if not is_car_ahead:
            # Check if we're past the overtaken vehicle
            overtaking_state = "returning"
            # Generate return path to original racing line
            plan = generate_return_path(data.pose.position, original_path)
    
    elif overtaking_state == "returning":
        # Check if we're back on the racing line
        if distance_to_racing_line(data.pose.position, original_path) < 0.3:
            overtaking_state = "following"
            plan = original_path.copy()
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
def generate_overtake_path(current_pose, original_path):
    """Generate a path for overtaking"""
    overtake_path = []
    lateral_offset = 1.5  # meters to the side for overtaking
    
    # Find closest point on original path
    closest_idx = distance_to_racing_line(current_pose, original_path)
    
    # Generate parallel path for overtaking
    for i in range(int(closest_idx), int(min(closest_idx + 50, len(original_path)))):
        point = original_path[i]
        # Calculate perpendicular offset
        heading = math.atan2(original_path[i][1] - original_path[i-1][1],
                           original_path[i][0] - original_path[i-1][0])
        perpendicular = heading + math.pi/2
        
        new_x = point[0] + lateral_offset * math.cos(perpendicular)
        new_y = point[1] + lateral_offset * math.sin(perpendicular)
        overtake_path.append([new_x, new_y, point[2], point[3], point[4]])
    
    return overtake_path

def generate_return_path(current_pose, original_path):
    """Generate a path to return to the racing line"""
    return_path = []
    closest_idx = distance_to_racing_line(current_pose, original_path)
    
    # Create a smooth curve back to the racing line
    current_pos = [current_pose.x, current_pose.y]
    target_idx = closest_idx + 20  # Look ahead on racing line
    
    # Generate Bezier curve points
    control_points = [
        current_pos,
        [current_pos[0] + 1.0, current_pos[1]],  # Control point 1
        [original_path[target_idx][0], original_path[target_idx][1]]  # Target point
    ]
    
    # Generate path points using Bezier curve
    for t in np.linspace(0, 1, 20):
        point = bezier_point(control_points, t)
        return_path.append([point[0], point[1], 0, 0, 0.8])  # Add appropriate velocity
    
    return return_path

def distance_to_racing_line(pose, racing_line):
    """Calculate minimum distance to racing line"""
    distances = [math.sqrt((pose.x - p[0])**2 + (pose.y - p[1])**2) for p in racing_line]
    return min(distances)

def bezier_point(control_points, t):
    """Calculate point on Bezier curve"""
    n = len(control_points) - 1
    point = [0, 0]
    for i in range(n + 1):
        coef = math.comb(n, i) * (1 - t)**(n - i) * t**i
        point[0] += coef * control_points[i][0]
        point[1] += coef * control_points[i][1]
    return point

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        
        # Add laser scan subscriber
        rospy.Subscriber('/{}/scan'.format(car_name), LaserScan, laser_callback)
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass