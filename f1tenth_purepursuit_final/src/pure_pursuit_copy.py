#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
import tf
import numpy as np
from copy import deepcopy

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud

from sensor_msgs.msg import LaserScan
from utils import *

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
point_pub = rospy.Publisher('lidar_pointcloud', PointCloud, queue_size=10)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon
global speed

max_speed = 65
min_speed = 35

min_lookahead = 1.7
max_lookahead = 2.4

wp_seq          = 0
control_polygon = PolygonStamped()
speed = min_speed
speed_reduction = 0
is_overtaking = False
entered_zone = False
published = False

CUR_PLAN_IDX = 0
PLAN_PATHS = [
    "smoothed_anti_line_slower",
    "combined_overtake"
]

def load_path(path_name):
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    my_plan = []
    file_path = os.path.expanduser('/home/nvidia/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(path_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            my_plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(my_plan)):
        for point in range(0, len(my_plan[index])):
            my_plan[index][point] = float(my_plan[index][point])

    return my_plan

def toggle_plan(idx=None):
    global CUR_PLAN_IDX
    global plan
    
    if idx:
        CUR_PLAN_IDX = idx
    else:
        CUR_PLAN_IDX = (CUR_PLAN_IDX + 1) % len(PLAN_PATHS)
    plan = load_path(PLAN_PATHS[CUR_PLAN_IDX])

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    global plan

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
    lookahead = (max_lookahead - min_lookahead) * prop + min_lookahead
    return max(lookahead, 1)


############ LIDAR ############ LIDAR ############ ############ LIDAR ############ LIDAR ############ 

angler2dists = {}
heading = 0

# Some useful variable declarations.

def getRange(data):
    global angler2dists

    my_angler2dists = {}
    angle_min_rad = -math.pi / 2
    angle_max_rad = math.pi / 2
    kmin = int((angle_min_rad - data.angle_min) / data.angle_increment)
    kmax = int((angle_max_rad - data.angle_min) / data.angle_increment)
        
    for i in range(kmin , kmax): # iterating -90 deg to 90 deg
        angler = i * data.angle_increment + data.angle_min
        dist = data.ranges[i]
        dist = None if math.isnan(dist) else dist
        my_angler2dists[angler] = dist
    
    angler2dists = my_angler2dists
		
def lidar_callback(data):
	global forward_projection
	getRange(data) # obtain the ray distance for theta

def get_lidar_dist(angle_rad):
    global angler2dists

    min_error = float("inf")
    angle_rad_est = None

    my_angler2dists = angler2dists
    for ar in my_angler2dists:
        error = abs(ar - angle_rad) 
        if error < min_error:
            min_error = error
            angle_rad_est = ar
    
    if not angle_rad_est:
        # print("failed to retrieving", angle_rad_est)
        return 0, 0

    # print("retrieving", angle_rad_est, ":", angler2dists[angle_rad_est])
    return angler2dists[angle_rad_est], angle_rad_est

def get_lidar_window_dist(delta, granularity=10, jump = .12, ignore=.1):
    min_dist = float('inf')
    min_delta_est = None

    increment = jump / granularity
    for i in range(granularity):
        theta = delta - jump / 2 + i * increment
        # print(delta, "increment", theta)

        obst_dist, delta_est = get_lidar_dist(theta)
        if not obst_dist or obst_dist < ignore: continue
        if obst_dist <= min_dist:
            min_dist = obst_dist
            min_delta_est = delta_est

    # print(min_dist, min_delta_est)
    return min_dist, min_delta_est

# get_lidar_window_dist(0)
inside_track = load_path("smoothed_inside_line_copy")
outside_track = load_path("smoothed_outside_line_copy")
all_points = inside_track + outside_track

def is_part_of_track(x, y, theta, error = 0.2, ret_obs = False):
    global heading

    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    d, theta_est = get_lidar_dist(theta)
    # print("inner d", d, "theta", theta)
    if not theta_est or not d:
        if ret_obs:
            return True, None
        return True

    # print("heading", heading, "theta_est", theta_est)

    theta_est += heading
    observed_point = (d * math.cos(theta_est) + x, d * math.sin(theta_est) + y)
    # print("observed_point", observed_point)
    min_dist = float('inf')
    b = False

    for point in all_points:
        # print("point", point)
        if dist(point, observed_point) <= error:
            b = True
        min_dist = min(min_dist, dist(point, observed_point))
        # print(dist(point, observed_point))

    # print(min_dist)
    if ret_obs:
        return b, observed_point
    
    return b

############ LIDAR ############ LIDAR ############ ############ LIDAR ############ LIDAR ############ 

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm
    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    
    global curr_polygon
    global speed
    global published
    global heading
    global speed_reduction
    global is_overtaking
    global entered_zone

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.
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
    reference_idx = None
    i = min_dist_i
    running_dist = 0
    while True:
        cur_pos = plan[i]
        next_pos = plan[(i + 1) % len(plan)]
        running_dist += dist(cur_pos, next_pos)
        i = (i + 1) % len(plan)
        reference_idx = i
        if running_dist >= lookahead_distance:
            break

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here
    heading_vector = np.array([np.cos(heading), np.sin(heading)])
    target_vector = np.array([plan[i][0] - odom_x, plan[i][1] - odom_y])
    y_t = np.cross(target_vector, heading_vector)
    # print('y_t', y_t)

    # alpha = math.asin(y_t / lookahead_distance)
    delta = -math.atan2(2 * WHEELBASE_LEN * y_t, lookahead_distance ** 2)
    # print(delta)
    # print(lookahead_distance)

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # scale radians to degrees
    command.steering_angle = delta / (math.pi / 6) * 100
    command.steering_angle = max(command.steering_angle, -STEERING_RANGE)
    command.steering_angle = min(command.steering_angle, STEERING_RANGE)
    # if delta < STEERING_RANGE and delta > -1*STEERING_RANGE:
    #     command.steering_angle = delta

    # print("Steering Angle:\t\t" + str(math.degrees(delta)))
    # print("True Steering Angle:" + str(command.steering_angle))

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

    curr_speed = plan[i][4] * (max_speed - min_speed) + min_speed # get_speed(command.steering_angle)

    ################################################
    delta = 0
    error = .2 + ((curr_speed) / max_speed) * .2
    following_dist = 2 + ((curr_speed) / max_speed) * 2
    emergency_stop_dist = .8
    emergency = False
    reduction_delta = 2
    is_track = True
    obst_dist, delta_est = None, None
    deltas = [-0.12, -.08, -0.05, 0, 0.08, 0.1, 0.12]
    points_to_pub = []

    print("\n")
    print("og speed", curr_speed)

    # overtake logic
    
    # ### Try multiple
    # deltas = [-0.05 + i / 10 for i in range(10)]
    # for d in deltas:
    #     obst_dist, delta_est = get_lidar_dist(d)
    #     is_track, observed_point = is_part_of_track(odom_x, odom_y, delta, error=error, ret_obs = True)
    #     if not is_track: 
    #         break

    ### Try one
    for delta in deltas:
        if not is_track: break
        obst_dist, delta_est = get_lidar_dist(delta)   
        # print(delta, obst_dist, delta_est) 
        if delta_est:
            is_track, observed_point = is_part_of_track(odom_x, odom_y, delta, error=error, ret_obs = True)
            if observed_point:
                points_to_pub.extend([
                    (observed_point[0], observed_point[1]),
                    (odom_x, odom_y)
                ])

    if obst_dist and not is_track:
        if obst_dist <= following_dist: # overtake!
            speed_reduction += (1 - obst_dist / following_dist) * 3.5
        elif obst_dist <= emergency_stop_dist: # hard stop!
            emergency = True
    if is_track:
        speed_reduction = 0

    curr_speed = max(22, curr_speed - speed_reduction)
    
    if emergency:
        curr_speed = 0

    point_pub.publish(create_point_list(
        points_to_pub
    ))

    print("heading", heading, "theta", delta)
    # print("(X, Y):", odom_x, odom_y)
    print("error:", error)
    print("dist:", obst_dist)
    print("is track:", is_track)
    print("following_dist:", following_dist)
    print("emergency", emergency)
    print("speed", curr_speed)
    print("speed red", speed_reduction)
    print("plan", CUR_PLAN_IDX)
    print("reference idx", i)
    print("\n")

    # exit()

    ################################################

    command.speed = curr_speed

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
    line_pub.publish(create_race_line([p[:5] for p in plan]))
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
            print("constructed")

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber("/car_0/scan", LaserScan, lidar_callback)
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
