import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud

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

	marker.color.r = 0
	marker.color.g = 0
	marker.color.b = 1
	marker.color.a = 1

	return marker

def create_race_line(points):
	marker = Marker()
	marker.header.frame_id = 'map'
	marker.header.stamp = rospy.Time.now()
	marker.ns = "race_line"
	marker.id = 0
	marker.type = Marker.LINE_STRIP
	marker.action = Marker.ADD

	marker.scale.x = 0.1
	marker.color.a = 1.0
	marker.color.r = 1
	marker.color.g = 1
	marker.color.b = 1

	for x, y, _, _, _ in points:
		point = Point()
		point.x = x
		point.y = y
		point.z = 0.0
		marker.points.append(point)
	return marker

def create_point_list(points_list):
	pcd_msg = PointCloud()
	pcd_msg.header.stamp = rospy.Time.now()
	pcd_msg.header.frame_id = "map"

	for x, y in points_list:
		pt_msg = Point32(x, y, 0.1)
		pcd_msg.points.append(pt_msg)

	return pcd_msg