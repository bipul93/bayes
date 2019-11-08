#!/usr/bin/env python
# Ref 1: http://wiki.ros.org/rviz/DisplayTypes/Marker
# Ref 2: https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

pub_line_list = None
marker = Marker()


def init_line_marker():
    global marker
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    # Marker action (Set this as ADD)
    marker.action = marker.ADD

    # Marker scale
    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025

    # Marker color (Make sure a=1.0 which sets the opacity)
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Marker orientaiton (Set it as default orientation in quaternion)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Marker position
    # The position of the marker. In this case it the COM of all the points
    # Set this as 0,0,0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0


init_line_marker()


def display_line_list(points, publisher):
    global marker

    marker_point = Point()  # Create a new Point()
    marker_point.x = points[0]
    marker_point.y = points[1]
    marker_point.z = 0.0
    marker.points.append(marker_point)  # Append the marker_point to the marker.points list

    # Publish the Marker using the appropriate publisher
    publisher.publish(marker)


def display_cube_list(points, publisher):
    cube_marker = Marker()
    # The coordinate frame in which the marker is published.
    # Make sure "Fixed Frame" under "Global Options" in the Display panel
    # in rviz is "/map"
    cube_marker.header.frame_id = "/map"
    cube_marker.header.stamp = rospy.Time.now()

    # Mark type (http://wiki.ros.org/rviz/DisplayTypes/Marker)
    # CUBE_LIST
    cube_marker.type = cube_marker.CUBE_LIST

    # Marker action (Set this as ADD)
    cube_marker.action = cube_marker.ADD

    # Marker scale (Size of the cube)
    cube_marker.scale.x = 0.1
    cube_marker.scale.y = 0.1
    cube_marker.scale.z = 0.1

    # Marker color (Make sure a=1.0 which sets the opacity)
    cube_marker.color.a = 1.0
    cube_marker.color.r = 0.0
    cube_marker.color.g = 1.0
    cube_marker.color.b = 0.0

    # Marker orientation (Set it as default orientation in quaternion)
    cube_marker.pose.orientation.x = 0.0
    cube_marker.pose.orientation.y = 0.0
    cube_marker.pose.orientation.z = 0.0
    cube_marker.pose.orientation.w = 1.0

    # Marker position 
    # The position of the marker. In this case it the COM of all the cubes
    # Set this as 0,0,0
    cube_marker.pose.position.x = 0.0
    cube_marker.pose.position.y = 0.0
    cube_marker.pose.position.z = 0.0

    # Marker line points
    cube_marker.points = []

    for point in points:
        marker_point = Point()  # Create a new Point()
        marker_point.x = point[0]
        marker_point.y = point[1]
        marker_point.z = 0.0
        cube_marker.points.append(marker_point)  # Append the marker_point to the marker.points list

    # Publish the Marker using the apporopriate publisher
    publisher.publish(cube_marker)


def set_landmarks(points):
    global pub_line_list
    pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=10)
    pub_line_list = rospy.Publisher('line_list', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    count = 0
    while pub_cube_list.get_num_connections() < 1 or count < 5:
        display_cube_list(points, pub_cube_list)
        count += 1
        rate.sleep()


def set_line_points(points):
    global pub_line_list
    display_line_list(points, pub_line_list)

