#!/usr/bin/env python
# Ref 1: http://wiki.ros.org/rviz/DisplayTypes/Marker
# Ref 2: https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.loginfo('Rviz example')

line_points = []
cube_points = []

pub_line_list = None
marker = Marker()
marker.header.frame_id = "/map"

# Mark type (http://wiki.ros.org/rviz/DisplayTypes/Marker)
# LINE_LIST: It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
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




def display_line_list(points, publisher):
    global marker
    """ 
    A function that publishes a set of points as marker line list to Rviz.
    It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...    
  
    Parameters: 
    points (list): Each item in the list is a tuple (x, y) representing a point in xy space.
    publisher (rospy.Publisher): A publisher object used to pubish the marker
  
    Returns: 
    None
  
    """


    # The coordinate frame in which the marker is publsihed.
    # Make sure "Fixed Frame" under "Global Options" in the Display panel
    # in rviz is "/map"
    marker.header.frame_id = "/map"

    # Mark type (http://wiki.ros.org/rviz/DisplayTypes/Marker)
    # LINE_LIST: It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
    marker.type = marker.LINE_L

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

    # Marker line points
    # marker.points = []
    # marker.points.append(points[0], points[1])

    # for point in points:
    marker_point = Point()  # Create a new Point()
    marker_point.x = points[0]
    marker_point.y = points[1]
    marker_point.z = 0.0
    marker.points.append(marker_point)  # Append the marker_point to the marker.points list

    # Publish the Marker using the appropriate publisher
    publisher.publish(marker)


def display_cube_list(points, publisher):
    """ 
    A function that publishes a set of points as marker cubes in Rviz.
    Each point represents the COM of the cube to be displayed.
  
    Parameters: 
    points (list): Each item in the list is a tuple (x, y) representing a point in xy space
                   for the COM of the cube.
    publisher (rospy.Publisher): A publisher object used to pubish the marker
  
    Returns: 
    None
  
    """

    marker = Marker()
    # The coordinate frame in which the marker is published.
    # Make sure "Fixed Frame" under "Global Options" in the Display panel
    # in rviz is "/map"
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()

    # Mark type (http://wiki.ros.org/rviz/DisplayTypes/Marker)
    # CUBE_LIST
    marker.type = marker.CUBE_LIST

    # Marker action (Set this as ADD)
    marker.action = marker.ADD

    # Marker scale (Size of the cube)
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # Marker color (Make sure a=1.0 which sets the opacity)
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Marker orientation (Set it as default orientation in quaternion)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Marker position 
    # The position of the marker. In this case it the COM of all the cubes
    # Set this as 0,0,0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # Marker line points
    marker.points = []

    for point in points:
        marker_point = Point()  # Create a new Point()
        marker_point.x = point[0]
        marker_point.y = point[1]
        marker_point.z = 0.0
        marker.points.append(marker_point)  # Append the marker_point to the marker.points list

    # Publish the Marker using the apporopriate publisher
    publisher.publish(marker)


# if __name__ == "__main__":
#     rospy.init_node('rviz_pub_example')
#
#     # Rviz is a node that can subscribe to topics of certain message types.
#     # Here we use the Marker message type and create two publishers
#     # to display the robot trejectory as a list of lines(MARKER.LINE_LIST)
#     # and display the landmarks as a list of cubes (MARKER.CUBE_LIST)
#     # Make sure you in rviz you subsribe to these topics in rviz by clicking
#     # on Add in the display panel, selecting by topic and choosing the
#     # appropraite topic.
#
#     # Initilize a publisher for LINE_LIST makers
#     pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
#
#     # Initilize a publisher for CUBE_LIST makers
#     pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=1)
#
#     # Example Input: Set of points to draw a square
#     line_points = [(0, 0), (0, 0.5), (0, 0.5), (0.5, 0.5), (0.5, 0.5), (0.5, 0), (0.5, 0), (0, 0)]
#
#     # Example Input: Set of cubes at four positions
#     cube_points = [(0, 0), (0, 1), (1, 0), (1, 1)]
#
#     # Call the display functions in a loop to see the markers on rviz over time
#     while not rospy.is_shutdown():
#         display_line_list(line_points, pub_line_list)
#         display_cube_list(cube_points, pub_cube_list)

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
    global line_points, pub_line_list
    line_points = points
    if len(line_points) > 0:
        display_line_list(line_points, pub_line_list)



def test():
    global line_points, cube_points
    print("Function call test")
    print(cube_points)
    print(line_points)
    # Initialize a publisher for LINE_LIST makers


    # Initialize a publisher for CUBE_LIST makers
    # pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=1)

    # Example Input: Set of points to draw a square
    # line_points = [(0, 0), (0, 0.5), (0, 0.5), (0.5, 0.5), (0.5, 0.5), (0.5, 0), (0.5, 0), (0, 0)]

    # Example Input: Set of cubes at four positions
    # cube_points = [(0, 0), (0, 1), (1, 0), (1, 1)]

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # display_cube_list(cube_points, pub_cube_list)
        if len(line_points) > 0:
            display_line_list(line_points, pub_line_list)
        rate.sleep()
