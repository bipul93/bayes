#!/usr/bin/env python
import os
from os import sys, path

from tf.transformations import euler_from_quaternion

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rosbag
import rospy
import numpy
import math

from scripts.markers import set_landmarks, set_line_points

# Global parameters

to_radian = 0.0174533
to_degree = 57.2958

# Occupancy grid details
# Grid Boundary 7m  and cell size 20cm
grid_dimension = 700  # type: int
cell_size = 20
heading_discretization = 30  # degrees
discretization_range = 360 / heading_discretization

# shape should be (35,35,4)
grid = numpy.zeros((int(grid_dimension / cell_size), grid_dimension / cell_size, discretization_range))

# Bot's initial pos is (12,28) and 200.52 degrees. our index starts at 0.
grid[11, 27, 6] = 0.8

for i in range(int(discretization_range)):
    if i != 4:
        grid[11, 27, i] = 0.1
neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
for m, x in enumerate(neighbour_nodes):
    for i in range(int(discretization_range)):
        grid[11 + x[0], 27 + x[1], i] = 0.1

# Landmarks in meters
landmarks = [(1.25, 5.25), (1.25, 3.25), (1.25, 1.25), (4.25, 1.25), (4.25, 3.35), (4.25, 5.25)]

bag = rosbag.Bag(rospy.myargv(argv=sys.argv)[1])

estimation = []


# Utils
def translation(x, y):
    return (x + 1) * 20 - 10, (y + 1) * 20 - 10


def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def normal_pdf(z, mean, var):
    variance = var
    variance = float(variance) ** 2
    numerator = math.exp(-(float(z) - float(mean)) ** 2 / (2 * variance))
    denominator = (2 * math.pi * variance) ** .5
    return numerator / denominator


def write_to_file():
    global to_degree
    print("Thank you for being patient!")
    f = open(path.dirname(os.path.realpath(__file__)) + "/estimation.txt", "w")
    for i in range(1, len(estimation)):
        angle = estimation[i][2] * (math.pi / 6) + (math.pi / 12)
        if i % 2 == 0:
            f.write("U: (" + str(estimation[i][0]) + ", " + str(estimation[i][1]) + ". " + str(
                round(angle * to_degree, 1)) + ")\n")
        else:
            f.write("P: (" + str(estimation[i][0]) + ", " + str(estimation[i][1]) + ". " + str(
                round(angle * to_degree, 1)) + ")\n")
    f.close()
    print("Output written into file estimation.txt")


def read_bag():
    global grid
    print("Reading bag file and running bayes filter...")
    try:
        for topic, msg, t in bag.read_messages(topics=["Movements", "Observations"]):
            print("Running Tag: ", msg.timeTag)
            # if msg.timeTag > 4:
            #     break
            if topic == "Movements":
                # print("Movements", msg)
                motion_model(msg.rotation1, msg.translation, msg.rotation2)
            else:
                # print("Observations", msg)
                sensor_model(msg.tagNum, msg.range, msg.bearing)
            index = numpy.unravel_index(numpy.argmax(grid, axis=None), grid.shape)
            point = (translation(index[0], index[1]))

            set_line_points((float(point[0]) / 100, float(point[1]) / 100))
            estimation.append((float(point[0]) / 100, float(point[1]) / 100, index[2]))
    # except:
    #     print("Something went wrong while reading grid.bag")
    finally:
        bag.close()
    write_to_file()


def motion_model(rot1, trans, rot2):
    global to_degree, grid
    rot1 = euler_from_quaternion((rot1.x, rot1.y, rot1.z, rot1.w))
    rot2 = euler_from_quaternion((rot2.x, rot2.y, rot2.z, rot2.w))

    cell = (numpy.where(grid > 0.000001))

    total = 0
    for cell_index in range(cell[0].shape[0]):
        x1, y1 = translation(cell[0][cell_index], cell[1][cell_index])
        k1 = normalize(cell[2][cell_index] * (math.pi / 6) + (math.pi / 12))

        for ind, data in numpy.ndenumerate(grid):

            x2, y2 = translation(ind[0], ind[1])
            k2 = normalize(ind[2] * (math.pi / 6) + (math.pi / 12))

            trans_cap = numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            rot1_cap = normalize(math.atan2(y1 - y2, x1 - x2) - k2)
            rot2_cap = normalize(k1 - k2 - rot1_cap)

            p1 = normal_pdf(rot1_cap, rot1[2], (math.pi / 12) + 0.2)
            p2 = normal_pdf(trans_cap, trans, 25)
            p3 = normal_pdf(rot2_cap, rot2[2], (math.pi / 12) + 0.2)

            prob = grid[cell[0][cell_index], cell[1][cell_index], cell[2][cell_index]] * p1 * p2 * p3
            grid[ind[0], ind[1], ind[2]] += prob
            total = total + prob
    grid = grid / total


def sensor_model(tag_num, tag_range, tag_bearing):
    global grid, landmarks
    bearing = euler_from_quaternion((tag_bearing.x, tag_bearing.y, tag_bearing.z, tag_bearing.w))
    total = 0

    bearing_index = int(bearing[2] / (math.pi / 6))
    x1, y1 = (landmarks[tag_num][0] * 100, landmarks[tag_num][1] * 100)
    k1 = normalize(bearing_index * (math.pi / 6) + (math.pi / 12))

    for ind, data in numpy.ndenumerate(grid):
        x2, y2 = translation(ind[0], ind[1])
        k2 = normalize(ind[2] * (math.pi / 6) + (math.pi / 12))

        trans_cap = numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        rot1_cap = normalize(math.atan2(y2 - y1, x2 - x1) - k2)

        p1 = normal_pdf(rot1_cap, bearing[2], (math.pi / 12) + 0.05)
        p2 = normal_pdf(trans_cap, tag_range * 100, 12)

        prob = grid[ind[0], ind[1], ind[2]] * p1 * p2
        grid[ind[0], ind[1], ind[2]] = prob
        total = total + prob
    grid = grid / total


def init():
    global grid
    rospy.init_node("aviator", anonymous=False)
    rate = rospy.Rate(10)
    set_landmarks(landmarks)
    print("---------------------------------------------------------------------------------------------------")
    print("Grid Shape: ", grid.shape)
    print("Belief Threshold: 0.000001")
    print("---------------------------------------------------------------------------------------------------")
    read_bag()
    rospy.spin()


if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        raise Exception("ROSInterruptException")
        pass
