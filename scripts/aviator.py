#!/usr/bin/env python
import os
from os import sys, path

from tf.transformations import euler_from_quaternion

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
#
# dirpath = os.getcwd()
# print("current directory is : " + dirpath)
# foldername = os.path.basename(dirpath)
# print("Directory name is : " + foldername)
# scriptpath = os.path.realpath(__file__)
# print("Script path is : " + scriptpath)
# print("Script path is : " + path.dirname(scriptpath))
# print(path.dirname())

import rosbag
import rospy
import numpy
import math

from scripts.markers_example import test, set_landmarks, set_line_points

# Global parameters

to_radian = 0.0174533
to_degree = 57.2958

# Occupancy grid details
# Grid Boundary 7m  and cell size 20cm
grid_dimension = 700  # type: int
cell_size = 20
heading_discretization = 45  # degrees
discretization_range = 360 / heading_discretization

# shape should be (35,35,4)
grid = numpy.zeros((int(grid_dimension / cell_size), grid_dimension / cell_size, discretization_range))

# Bot's initial pos is (12,28) and 200.52 degrees. our index starts at 0.
grid[11, 27, 4] = 0.8

for i in range(int(discretization_range)):
    if i != 4:
        grid[11, 27, i] = 0.1
neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
for index, x in enumerate(neighbour_nodes):
    for i in range(int(discretization_range)):
        grid[11 + x[0], 27 + x[1], i] = 0.1


# Landmarks in meters
landmarks = [(1.25, 5.25), (1.25, 3.25), (1.25, 1.25), (4.25, 1.25), (4.25, 3.35), (4.25, 5.25)]
bag = rosbag.Bag(rospy.myargv(argv=sys.argv)[1])

line_points = []

estimation = []


def translation(x, y):
    return (x + 1) * 20 - 10, (y + 1) * 20 - 10


def normal_pdf(x, mean, var):
    variance = var
    variance = float(variance)**2
    numerator = math.exp(-(float(x)-float(mean))**2/(2*variance))
    denominator = (2 * math.pi * variance) ** .5
    return numerator/denominator


def write_to_file():
    global to_degree
    f = open(path.dirname(os.path.realpath(__file__))+"/estimation.txt", "w")
    for i in range(1, len(estimation)):
        angle = estimation[i][2] * (math.pi / 4) + (math.pi / 8)
        if i % 2 == 0:
            f.write("U: ("+str(estimation[i][0])+", "+str(estimation[i][1])+". "+str(round(angle * to_degree, 1))+")\n")
        else:
            f.write("P: ("+str(estimation[i][0])+", "+str(estimation[i][1])+". "+str(round(angle * to_degree, 1))+")\n")
    f.close()


def read_bag():
    global grid, line_points
    # try:
    for topic, msg, t in bag.read_messages(topics=["Movements", "Observations"]):
        print(msg.timeTag)
        if msg.timeTag > 4:
            break
        if topic == "Movements":
            # print("Movements", msg)
            motion_model(msg.rotation1, msg.translation, msg.rotation2)
        else:
            # print("Observations", msg)
            sensor_model(msg.tagNum, msg.range, msg.bearing)
        # limiter += 1
        # print(numpy.sum(grid))
        index = numpy.unravel_index(numpy.argmax(grid, axis=None), grid.shape)
        # print(index)
        point = (translation(index[0], index[1]))
        # line_points.append((float(point[0])/100, float(point[1])/100))
        set_line_points((float(point[0])/100, float(point[1])/100))
        estimation.append((float(point[0])/100, float(point[1])/100, index[2]))
    # test()
    write_to_file()
    # except:
    #     print("Something went wrong while reading grid.bag")
    # finally:
    bag.close()


def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def motion_model(rot1, trans, rot2):
    global to_degree, grid
    rot1 = euler_from_quaternion((rot1.x, rot1.y, rot1.z, rot1.w))
    rot2 = euler_from_quaternion((rot2.x, rot2.y, rot2.z, rot2.w))

    copy_grid = numpy.copy(grid)
    cell = (numpy.where(grid > 0.00001))
    # print(cell)
    total = 0
    for cell_index in range(cell[0].shape[0]):
        # print(cell[2][cell_index])
        x1, y1 = translation(cell[0][cell_index], cell[1][cell_index])
        k1 = normalize(cell[2][cell_index] * (math.pi / 4) + (math.pi / 8))
        # print(k1 * to_degree)
        for ind, data in numpy.ndenumerate(grid):
        # for i in numpy.arange(grid.shape[0]):
        #     for j in numpy.arange(grid.shape[1]):
        #         for k in numpy.arange(grid.shape[2]):

            x2, y2 = translation(ind[0], ind[1])
            k2 = normalize(ind[2] * (math.pi / 4) + (math.pi / 8))

            trans_cap = numpy.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            rot1_cap = normalize(math.atan2(y2 - y1, x2 - x1) - k1)
            # rot1_cap = normalize(math.atan2(y2 - y1, x2 - x1))
            rot2_cap = normalize(k2 - k1 - rot1_cap)
            # rot2_cap = normalize(rot1_cap - k2)

            # rot1_cap = normalize(k1 - rot1_cap)

            # p1 = normal_pdf(rot1_cap, rot1[2], (math.pi / 8))
            # p2 = normal_pdf(trans_cap, trans, 10)
            # p3 = normal_pdf(rot2_cap, rot2[2], (math.pi / 8))
            p1 = normal_pdf(rot1[2], rot1_cap, (math.pi / 8))
            p2 = normal_pdf(trans, trans_cap, 10)
            p3 = normal_pdf(rot2[2], rot2_cap, (math.pi / 8))

            prob = copy_grid[cell[0][cell_index], cell[1][cell_index], cell[2][cell_index]] * p1 * p2 * p3
            grid[ind[0], ind[1], ind[2]] = grid[ind[0], ind[1], ind[2]] + prob
            total = total + prob
    # print(total)
    grid = grid / total


def sensor_model(tag_num, tag_range, tag_bearing):
    global grid, landmarks
    bearing = euler_from_quaternion((tag_bearing.x, tag_bearing.y, tag_bearing.z, tag_bearing.w))
    total = 0
    bearing_index = int(bearing[2] / (math.pi / 4))
    x1, y1 = (landmarks[tag_num][0]*100, landmarks[tag_num][1]*100)
    k1 = normalize(bearing_index * (math.pi / 4) + (math.pi / 8))
    for i in numpy.arange(grid.shape[0]):
        for j in numpy.arange(grid.shape[1]):
            for k in numpy.arange(grid.shape[2]):
                x2, y2 = translation(i, j)
                k2 = normalize(k * (math.pi / 4) + (math.pi / 8))

                trans_cap = numpy.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                rot1_cap = normalize(math.atan2(y1 - y2, x1 - x2) - k1)
                # rot2_cap = k2 - rot1_cap

                # p1 = normal_pdf(rot1_cap, bearing[2], (math.pi / 8)) * 0.8
                # p2 = normal_pdf(trans_cap, tag_range*100, 10) * 0.8
                p1 = normal_pdf(bearing[2], rot1_cap, (math.pi / 8)) * 0.8
                p2 = normal_pdf(tag_range*100, trans_cap, 10) * 0.8

                prob = grid[i, j, k] * p1 * p2
                grid[i, j, k] = prob
                total = total + prob
    grid = grid / total


def init():
    rospy.init_node("aviator", anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    set_landmarks(landmarks)
    print("read bag")
    read_bag()
    rospy.spin()


if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        raise Exception("ROSInterruptException")
        pass
