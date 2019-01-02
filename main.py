import json
import math
import random as rand
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


# How to run the STL2Voxel script:  python stltovoxel.py ./Quadcopter.stl ./stl_quad.xyz

def getXYZArrFromJsonFile():
    xyzArr = []
    with open('3DFiles/torus.json') as jsonFile:
        jsonData = json.load(jsonFile)
        for vox in jsonData["voxels"]:
            for i in range(10):
                point = [int(vox["x"]), i, int(vox["z"]), 1]
                if point not in xyzArr:
                    xyzArr.append(point)
    return xyzArr


def get_XYZ_array_from_XYZ_file(fileName='3DFiles/stl_quad.xyz'):
    xyzArr = []
    with open(fileName) as xyzFile:
        for line in xyzFile:
            row = line.split()
            xyzArr.append([int(row[0]), int(row[1]), int(row[2]), 1])
    return xyzArr


def average_axes(xs, ys, zs):
    x = np.average(xs)
    y = np.average(ys)
    z = np.average(zs)
    return x, y, z


def display_result_in_matplotlib(xCom, xs, yCom, ys, zCom, zs):
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs, ys, zs)
    ax.scatter(xCom, yCom, zCom, c="Black")
    plt.show()


def old_optimizeCenterOfMass(xyz, reqXY, res=2):
    xs, ys, zs = extractAxesFromMatrix(xyz)
    xCom, yCom, _ = average_axes(xs, ys, zs)
    stop = 0
    while (abs(xCom - reqXY[0]) > res) or (abs(yCom - reqXY[1]) > res):
        for i in range(1000):
            xyz.sort()
            t = rand.randrange(len(xyz) // 2, len(xyz))
            xyz[t][3] = 0
        stop += 1
        if stop == 10: break
        xs, ys, zs = extractAxesFromMatrix(xyz)
        xCom, yCom, _ = average_axes(xs, ys, zs)
    print("Stopped at:", stop)
    return xyz


def createOFFFile(xyz):
    with open('test.off', "w+") as offFile:
        offFile.write("OFF\n")
        c = 0
        for lst in xyz:
            if lst[3]: c += 1
        offFile.write(str(c) + " 0 0\n")
        for point in xyz:
            if point[3]:
                strPoint = str(point[0]) + " " + str(point[1]) + " " + str(point[2]) + "\n"
                offFile.write(strPoint)


def createCoMFile(xCom, yCom, zCom):
    with open('CoM.xyz', "w+") as comFile:
        string1 = str(xCom) + " " + str(yCom) + " " + str(zCom)
        comFile.write(string1)


# the same function for x
def find_shell_x(mat, y, z):
    last_x = None
    x_edges = set()

    temp = mat[mat[:, 1] == y, :]
    temp = temp[temp[:, 2] == z, :]

    for row in temp:
        if last_x is None:
            x_edges.add((row[0], y, z, 1))
        elif (last_x + 1) < row[0]:
            x_edges.add((row[0], y, z, 1))
            x_edges.add((last_x, y, z, 1))
        last_x = row[0]

    if last_x is not None:
        x_edges.add((last_x, y, z, 1))

    return x_edges


# the same function for y
def find_shell_y(mat, x, z):
    last_y = None
    y_edges = set()

    temp = mat[mat[:, 0] == x, :]
    temp = temp[temp[:, 2] == z, :]

    for row in temp:
        if last_y is None:
            y_edges.add((x, row[1], z, 1))
        elif (last_y + 1) < row[1]:
            y_edges.add((x, row[1], z, 1))
            y_edges.add((x, last_y, z, 1))
        last_y = row[1]

    if last_y is not None:
        y_edges.add((x, last_y, z, 1))

    return y_edges


# the same function once z
def find_shell_z(mat, x, y):
    last_z = None
    z_edges = set()

    temp = mat[mat[:, 0] == x, :]
    temp = temp[temp[:, 1] == y, :]

    for row in temp:
        if last_z is None:
            z_edges.add((x, y, row[2], 1))
        elif (last_z + 1) < row[2]:
            z_edges.add((x, y, row[2], 1))
            z_edges.add((x, y, last_z, 1))
        last_z = row[2]

    if last_z is not None:
        z_edges.add((x, y, last_z, 1))
    return z_edges


def find_shell(mat):
    edges = set()

    temp_x = mat[mat[:, 0].argsort()]
    x_min = temp_x[0][0]
    x_max = temp_x[-1][0]

    temp_y = mat[mat[:, 1].argsort()]
    y_min = temp_y[0][1]
    y_max = temp_y[-1][1]

    temp_z = mat[mat[:, 2].argsort()]
    z_min = temp_z[0][2]
    z_max = temp_z[-1][2]

    for y in range(y_min, y_max + 1):
        for z in range(z_min, z_max + 1):
            edges = edges.union(find_shell_x(temp_x, y, z))
    print(".")

    for x in range(x_min, x_max + 1):
        for z in range(z_min, z_max + 1):
            edges = edges.union(find_shell_y(temp_y, x, z))

    print(".")

    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            edges = edges.union(find_shell_z(temp_z, x, y))

    print(".")
    return edges


def write_xyz(file_name, data, print_all=False):
    with open(file_name, "w+") as shell:
        for row in data:
            if print_all or row[3] == 1:
                shell.write("%d %d %d\n" % (row[0], row[1], row[2]))


def calculate_balance_point(mat):
    # all the points with the lowest Z value
    z_min = mat[mat[:, 2].argsort()][0][2]
    temp = mat[mat[:, 2] == z_min, :]

    return calculate_center_of_mass(list(temp))


def calculate_center_of_mass(mat):
    xs, ys, zs = extractAxesFromMatrix(mat)
    center_of_mass = average_axes(xs, ys, zs)

    return center_of_mass


def update_center_of_mass(center_of_mass, values_to_decrease, axis_updated_len):
    for axis in range(3):
        center_of_mass[axis] *= axis_updated_len + 1
        center_of_mass[axis] -= values_to_decrease[axis]
        center_of_mass[axis] /= axis_updated_len
    return center_of_mass


def calculate_cutting_plane(center_of_mass, balance_point):
    planes_coefs = np.subtract(center_of_mass, balance_point)
    planes_coefs[2] = 0
    planes_coefs_D = np.dot(planes_coefs, balance_point)
    planes_coefs_final = [0, 0, 0, 0]
    planes_coefs_final[:2] = planes_coefs[:2]
    planes_coefs_final[3] = -planes_coefs_D
    print("Center of mass: ", center_of_mass, "\nBalance point: ", balance_point, "\nCutting Plane coefs: ",
          planes_coefs_final)
    return planes_coefs_final


def distance_from_plane(planes_coefs, point):
    # extended_point = np.array([point[0], point[1], point[2], 1])
    length_coef = math.sqrt(math.pow(planes_coefs[0], 2) + math.pow(planes_coefs[1], 2) + math.pow(planes_coefs[2], 2))

    dot_point = (planes_coefs[0] * point[0]) + (planes_coefs[1] * point[1]) + (planes_coefs[2] * point[2]) + \
                planes_coefs[3]

    return dot_point / length_coef


def get_Ecom(current_com, balance_point, values_to_decrease, axis_updated_len):
    current_com = update_center_of_mass(current_com, values_to_decrease, axis_updated_len)
    return np.linalg.norm(np.subtract(current_com[:2], balance_point[:2])), current_com


def remove_some_points(sorted_xyz, edges, balance_point, starting_com, step=10):
    edges_array = []
    for edge in edges:
        edges_array.append([e for e in edge])
    counter = 0
    current_com = [axis for axis in starting_com]
    min_Ecom = sys.maxsize
    xyzs_on_voxels_len = len(sorted_xyz)
    for point_index in range(0, len(sorted_xyz), step):
        if sorted_xyz[point_index] not in edges_array:
            points_sum = [0, 0, 0]
            for i in range(step):
                sorted_xyz[point_index + i][3] = 0
                points_sum[0] += sorted_xyz[point_index + i][0]
                points_sum[1] += sorted_xyz[point_index + i][1]
                points_sum[2] += sorted_xyz[point_index + i][2]
            xyzs_on_voxels_len -= step
            current_Ecom, current_com = get_Ecom(current_com, balance_point, points_sum, xyzs_on_voxels_len)
            if current_Ecom > min_Ecom:
                sorted_xyz[point_index][3] = 1
                print("Stopped because of ecom")
                break
            else:
                min_Ecom = current_Ecom
            counter += 1
        if counter % 3000 == 0 and counter != 0:
            print(counter)
        if counter > len(sorted_xyz) / 2:
            print("Stopped because of number of deletions")
            break
    print(min_Ecom)
    return sorted_xyz, current_com


def optimize_center_of_mass(xyz, center_of_mass, balance_point, edges):
    planes_coefs = calculate_cutting_plane(center_of_mass, balance_point);
    sorted_xyz = sorted(xyz, key=lambda point: distance_from_plane(planes_coefs, point), reverse=True)
    return remove_some_points(sorted_xyz, edges, balance_point, center_of_mass)
    # print(sorted_xyz[0], sorted_xyz[40000], sorted_xyz[-1])
    # print(distance_from_plane(planes_coefs, sorted_xyz[0]), distance_from_plane(planes_coefs, sorted_xyz[40000]),
    #       distance_from_plane(planes_coefs, sorted_xyz[-1]))


def doTheThing():
    item_name = "extruder"
    xyz = get_XYZ_array_from_XYZ_file("3DFiles/Inputs/" + item_name + ".xyz")

    edges = list(find_shell(np.array(xyz)))
    write_xyz("3DFiles/Outputs/" + item_name + "_edges.xyz", edges)

    balance_point = calculate_balance_point(np.array(xyz))
    print(balance_point)
    center_of_mass = calculate_center_of_mass(xyz)
    print("Before optimizing: ", center_of_mass)

    optimized_xyz, center_of_mass = optimize_center_of_mass(xyz, center_of_mass, balance_point, edges)
    print("After optimizing: ", center_of_mass)

    write_xyz("3DFiles/Outputs/" + item_name + "_com_and_balance_points.xyz", [balance_point, center_of_mass],
              print_all=True)
    write_xyz("3DFiles/Outputs/" + item_name + "_test.xyz", optimized_xyz)

    # xyz = optimizeCenterOfMass(xyz, [45, 11])


# createOFFFile(xyz)
# createCoMFile(xCom, yCom, zCom)

# "3DFiles/stl_top.xyz"

# displayResult(xCom, xs, yCom, ys, zCom, zs)  #to display in matplot

# Extract the voxels that weren't "deleted" from the struct
def extractAxesFromMatrix(xyz):
    xs = [lst[0] for lst in xyz if lst[3]]
    ys = [lst[1] for lst in xyz if lst[3]]
    zs = [lst[2] for lst in xyz if lst[3]]
    # temp = xyz[xyz[:,3]==1,:]
    return xs, ys, zs


if __name__ == '__main__':
    doTheThing()
