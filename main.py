import copy
import json
import math
import sys
import time

import numpy as np


# How to run the STL2Voxel script:  python stltovoxel.py ./Quadcopter.stl ./stl_quad.xyz

# Extract the voxels that weren't "deleted" from the struct
def extractAxesFromMatrix(xyz):
    xs = [lst[0] for lst in xyz if lst[3]]
    ys = [lst[1] for lst in xyz if lst[3]]
    zs = [lst[2] for lst in xyz if lst[3]]
    return xs, ys, zs


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


def get_XYZ_array_from_XYZ_file(fileName):
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
    print("Finished finding X edges")

    for x in range(x_min, x_max + 1):
        for z in range(z_min, z_max + 1):
            edges = edges.union(find_shell_y(temp_y, x, z))
    print("Finished finding Y edges")

    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            edges = edges.union(find_shell_z(temp_z, x, y))
    print("Finished finding Z edges")
    return edges


def write_xyz(file_name, data, print_all=False):
    with open(file_name, "w+") as shell:
        for row in data:
            if print_all or row[3] == 1:
                shell.write("%d %d %d\n" % (row[0], row[1], row[2]))


# All points with the lowest Z value
def calculate_balance_point(mat):
    z_min = mat[mat[:, 2].argsort()][0][2]
    temp = mat[mat[:, 2] == z_min, :]

    return calculate_center_of_mass(list(temp))


# Calculating Center of Mass from using XYZ matrix
def calculate_center_of_mass(mat):
    xs, ys, zs = extractAxesFromMatrix(mat)
    center_of_mass = average_axes(xs, ys, zs)

    return center_of_mass

# Calculating the new Center of Mass according to the deleted voxels and the previous center of mass
def update_center_of_mass(center_of_mass, values_to_decrease, axis_updated_len):
    for axis in range(3):
        center_of_mass[axis] *= axis_updated_len + 1
        center_of_mass[axis] -= values_to_decrease[axis]
        center_of_mass[axis] /= axis_updated_len
    return center_of_mass


# Calculating the cutting planes coefs
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


# Calculating the distance of a point from a plane represented by the input planes coefs
def distance_from_plane(planes_coefs, point):
    length_coef = math.sqrt(math.pow(planes_coefs[0], 2) + math.pow(planes_coefs[1], 2) + math.pow(planes_coefs[2], 2))
    dot_point = (planes_coefs[0] * point[0]) + (planes_coefs[1] * point[1]) + (planes_coefs[2] * point[2]) + \
                planes_coefs[3]
    return dot_point / length_coef


# Calculating the ECoM
def get_Ecom(current_com, balance_point, values_to_decrease, axis_updated_len):
    current_com = update_center_of_mass(current_com, values_to_decrease, axis_updated_len)
    return np.linalg.norm(np.subtract(current_com[:2], balance_point[:2])), current_com


def remove_voxels(sorted_xyz, edges, balance_point, starting_com):
    edges_array = []
    for edge in edges:
        edges_array.append([e for e in edge])
    counter = 0
    current_com = [axis for axis in starting_com]
    best_com = []
    min_Ecom = sys.maxsize
    xyz_updating_len = len(sorted_xyz)
    best_alpha = copy.deepcopy(sorted_xyz)
    flag = False
    for point_index in range(len(sorted_xyz)):
        if sorted_xyz[point_index] not in edges_array:
            sorted_xyz[point_index][3] = 0
            xyz_updating_len -= 1
            current_Ecom, current_com = get_Ecom(current_com, balance_point, sorted_xyz[point_index], xyz_updating_len)
            if current_Ecom < min_Ecom:
                last_point_index = point_index
                best_com = current_com[:]
                min_Ecom = current_Ecom
                flag = True
            elif current_Ecom > min_Ecom and flag:
                sorted_xyz[last_point_index][3] = 1
                flag = False
                best_alpha[:last_point_index + 1] = sorted_xyz[:last_point_index + 1]
            counter += 1
            if counter % 3000 == 0 and counter != 0:
                print("Deleted", counter, "voxels.", "Ecom is: ", current_Ecom)
    print("Final Ecom: ", min_Ecom)
    return sorted_xyz, best_com, best_alpha


def optimize_center_of_mass(xyz, center_of_mass, balance_point, edges):
    planes_coefs = calculate_cutting_plane(center_of_mass, balance_point);
    sorted_xyz = sorted(xyz, key=lambda point: distance_from_plane(planes_coefs, point), reverse=True)
    return remove_voxels(sorted_xyz, edges, balance_point, center_of_mass)


def make_it_stand():
    item_name = str(sys.argv[1])
    xyz = get_XYZ_array_from_XYZ_file("3DFiles/Inputs/" + item_name + ".xyz")

    edges = list(find_shell(np.array(xyz)))
    write_xyz("3DFiles/Outputs/" + item_name + "_edges.xyz", edges)

    balance_point = calculate_balance_point(np.array(xyz))
    center_of_mass = calculate_center_of_mass(xyz)
    print("Before optimizing:", center_of_mass)

    start_time = time.time()
    optimized_xyz, best_com, best_alpha = optimize_center_of_mass(xyz, center_of_mass, balance_point, edges)
    end_time = time.time()
    print("After optimizing:", best_com)

    print("Optimizing Alphas took:", end_time - start_time, "seconds")

    write_xyz("3DFiles/Outputs/" + item_name + "_com_and_balance_points.xyz", [balance_point, best_com],
              print_all=True)
    write_xyz("3DFiles/Outputs/" + item_name + "_final.xyz", optimized_xyz)

    write_xyz("3DFiles/Outputs/" + item_name + "_best_alpha.xyz", best_alpha)


if __name__ == '__main__':
    make_it_stand()
