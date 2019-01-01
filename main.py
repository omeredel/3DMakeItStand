import json
import random as rand
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


def get_XYZ_array__from_XYZ_file(fileName='3DFiles/stl_quad.xyz'):
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


def write_xyz(file_name, data):
    with open(file_name, "w+") as shell:
        for row in data:
            if row[3] == 1:
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


def calculate_cutting_plane(center_of_mass, balance_point):
    a = center_of_mass - balance_point
    a[2] = 0
    return a


def distance_from_plane(planes_cof, point):
    extend = np.array([point[0], point[1], point[2], 1])
    length_cof = np.sqrt(np.pow(planes_cof[0], 2), np.pow(planes_cof[1], 2), np.pow(planes_cof[2], 2))

    dot_point = planes_cof.dot(extend)

    return np.abs(dot_point) / length_cof


def doTheThing():
    xyz = get_XYZ_array__from_XYZ_file("3DFiles/extruder.xyz")

    edges = list(find_shell(np.array(xyz)))
    write_xyz("edges.xyz", edges)

    balance_point = calculate_balance_point(np.array(xyz))
    print(balance_point)

    # xyz = optimizeCenterOfMass(xyz, [45, 11])
    center_of_mass = calculate_center_of_mass(xyz)
    print(center_of_mass)


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
