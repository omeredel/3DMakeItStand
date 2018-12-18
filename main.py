import json
import random as rand
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

#How to run the STL2Voxel script:  python stltovoxel.py ./Quadcopter.stl ./stl_quad.xyz

def getXYZArrFromJsonFile():
    xyzArr = []
    with open('3DFiles\\torus.json') as jsonFile:
        jsonData = json.load(jsonFile)
        for vox in jsonData["voxels"]:
            for i in range(10):
                point = [int(vox["x"]), i, int(vox["z"]), 1]
                if point not in xyzArr:
                    xyzArr.append(point)
    return xyzArr

def getXYZArrFromXYZFile(fileName=None):
    xyzArr = []
    with open('3DFiles\\stl_quad.xyz') as xyzFile:
        for line in xyzFile:
            row = line.split()
            xyzArr.append([int(row[0]),int(row[1]),int(row[2]),1])
    return xyzArr


def calculateCenterOfMass(xs, ys, zs):
    x = np.average(xs)
    y = np.average(ys)
    z = np.average(zs)
    return x, y, z


def displayResult(xCom, xs, yCom, ys, zCom, zs):
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs, ys, zs)
    ax.scatter(xCom, yCom, zCom, c="Black")
    plt.show()


def optimizeCenterOfMass(xyz, reqXY, res=2):
    xs, ys, zs = extractAxesFromMatrix(xyz)
    xCom, yCom, _ = calculateCenterOfMass(xs, ys, zs)
    stop = 0
    while (abs(xCom - reqXY[0]) > res) or (abs(yCom - reqXY[1]) > res):
        for i in range(1000):
            xyz.sort()
            t = rand.randrange(len(xyz) // 2, len(xyz))
            xyz[t][3] = 0
        stop += 1
        if stop == 10: break
        xs, ys, zs = extractAxesFromMatrix(xyz)
        xCom, yCom, _ = calculateCenterOfMass(xs, ys, zs)
    print("Stopped at:", stop)
    return xyz


def createOFFFile(xyz):
    with open('test.off', "w+") as offFile:
        offFile.write("OFF\n")
        c = 0
        for lst in xyz:
            if lst[3]: c+=1
        offFile.write(str(c) + " 0 0\n")
        for point in xyz:
            if point[3]:
                strPoint = str(point[0]) + " " + str(point[1]) + " " + str(point[2]) + "\n"
                offFile.write(strPoint)


def createCoMFile(xCom, yCom, zCom):
    with open('CoM.off', "w+") as offFile:
        offFile.write("OFF\n")
        offFile.write("1 0 0\n")
        string1 = str(xCom) + " " + str(yCom) + " " + str(zCom)
        offFile.write(string1)


def doTheThing():
    xyz = getXYZArrFromXYZFile(None)
    xyz = optimizeCenterOfMass(xyz, [45, 11])
    xs, ys, zs = extractAxesFromMatrix(xyz)
    xCom, yCom, zCom = calculateCenterOfMass(xs, ys, zs)
    createOFFFile(xyz)
    createCoMFile(xCom, yCom, zCom)
    print(xCom,yCom,zCom)

    # displayResult(xCom, xs, yCom, ys, zCom, zs)  to display in matplot


def extractAxesFromMatrix(xyz):
    xs = [lst[0] for lst in xyz if lst[3]]
    ys = [lst[1] for lst in xyz if lst[3]]
    zs = [lst[2] for lst in xyz if lst[3]]
    return xs, ys, zs


if __name__ == '__main__':
    doTheThing()
