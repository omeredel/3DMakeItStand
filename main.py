import json
import random as rand

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def getXYZArr():
    xyzArr = []
    with open('3DFiles\\torus.json') as jsonFile:
        jsonData = json.load(jsonFile)
        for vox in jsonData["voxels"]:
            for i in range(10):
                xyzArr.append([int(vox["x"]), i, int(vox["z"]), 1])
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
        for i in range(0, 1000):
            xyz.sort()
            t = rand.randrange(len(xyz) // 2, len(xyz))
            xyz[t][3] = 0
        stop += 1
        if stop == 10: break
        xs, ys, zs = extractAxesFromMatrix(xyz)
        xCom, yCom, _ = calculateCenterOfMass(xs, ys, zs)
    print("Stopped at:", stop)
    return xyz


def doTheThing():
    xyz = getXYZArr()
    xyz = optimizeCenterOfMass(xyz, [7, 4.5])
    print(xyz)
    xs, ys, zs = extractAxesFromMatrix(xyz)
    xCom, yCom, zCom = calculateCenterOfMass(xs, ys, zs)
    print(xCom, yCom, zCom)
    displayResult(xCom, xs, yCom, ys, zCom, zs)


def extractAxesFromMatrix(xyz):
    xs = [lst[0] for lst in xyz if lst[3]]
    ys = [lst[1] for lst in xyz if lst[3]]
    zs = [lst[2] for lst in xyz if lst[3]]
    return xs, ys, zs


if __name__ == '__main__':
    doTheThing()
