# 3DMakeItStand

## Introduction:
This algorithm lets users produce fabricated models, that after being 3D-printed, stand in one or more poses without requiring glue or heavy pedestals.

![Example](https://github.com/omeredel/3DMakeItStand/blob/master/readme_example.JPG)

## The Algorithm:
We use balance optimization as an energy minimization, improving stability by modifying the volume of the object, while preserving its surface details.

(This is an implementation of the ["Make It Stand: Balancing Shapes for 3D Fabrication"](https://igl.ethz.ch/projects/make-it-stand/make-it-stand-siggraph-2013-prevost-et-al.pdf) paper by Romain Prévost, Emily Whiting, Sylvain Lefebvre, Olga Sorkine-Hornung)

## Example:
![Extruder example](https://github.com/omeredel/3DMakeItStand/blob/master/extruder.gif)


## How to run:
python ./main.py [_The name of the xyz file which is inside 3DFiles/inputs directory, with the .xyz extention_]

(for example: _python ./main.py extruder_)

Outputs:
- _best_alpha file contains the best inner carving pattern
- _edges file contains only the edges of the model
- _com_and_balance_points file contains the balance_point and the final CoM

All of which are .xyz files you can load to MeshLab

To create an xyz file from an stl file you can use [https://github.com/cpederkoff/stl-to-voxel](stl-to-voxel)


