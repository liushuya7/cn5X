# Introduction
To be continued

## Functions:
To be continued

# Installation:

## Dependency

* conda: conda-forge channel
* Essential: vtk, pyqt5, opencv
* Camera: pyrealsense (python=3.7) 
* Ros Communication: roslibpy
* Automatic Registration: trimesh, scikit-learn, rtree

## Installation command
```
conda create --name laser -c conda-forge python=3.7 vtk opencv pyqt trimesh scikit-learn rtree roslibpy
```
```
pip install pyrealsense2
```

## handeye calibration package:
* [tuw\_aruco](http://wiki.ros.org/tuw_aruco?distro=melodic)

* [marker\_rviz\_plugin](http://wiki.ros.org/marker_rviz_plugin): note that we need to replace the font **Arial** with **Liberation Sans** in src/ogre_visuals/marker.cpp

# ToDo & Issues
- [ ] Add test script/lib for ICP

## *Warning !*
*This repository is alpha version under development. It is not intended for production.
Use at your own risk.*
