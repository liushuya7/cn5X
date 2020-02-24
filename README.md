# cn5X++

<p align="center">
  <img src="https://github.com/fra589/cn5X/blob/master/images/XYZAB.svg" alt="5X++ Logo" />
</p>  

-------------

Nouveau panneau de contrôle Grbl 5/6 axes avec pour but d'implémenter toutes les fonctionalités de grbl-Mega-5X...  
*New 5/6 axis Grbl control panel to implement all the grbl-Mega-5X capabilities...*

## Attention ! FOR IROS PROJECT !

### Dependencies:
* conda: conda-forge channel
* Essential: vtk, pyqt5, opencv
* Camera: pyrealsense (python=3.7) 
* Automatic Registration: trimesh, scikit-learn, rtree
* All in one:
```
conda create --name laser python=3.7 vtk opencv pyqt trimesh scikit-learn rtree 
```
```
pip install pyrealsense2
```


### handeye calibration package:
* [tuw\_aruco](http://wiki.ros.org/tuw_aruco?distro=melodic)

* [marker\_rviz\_plugin](http://wiki.ros.org/marker_rviz_plugin): note that we need to replace the font **Arial** with **Liberation Sans** in src/ogre_visuals/marker.cpp

## *Warning !*
*This repository is alpha version under development. It is not intended for production.
Use at your own risk.*

## Prérequis :
cn5X++ est basé sur Python3, PyQT5 et PyQT5-QtSerialPort.  
Pour installer les prérequis sur un système Linux type Debian :
```
apt-get install python3 python3-pyqt5 python3-pyqt5.qtserialport
```

l'utilisateur doit faire partie du groupe Unix dialout pour pouvoir utiliser les ports série :  
```
adduser <username> dialout
```

-------------
cn5X++ is an open-source project and fueled by the free-time of our intrepid administrators and altruistic users. If you'd like to donate, all proceeds will be used to help fund supporting hardware and testing equipment. Thank you!

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://paypal.me/pools/c/842hNSm2It)
