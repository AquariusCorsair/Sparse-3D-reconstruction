# sparse 3D Reconstruction

# Description

This program calibrated the camera and restored a Sparse 3D scene.

# Build And Run:
* Create build directory:

> mkdir build && cd build

* make .exe with cmake :

> cmake .. && make

* run .exe

> ./exe

# The result is in folder /output/
* file (PointsCloudX*,PointsCloudY*,PointsCloudZ*) is the *th reconstructed points. Use the file "ReadTest.m" to visulise them in Matlab. Or othe visulization tools.
[3D reconstruct result img](./output/3dReconstruct.png)
* file flowToImg* is the opticalFlow for img*
[move direction detection result img](./output/flow.png)

# For more info:
* [paper](https://github.com/hualiu01/Sparse3DReconstruction/blob/master/report/FinalReport.pdf)
* [demo](https://github.com/hualiu01/Sparse3DReconstruction/blob/master/report/demo.pdf)

