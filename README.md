# pointCloud
This repository contatains the research and code for the point cloud generation part of the stereo camera biomass project. The goal of this project is to create a point cloud by combining rgb and depth images from a stereo camera. The final goal of the project is to be able to take an image of a grass field and calculate the amount of biomass that is in the picture.

There is a basic version of the point cloud generation already available. It has a couple of problems that need to be fixed. The problems are listed below:
- The point cloud is not aligned with the rgb image
- There are holes in the point cloud
- The point cloud is not accurate enough
- The depth of the pointcloud gets rendered in layers instead of a fluid transition

## Research
This part of the project has been mostly research. The research is conducted in the form of jupyter notebooks. The notebooks can be found in the [src/research](src/research) folder. There are 3 stages of notebooks: [DepthImagePCGen](DepthImagePCGen.ipynb), [ColoredPCGen](ColoredPCGen.ipynb) and [BackgroundRemoval](BackgroundRemoval.ipynb). The first notebook is the first version of the code. We started with a simple point cloud of just black points based on  the depth image. In the second notebook, the black points are replaced with colored ones based on a rgb image. In the third notebook, a system is created to remove the background of the image. This is an experimental feature for now.

## Class
For ease of use, the code is written in a class. The [documentation](src/Class/PointCloudGen.md) and [source code](src/Class/PointCloudGen.py) can be found in the [src/class](src/class) folder. The code is written in python 3.9.

## Examples
### Images
There are 2 sets of images. The [original images](images/originalImages): these images that are taken with the camera and the camera software itself. The [second set of images](images/databomb2/) are images created with the application of the project.

### Used libraries
- imageio
- numpy
- matplotlib
- open3d
  
#### Installation
```bash
pip install -r requirements.txt
```

## Features
### General
- [x] Create a pointcloud from a depth image.
- [x] Create a colored pointcloud from a rgb-d image.
- [x] Export the pointcloud to ply format.
- [x] Remove the background from the pointcloud.
- [ ] Allign the rgb and depth image in the pointcloud.
- [ ] Fix the layers in the pointcloud.

### Camera
- [ ] Fix the camera callibration values.

### Future
- The features noted above should be implemented or fixed.
- The generator process has lots of parameters that are now statically set to values that seemed to work. These parameters should be made dynamic for the user to assign.
- The post processing of the generator should be implemented.
- In following development, the class described above can be used for the base of the biomass prediction.

## Sources
### Pointcloud
[Source for the pointcloud](https://betterprogramming.pub/introduction-to-point-cloud-processing-dbda9b167534)
