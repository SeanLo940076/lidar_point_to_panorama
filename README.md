# lidar_point_to_panorama
[![Demo Image](./images/demo.png)]

## How to use

	roslaunch lidar_point_to_panorama panorama.launch
	roslaunch lidar_point_to_panorama panorama_all.launch 

All settings can be adjusted in the parameters.yaml file. This includes three functionalities:

1. Point cloud filtering.
2. Rotating the point cloud angle.
3. Converting 3D point cloud to 2D image, where the color indicates the distance to the object: " the closer the object, the redder it appears, and the farther away, the greener it appears"

`Ubuntu 20.04` `Noetic`