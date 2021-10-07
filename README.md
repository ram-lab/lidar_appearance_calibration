## Overview
This a ros package for multi lidar calibration by improving Qinghai's previous appearance-based work. Several method are to be implemented:

## Dependency List  
- PCL 1.8 ([http://pointclouds.org/](http://pointclouds.org/))
- Eigen 3
- Boost
- libpointmatcher ([https://github.com/ethz-asl/libpointmatcher](https://github.com/ethz-asl/libpointmatcher))
- Ceres-solver ([http://ceres-solver.org/](http://ceres-solver.org/))
- YAML ([https://github.com/jbeder/yaml-cpp](https://github.com/jbeder/yaml-cpp))

This package was tested on Ubuntu 16.04, ROS Kinetic

## Usage
1. Create a yaml file ```cfg.yaml``` into a fold, please follow <b>../data/example/top_tail/cfg.yaml</b> to write
2. Preproces raw pointcloud to keep points in planar surfaces. You can use the below function or <b>CloudCompare</b> software to filter redundant points:<br>
   - ```rosrun lidar_appearance_calibration calib_preprocess ../data/example/raw/ref.pcd ../data/example/raw/data.pcd ../data/example/raw/ref_filter.pcd ../data/example/raw/data_filter.pcd``` 
   - <img src="image/filter.png" width="300">
3. Extract planes from pointcloud using RANSAC <br>
	- ```rosrun lidar_appearance_calibration calib_plane_extraction pcd ../data/example/top_front/cfg.yaml```
	- ```rostopic pub /contact/icp std_msgs/String "data: ''"```
	- ```rviz -d ../rviz/plane_extraction```
4. Visualize and check the extracted plane order (same colors mean that data are associated)
   - ```rosrun pcl_ros pcd_to_pointcloud ../data/example/top_front/plane/ref_planes.pcd 1```
   - <img src="image/ref_planes.png" width="300">
   - ```rosrun pcl_ros pcd_to_pointcloud ../data/example/top_front/plane/data_planes.pcd 1```
   - <img src="image/data_planes.png" width="300">
   - ```rviz -d ../rviz/plane_extraction```
	<br>
5. Implement ICP to minimize Plane-to-Plane error
   * Auto initialization: <br>
	```rosrun lidar_appearance_calibration calib_icp ../data/example/top_front/ref_cfg.yaml ../data/example/top_front/data_cfg.yaml a```
   * Manual initialization: <br>
	```rosrun lidar_appearance_calibration calib_icp ../data/example/top_front/ref_cfg.yaml ../data/example/top_front/data_cfg.yaml m```
   * Call the program: <br>
	```rostopic pub /contact/save_plane std_msgs/String "data: ''"```
6. Visualize the calibration result <br>
	```pcl_viewer ../data/example/top_front/merged_opt.pcd```

## Result
1. Plane extraction <br>
	<img src="image/plane_extraction.png" width="300"> <br>
2. Pointcloud fusion <br>
	<img src="image/merged.png" width="300"> <br>
3. Calibration resut <br>
	<img src="image/calibration_result.png" width="600"> <br>

## Reference  
To use the code, pleace cite this paper:
```
@inproceedings{jiao2019novel,
  title={A novel dual-lidar calibration algorithm using planar surfaces},
  author={Jiao, Jianhao and Liao, Qinghai and Zhu, Yilong and Liu, Tianyu and Yu, Yang and Fan, Rui and Wang, Lujia and Liu, Ming},
  booktitle={2019 IEEE Intelligent Vehicles Symposium (IV)},
  pages={1499--1504},
  year={2019},
  organization={IEEE}
}
```
