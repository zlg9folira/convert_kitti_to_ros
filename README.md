# Kitti to ROS
This ROS PACKAGE provides two nodes for dealing with KITTI point clouds (http://www.cvlibs.net/datasets/kitti/raw_data.php). One node converts the Kitti `bin` point cloud (single or multi) to user-specified `pcd` format. The second node converts `pcd` files to ROS `bag` format given `frame_rate`, `topic_name` and `frame_name`.

![kitti](https://user-images.githubusercontent.com/35779029/104856972-5a419500-58e3-11eb-9244-8e9c50048a65.gif)

### Requirements:
```
CMake 3.2
Boost
Catkin (ROS Kinetic)
PCL 1.2
```

### Build:
Tested on Ubuntu 16.04 ROS Kinetic 
```
catkin_make --only-pkg-with-deps convert_kitti_to_ros
```

## 1- kitti2pcd
Converts Velodyne Kitti point cloud dataset to PCD files (binary or ascii output supported). Check arguments and options for more information.
### Usage:
```
 rosrun convert_kitti_to_ros kitti2pcd --folder /PATH/TO/INPUT/FOLDER/velodyne_points/data --outpath /PATH/TO/OUTPUT/FOLDER/velodyne_points/pcd_ascii -a
```
Kitti2pcd command line options are:
```
   --help or -h:     "list command line help options"
   --ascii or -a:    "save pcd as ASCII code. Default is binary"
   --file or -s:     "single file. e.g. --file PCD_0001.bin"
   --folder or -d:   "entire folder. e.g. --folder /home/data/velodyne"
   --files or -m:    "multiple files. e.g. --files 01.bin 02.bin"
   --outpath or -o": "path to where pcd files are saved. e.g. --outpath /home/data/out"
```

## 2- pcd2bag
Converts and writes PCD files (ascii) into ROSBAG file. At the same time, it publishes point cloud topic to be visualized in rviz. Note that, the publish frame rate depends on how fast the PCD files are being read from filesystem, however, the ROSBAG is created based on 10Hz frame rate (default).
### Usage:
```
 rosrun convert_kitti_to_ros pcd2bag --folder /PATH/TO/INPUT/FOLDER/velodyne_points/pcd_ascii --outpath /PATH/TO/OUTPUT/ROSBAG/velodyne_points/kitti.bag
```
pcd2bag command line options are:
```
   --help or -h:     "list command line help options"
   --folder or -d:   "entire folder. e.g. --folder /home/data/velodyne"
   --outpath or -o": "path to where pcd files are saved. e.g. --outpath /home/data/out"
```
