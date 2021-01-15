# kitti2pcl
This software converts Velodyne Kitti point cloud data (http://www.cvlibs.net/datasets/kitti/raw_data.php) to PCD files. 

Build:
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Usage:
```
$ ./velo2pcd --folder /DIR/TO/KITTI/velodyne_points/data --outpath /DIR/TO/OUTPUT/FOLDER/pcd_ascii -a

```
