# velodyne_track_person
ROS node for tracking a person in point clouds from 3D LIDAR. The task is: given the data from a Velodyne sensor on a mobile robot, there is need to detect a person moving in the room.

## Test data
Bag files for testing algorithm are awailable [here](https://drive.google.com/drive/folders/0B1lMgHXUW898d01hREY5Sk1mMUE?usp=sharing).


## Usage:
Run ROS master:
```
roscore
```
Play rosbag:
```
rosbag play --clock velodyne_1_person.bag
```
Run rviz:
```
rosrun rviz rviz
```
Running velodyne_track_person node:

```
rosrun velodyne_track_person velodyne_track_person <options>
```

Options are:

* _h=true:                                 Show help
* _res:=<res>                              Octree resolution for the Octree change detector (side length of octree voxels)
* _noise:=<noise>                                  Noise filter for the Octree change detector
* _mean_k:=<mean_k>                            Number of points to use for mean distance estimation in StatisticalOutlierRemoval noise filtering
* _std_dev_mul:=<std_dev_mul>              Standard deviation multiplier threshold for StatisticalOutlierRemoval noise filtering
* _rad:=<rad>                              Correspondence grouping size
* _min_neighbors:=<min_neighbors>          Minimum number of neighbors for RadiusOutlierRemoval noise filtering
