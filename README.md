## Bag2PCD_ROStool ##

### 1. Build Project ###
Save the project in */catkin_ws/src* and run	
	`cd catkin_ws`; `catkin_make`
### 2. Run bag_ to_pcd Node ###
Run `roscore` on the first console. Then open a new console, run `rosrun Bag2PCD_ROStool bag_to_pcd <cloud_topic> <output_directory> `in the *catkin_ws* directory.

Where: 

***cloud_topic*** is the point cloud topic to save.

***output_directory*** is the directory on disk in which to create PCD files from the point cloud messages.

Example
```
rosrun Bag2PCD_ROStool bag_to_pcd /pandar_points ./pcd_files
```
### 3. Run Bag File ###
Open a new console, run 
```
rosbag play -r 0.5 <file_name>.bag
```
.The result files are save into the *output_directory*

### 4. Coordinate System Transformation (Optional) ###
The purpose of coordinate system rotation and translation is achieved by adjusting the following parameter values in the file [bag_to_pcd.cc](/src/bag_to_pcd.cc). All rotation parameters are angled and clockwise is positive.

> **Visualization:**

> You can monitor the transformation results using *rviz*. ` rosrun rviz rviz`

> Add the PointCloud2 type to the Rviz interface and fill in the */topic_points* in the topic section.
