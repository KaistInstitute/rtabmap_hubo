# rtabmap_hubo


### Set up
* move rs_rgbd_hubo.launch file to /home/rainbow/catkin_ws/src/realsense/realsense2_camera/launch
* move data_grab.launch file to /home/rainbow/catkin_ws/src/mobileHubo_visionPC/drc_hubo_sensorpack/launch

### Running RTABMAP

* git clone this repository into your catkin workspace
```sh
$ roslaunch drc_hubo_sensorpack data_grab.launch 
$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/aligned_depth_to_color/camera_info rviz:=true rtabmapviz:=false frame_id:=hubo_base_link

```

### Capture data

* rosbag needed data for rtabmap + additional info
```sh
$ rosbag record -O test1.bag /hubo_lidar_output /hubo_realsenseRGBD_output /hubo_realsenseRGB_output /hubo_zedRGBD_output /hubo_zedRGB_output /tf /tf_static /imu_output /zed/rgb/camera_info_raw /camera/aligned_depth_to_color/camera_info /camera/color/camera_info /camera/color/image_raw /camera/aligned_depth_to_color/image_raw



```



