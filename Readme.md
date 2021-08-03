# Pose estimation

# Execute

roslaunch semfire_dataset_ntu run_dataset.launch


# Errors

`[ERROR] [1627814098.645725029]: PluginlibFactory: The plugin for class 'rviz_plugin_tutorials/Imu' failed to load.`

# Topics

```
back_lslidar_layer_num
/back_lslidar_packet
/back_lslidar_point_cloud
/back_lslidar_point_cloud_filtered
/back_lslidar_scan
/back_lslidar_scan_channel
/back_lslidar_sweep
/clicked_point
/clock
/crop_box/parameter_descriptions
/crop_box/parameter_updates
/dalsa_camera_720p/compressed
/front_lslidar_layer_num
/front_lslidar_packet
/front_lslidar_point_cloud
/front_lslidar_scan
/front_lslidar_scan_channel
/front_lslidar_sweep
/gps_fix
/gps_time
/gps_vel
/imu/data
/initialpose
/joint_states
/move_base_simple/goal
/pcl_manager/bond
/realsense/aligned_depth_to_color/image_raw
/realsense/color/camera_info
/realsense/color/image_raw
/realsense/color/image_raw/compressed
/realsense/depth/color/points
/repub_image_rect_color/compressed/parameter_descriptions
/repub_image_rect_color/compressed/parameter_updates
/rosout
/rosout_agg
/rviz/compressed/parameter_descriptions
/rviz/compressed/parameter_updates
/tf
/tf_static
```

## Topics of interest

/initial_pose: Type: geometry_msgs/PoseWithCovarianceStamped
/back_lslidar_point_cloud_filtered Type: sensor_msgs/PointCloud2
/realsense/depth/color/points sensor_msgs/PointCloud2
/front_lslidar_scan sensor_msgs/LaserScan
/realsense/color/image_raw sensor_msgs/Image
/move_base_simple/goal geometry_msgs/PoseStamped
/imu/data sensor_msgs/Imu
/joint_states sensor_msgs/JointState
/gps_fix sensor_msgs/NavSatFix
/gps_vel geometry_msgs/TwistStamped

### Imu /imu/data

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance
```

### GPS

```
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
```

### Joint states

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort
```

# Todo 

* Draw graph showing the trajectory and the change in the process noise