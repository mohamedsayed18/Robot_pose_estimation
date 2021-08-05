# Pose estimation

# Execute

roslaunch semfire_dataset_ntu run_dataset.launch


# Errors

`[ERROR] [1627814098.645725029]: PluginlibFactory: The plugin for class 'rviz_plugin_tutorials/Imu' failed to load.`

# Bag info

```
path:        2019_11_15_raw_data.bag
version:     2.0
duration:    13:48s (828s)
start:       Nov 15 2019 11:43:19.54 (1573807399.54)
end:         Nov 15 2019 11:57:08.34 (1573808228.34)
size:        18.8 GB
messages:    2932288
compression: none [23848/23848 chunks]
types:       geometry_msgs/TwistStamped        [98d34b0043a2093cf9d9345ab6eef12e]
             lslidar_c16_msgs/LslidarC16Packet [ae4f90a23256f44e82baa08dd45c3456]
             sensor_msgs/CameraInfo            [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/CompressedImage       [8f7a12909da2c9d3332d540a0977563f]
             sensor_msgs/Image                 [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu                   [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix             [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/TimeReference         [fded64a0265108ba86c3d38fb11c0c16]
topics:      /back_lslidar_packet                          1385995 msgs    : lslidar_c16_msgs/LslidarC16Packet
             /dalsa_camera_720p/compressed                    8270 msgs    : sensor_msgs/CompressedImage      
             /front_lslidar_packet                         1385998 msgs    : lslidar_c16_msgs/LslidarC16Packet
             /gps_fix                                          829 msgs    : sensor_msgs/NavSatFix            
             /gps_time                                         829 msgs    : sensor_msgs/TimeReference        
             /gps_vel                                          829 msgs    : geometry_msgs/TwistStamped       
             /imu/data                                       76009 msgs    : sensor_msgs/Imu                  
             /realsense/aligned_depth_to_color/image_raw     23839 msgs    : sensor_msgs/Image                
             /realsense/color/camera_info                    24845 msgs    : sensor_msgs/CameraInfo           
             /realsense/color/image_raw/compressed           24845 msgs    : sensor_msgs/CompressedImage
```

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

# PoseStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w

# Todo 

* Draw graph showing the trajectory and the change in the process noise