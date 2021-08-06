# Pose estimation

The goal to generate a 6D pose estimate for the robot.

## My approach to solution

### Environment setup

I followed the steps in this [link](https://semfire-core.slite.com/p/note/JfJtfmdjUWv2XBJTTA1yKH) to setup the environment and required packages.

**Note:** I had the following error:

```
[ERROR] [1627814098.645725029]: PluginlibFactory: The plugin for class 'rviz_plugin_tutorials/Imu' failed to load.
```

I solved it by installing the following package:

```bash
sudo apt install ros-noetic-rviz-plugin-tutorials
```

Then I run ros bag using the following command:

```
roslaunch semfire_dataset_ntu run_dataset.launch
```

### Check the rosbag

```
rosbag info 2019_11_15_raw_data.bag
```

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
topics:      /back_lslidar_packet                          1385995 msgs    : lslidar_c16_msgs 
LslidarC16Packet
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

After checking the available sensors and the data they provide, I decided to do kalman filter to build to estimate
the position and pose of the robot. I used the data provided by the Imu sensor.

#### Imu /imu/data

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

### Kalman Filter

I decided to use the kalman filter library [FilterPy](https://filterpy.readthedocs.io/en/latest/#).
This library was written in conjunction with the book [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

We need to define the matrices required for Kalman filter and the library handle the computation in two
methods `predict` and `update` which do the equation multiplication and give the new state.

#### Matrices design

Starting with the main equation `X = Fx + Bu`
The first matrix is the state matrix, which define the states we want to track.
In our problem we want to track the position and pose of the robot. The matrix will look like this

##### State matrix X

`X = [x, y, z, ddx, ddy, ddz, R, P, Y]`. where the first three elements represent the position 
and `ddx, ddy, ddz` represent the acceleration. We added the linear acceleration to the states to make use
of the acceleration provided by the IMU sensor. The last three `R, P, Y` represent the orientation of 
the robot in euler angles. 

#### State transition matrix F

It will be a 9x9 matrix which is multiplied by the states to compute the prior.

```
F = 
[[1.   0.   0.   (0.5 * self.dt**2) 0.   0.   0.   0.   0.  ]
 [0.   1.   0.   0.   (0.5 * self.dt**2) 0.   0.   0.   0.  ]
 [0.   0.   1.   0.   0.   (0.5 * self.dt**2) 0.   0.   0.  ]
 [0.   0.   0.   1.   0.   0.   0.   0.   0.  ]
 [0.   0.   0.   0.   1.   0.   0.   0.   0.  ]
 [0.   0.   0.   0.   0.   1.   0.   0.   0.  ]
 [0.   0.   0.   0.   0.   0.   1.   0.   0.  ]
 [0.   0.   0.   0.   0.   0.   0.   1.   0.  ]
 [0.   0.   0.   0.   0.   0.   0.   0.   1.  ]]
```

Where `dt` is the time step which is substituted by 0.2 after checking the different between two consecutive
time stamps of the IMU messages.

#### The state variance matrix

```
P = 
[[ 50.   0.   0.   0.   0.   0.   0.   0.   0.]
 [  0.  50.   0.   0.   0.   0.   0.   0.   0.]
 [  0.   0.  50.   0.   0.   0.   0.   0.   0.]
 [  0.   0.   0. 100.   0.   0.   0.   0.   0.]
 [  0.   0.   0.   0. 100.   0.   0.   0.   0.]
 [  0.   0.   0.   0.   0. 100.   0.   0.   0.]
 [  0.   0.   0.   0.   0.   0.   5.   0.   0.]
 [  0.   0.   0.   0.   0.   0.   0.   5.   0.]
 [  0.   0.   0.   0.   0.   0.   0.   0.   5.]]
```

This matrix represent the uncertainty we have in our state. We assumed that our states are not correlated
so all the elements of the diagonal are set to zero. The higher the values the more uncertainty
we have in the states. We specify an initial value for P and the Kalman filter updates it goes.

#### The measurement noise covariance R

For the Imu sensor we have the covariance matrix provided for the linear acceleration and the orientation

```
R = 
[[0.0015387262937311438, 0, 0, 0, 0, 0],
[0, 0.0015387262937311438, 0, 0, 0, 0],
[0, 0, 0.0015387262937311438, 0, 0, 0],
[0, 0, 0, 0.002741552146694444, 0, 0],
[0, 0, 0, 0, 0.002741552146694444, 0],
[0, 0, 0, 0, 0, 0.007615422629706791]]
```

#### The process noise Q

This matrix is represent the process noise and one on its function to keep the process vairance `P` from 
going to zero. We assumed it to be zero for simplicity.

### Code Implementation 

The kalman filter is implemented in `kal.py`

A ros node do to main functions subscribe to the `/imu/data` topic to get the sensor's reading
do a kalman filter iteration to get the new states then publish a transform between the `world` frame
and the robot base frame which contain the new position of the robot.

### Code explanation 

In the callback of the /imu/topic

```python
orientation_q = [data.orientation.x, data.orientation.y, data.orientation.y, data.orientation.w]
print("orientation_q", orientation_q)
euler_angles = euler_from_quaternion(orientation_q)
print("euler angles", euler_angles)
reading = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                euler_angles[0], euler_angles[1], euler_angles[2]])
```

We convert the readings from quaternion to euler and then we have the `reading` array which repesent
the measurement matrix in our kalman filter

```
km.robot_filter.predict()
km.robot_filter.update(reading)
```

then we do the `predict` which calculate the prior, followed by the `update` which take the measurement
to compute the new states. Note: these methods update the states internally.

```
br = TransformBroadcaster()

quat = quaternion_from_euler(*km.robot_filter.x[6:])
quat = quat/np.linalg.norm(quat)
br.sendTransform(km.robot_filter.x[0:3], quat, data.header.stamp, "bobcat_base", "World")
```

We created a transform broadcaster object which will send a transformation between a fixed `world` frame
and our robot base frame. The `sendTransform` fuction gets the rotation in quaternion so that's why
we converted the angles from euler to quaternion.
the send transform takes the translation, orientation, time stamp, the child frame and the parent frame.
