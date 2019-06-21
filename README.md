# Game Controller
Navigate search and rescue drone, create a S.L.A.M. map,  communicate with land vehicle and provide a game controller dashboard.

# Dependencies
* ROS kinetic
* bebop_autonomy
* pytorch

# Build
`cd` to the cloned directory, then run
```
catkin_make
source devel/setup.bash
```

# Running Tests
First launch connect to the bebop wifi and the launch bebop autonomy driver
```
roslaunch bebop_driver bebop_node.launch
```
Then run the drone controller
```
rosrun game_master drone_controller.py
```

# Recording data
We are mainly concerned about Image, Odom and Camera Orientation
```
rosbag record -O camera /bebop/image_raw /bebop/states/ardrone3/CameraState/Orientation /bebop/odom
```

# Playing Camera Feed
Camera feed can be played by ROS image viewer at the moment, please enter following command to start ROS image_view
```
rosrun image_view image_view image:=/bebop/image_raw raw
```

# Playing Recorded Data
Recorded data can be played back using `rosbag`
```
rosbag play [file_name]
```