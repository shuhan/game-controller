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

# Plans

## Plan C
Let jialin control the dron capture all movements and then play it back.
Use traditional computer vision techniques with blob detection and AR codes and so on to capture the image of MR. York.
Can be expanded to blob based floor and object detection using ground facing camera.

## Plan B
Use AR tags to correct orientation and use plan C like movements from point to point to make it more accurate.
Either use traditional CV or machine learning based CV depending on what is ready.

## Plan A
Use machine learning based CV, model movements and control signal, use sensor fusion for localization and mapping. Create SLAM map for navigating ground vehicle.

### Note
Make sure we follow from plan C to A so that we are ready with plan C for worst case and build incrimentally.
