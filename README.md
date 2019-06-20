# Game Controller
Navigate search and rescue drone, create a S.L.A.M. map,  communicate with land vehicle and provide a game controller dashboard.

# Dependencies
* ROS kinetic
* bebop_autonomy
* pytorch

# Running Tests
First launch connect to the bebop wifi and the launch bebop autonomy driver
```
roslaunch bebop_driver bebop_node.launch
```
Then run the drone controller
```
rosrun game_master drone_controller.py
```
