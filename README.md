# Game Controller
A Robot Swarm based search & rescue has been performed as part of the final group project for MSc Intelligent Robotics. This workspace contains one ros package `game_master`. It's designed to navigate a parrot debop dron indoor using the wall edges for avoiding collision avoidance and aruco markers to identify the take off and landing area. It can identify an accident site based on simple multi colour object detector and/or aruco markers. Once dentified it will take a photo of the accident site and search for the presense of groud robot to share a vector directing the land robot to the accident site. Once the job is accomplished, it will return to base and land on the landing pad.

# Dependencies
1. ROS kinetic
2. bebop_autonomy

# Build
`cd` to the cloned directory, then run
```
catkin_make
source devel/setup.bash
```

# Code Structure
Game master contains following executable scripts
```
manual_controller.py
flight_capture.py
autonomous_controller.py
rossocket_relay.py
rossocket_client.py
```

Before running any of the above scripts, you must be connected to the bebop wifi and launch bebop autonomy driver node.
```
roslaunch bebop_driver bebop_node.launch
```

Then `manual_controller.py` can bestarted using following command
```
rosrun game_master manual_controller.py
```
Which will let the user control the dron using keyboard. `flight_capture.py` can be used to record streamed frames from the drone for analysis or preparing training data for object detection/segmentation etc.

Autonomous controller is the heart of game master which can be launched by runing following command
```
rosrun game_master autonomous_controller.py
```
It provides full funcational manual control like the `manual_controller.py` with the addition of autonomy. Autonomus mode can be activated by pressing `x` on the keyboard or publishing an empty message to `/drone/go` rostopic.

Autonomus scrapper will publish following ROStopics when in autonomous mode
```
/drone/battery_status
/drone/intent
/drone/target
```
Battery percentil is published as Unit8 on `/drone/battery_status` topic. Whereas, Intent is an enumerator. Possible values are given bellow

| NAME                          | VALUE |
| ----------------------------- | ----- |
| `INTENT_FIND_THE_SITE`        | 0     |
| `INTENT_NAVIGATE_TO_SITE`     | 1     |
| `INTENT_FIND_GROUND_ROBOT`    | 2     |
| `INTENT_DIRECT_GROUND_ROBOT`  | 3     |
| `INTENT_RETURN_TO_BASE`       | 4     |
| `INTENT_RETURNING_TO_BASE`    | 5     |
| `INTENT_FIND_LANDING_PAD`     | 6     |
| `INTENT_PREPARE_TO_LAND`      | 7     |
| `INTENT_LAND`                 | 8     |

Drone target is published as a `Twist` message and it provides a vector direction to the land robot.

Each parameter of the `Twist` is explained bellow

| PROPERTY          | DESCRIPTION                                           |
| ----------------- | ----------------------------------------------------- |
| `target.linear.x` | Distance of the marker (Robot/Gate/Pad)               |
| `target.linear.y` | Identify the marker type<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Land Robot<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;1 - East Gate<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;2 - North Gate<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3 - Landing Pad                                                  |
| `target.linear.z` | N/A                                                   |
| `target.angular.x`| N/A                                                   |
| `target.angular.y`| N/A                                                   |
| `target.angular.z`| Orientation angle for the marker to face the target<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;+ Anti Clockwise Rotation<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Clockwise Rotation                                 |

Autonomous controller also subscribes to following topics
```
/landrobot/object_found
/drone/go
```

`/drone/go` let's the landrobot tell the drone when to begin the search operation whereas `/landrobot/object_found` would tell the drone to return to the base as the land robot has discovered the accident site already.

# Modules
Beside the executable scripts, `game_master` also containes following modules
```
drone.py
keyboard.py
measurement.py
object_detector.py
target_tracker.py
vision.py
```

`BebopDrone` class is the heart of drone control and is implimented in `drone.py`. It subscribes to topics published by `bebop_autonomy` package to update drone status like gyro scope readings, altitude, battery status, drone state, frame from drone camere etc. It also publish piloting and camera control commands to the `bebop_autonomy` package to control the drone.

`KBHit` class is used for detecting key press on the console to navigate the drone manually. It's implimented in `keyboard.py`.

`VisualMeasurement` class allow measurement of distance and angle based on drone's camera and altitude. It's implimented in `measurement.py`. Measurement of distance is more accurate at the centre of the frame where as the error appears to increase on the left and right edges.

`Detector` class is used to identify Mr. York, his car and the aruco markers, it's implimented in `object_detector.py`. Object detection is based on presense of multiple colours in the same neighbourhood and the morphology of the colour blobs.

`GoalTracker` class is the core of drone navigation using closed loop control. It allowes distance based and point based navigation using both camera and gyroscope. It's implimented in `target_tracker.py`.

`DroneVision` class allow visual navigation by detecting floor edges and calculating and orientation of floor edges. It also keeps track of different object and makers. It's implimented in `vision.py`.