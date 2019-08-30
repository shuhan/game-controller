#!/usr/bin/env python
import sys
import os
from os.path import expanduser
import rospy
import numpy as np
import cv2
import random
from std_msgs.msg import Empty, String, UInt8, Bool
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged, Ardrone3CameraStateOrientation, CommonCommonStateBatteryStateChanged
from keyboard import KBHit
from collections import OrderedDict
from drone import BebopDrone
from vision import DroneVision
from target_tracker import GoalTracker
from measurement import VisualMeasurement

ON_GROUND   = "On Ground "
ON_AIR      = "On Air    "
TAKINGOFF   = "Taking off"
LANDING     = "Landing   "

KEY_CONFIGS = OrderedDict([
    ('w' , 'Forward'),
    ('s' , 'Reverse'),
    ('a' , 'Left'),
    ('d' , 'Right'),
    ('q' , 'Yaw Left'),
    ('e' , 'Yaw Right'),
    (' ' , 'Takeoff/Landing'),
    ('8' , 'Camera Tilt Up'),
    ('2' , 'Camera Tilt Down'),
    ('4' , 'Camera Pitch Left'),
    ('6' , 'Camera Pitch Right'),
    ('+' , 'Speed Up'),
    ('-' , 'Speed Down')
])

class AutonomousController:

    INTENT_FIND_THE_SITE        = 0
    INTENT_NAVIGATE_TO_SITE     = 1
    INTENT_FIND_THE_BEAR        = 2
    INTENT_FIND_GROUND_ROBOT    = 3
    INTENT_DIRECT_GROUND_ROBOT  = 4
    INTENT_RETURN_TO_BASE       = 5
    INTENT_RETURNING_TO_BASE    = 6
    INTENT_FIND_LANDING_PAD     = 7
    INTENT_PREPARE_TO_LAND      = 8
    INTENT_LAND                 = 9

    SEARCH_HEIGHT               = 1.5
    TAKE_OFF_HEIGHT             = 1.3
    LANDING_HEIGHT              = 1.0
    MIN_BATTERY_LEVEL           = 15

    def __init__(self):

        rospy.init_node('drone_controller')

        self.char                   = ''
        self.kb                     = KBHit()
        self.drone                  = BebopDrone()
        self.goalTracker            = GoalTracker(self.drone)
        self.vision                 = DroneVision(self.drone)
        self.visualScale            = VisualMeasurement(self.goalTracker)
        self.drone.frame_callback   = self.vision.calculateFrontalDistance
        self.rate                   = rospy.Rate(100)
        self.status_init            = True
        self.site_found             = False
        self.autonomous             = False
        self.groundSwipeCount       = 0
        self.intent                 = self.INTENT_FIND_THE_SITE
        self.groundRobotFound       = False
        self.landingGateTarget      = None
        self.currentSearchGround    = 0

        self.battery_status_pub     = rospy.Publisher('/drone/battery_status', UInt8, queue_size=1)
        self.target_pub             = rospy.Publisher('/drone/target', Twist, queue_size=1)
        self.intent_pub             = rospy.Publisher('/drone/intent', UInt8, queue_size=1)
        self.bear_direction_pub     = rospy.Publisher('/drone/bear_direction', String, queue_size=1)
        self.landrobot_visual_sub   = rospy.Subscriber('/landrobot/object_found', String, self.landrobot_found_object, queue_size=1)
        self.control_sub            = rospy.Subscriber('/drone/go', Empty, self.lets_play, queue_size=1)

    #-----------------------------------------------------------------------
    # Manual control
    #-----------------------------------------------------------------------
    def print_help(self):
        # Upcoming Controller
        # print('Control:')
        # for key, purose in KEY_CONFIGS.items():
        #     print("\t{} : {}".format(key, purose))
        print('Control:\nw\t: Forward\ns\t: Reverse\nd\t: Right\na\t: Left\ne\t: Yaw Right\nq\t: Yaw Left\nr\t: Ascend\nf\t: Descend\nSpace\t: Takeoff/land\n+\t: Increase Speed\n-\t: Decrease Speed')

    def print_status(self):
        if not self.status_init:
            sys.stdout.write("\033[F") # Cursor up one line
        else:
            self.status_init = False
        print("Speed {0} Altitude {1:.2f} Tilt {2} Battery {3}% Status {4} Auto pilot {5}".format(self.drone.movement_speed, self.drone.altitude, self.drone.camera_tilt, self.drone.battery, self.drone.getStateStr(), "Yes" if self.autonomous else "No "))

    def adjust_speed(self):
        if self.char == '+':
            self.drone.movement_speed += 0.1
        if self.char == '-':
            self.drone.movement_speed -= 0.1
        
        if self.drone.movement_speed > 1.0:
            self.drone.movement_speed = 1.0
        if self.drone.movement_speed < 0.1:
            self.drone.movement_speed = 0.1

    def takeoff_landing(self):
        if self.char == ' ':
            self.drone.cameraControl(0, 0)
            self.drone.takeoffLanding()
            self.kb.clear()
        elif self.char == 'x':
            self.autonomous = not self.autonomous
    
    def navigate(self):
        
        if self.char == 'w':                # Forward
            self.drone.forward()
        elif self.char == 's':              # Reverse
            self.drone.reverse()
        elif self.char == 'a':              # Left
            self.drone.left()
        elif self.char == 'd':              # Right
            self.drone.right()
        elif self.char == 'q':              # YAW Left
            self.drone.yawLeft()
        elif self.char == 'e':              # YAW Right
            self.drone.yawRight()
        elif self.char == 'r':              # Ascend
            self.drone.ascend()
        elif self.char == 'f':              # Descend
            self.drone.descend()

    def move_cam(self):
        if self.char == '8':
            self.drone.cameraTiltUp()
        if self.char == '2':
            self.drone.cameraTiltDown()
        if self.char == '4':
            self.drone.cameraPanLeft()
        if self.char == '6':
            self.drone.cameraPanRight()
    #-----------------------------------------------------------------------
    # End manual control
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Common Functions
    #-----------------------------------------------------------------------
    def lets_play(self, data):
        if self.drone.state == self.drone.FLIGHT_STATE_NOT_FLYING or self.drone.state == self.drone.FLIGHT_STATE_UNKNOWN:
            self.drone.cameraControl(0, 0)
            self.drone.takeoff()
            self.autonomous = True

    def landrobot_found_object(self, data):
        print("Ground robot found the accident site\n\n")
        if self.intent > self.INTENT_FIND_THE_BEAR:
            self.return_to_base()

    def begin_search_swipe(self, func, height):
        '''
        This will reset ground Swipe count and run the given search function, func shall be one of
            >>> look_for_ground_robot
            >>> look_for_landing_gate
            >>> look_for_landing_pad
        Each of the seach function does almost the same swipe action however, they may have slightly different camera control
        '''
        self.groundSwipeCount = 0
        if callable(func):
            self.goalTracker.setHeightTarget(height, False, func)
    
    def take_a_photo(self, name):
        home = expanduser("~")
        image_path  = os.path.join(home, name + ".png")
        cv2.imwrite(image_path, self.drone.frame)

    def return_to_base(self):
        # if drone is in any early phase
        if self.intent < self.INTENT_RETURN_TO_BASE:
            self.intent = self.INTENT_RETURN_TO_BASE
            print("Looking for gate\n\n")
            self.begin_search_swipe(self.look_for_landing_gate, self.SEARCH_HEIGHT)
    #-----------------------------------------------------------------------
    # End Common Functions
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Autonomouse control routines : Explore
    #-----------------------------------------------------------------------
    def taken_off(self):
        print("Adjusted height\n\n")
        # Start processing ground line once the dron has taken off
        self.vision.processLine = True
        self.goalTracker.setVisualOrientationTarget(VisualMeasurement.NORTH, self.intial_orientate)

    def intial_orientate(self):
        self.visualScale.setNorth(self.drone.yaw)
        print("Orientation done\n\n")
        self.drone.cameraControl(-20, 0)
        self.goalTracker.setOrientationTarget(self.visualScale.northWall)
        self.goalTracker.setDistanceTarget(3, False, self.moved_in_middle)

    def go_up_high(self):
        self.goalTracker.setHeightTarget(self.SEARCH_HEIGHT, False, self.moved_in_middle)

    def moved_in_middle(self):
        # Now turn right
        print("Moved in the middle\n\n")
        # Start processing marker only after the drone has left gantry area to avoid hiting on it.
        self.vision.processMarker = True
        self.goalTracker.setOrientationTarget(self.visualScale.eastWall, True, self.faced_east)

    def faced_east(self):
        print("Turned towards east wall\n\n")
        self.vision.expectedDistance = self.drone.guideDistance
        self.goalTracker.setDistanceTarget(3, False, self.begin_look_for_accident_site)

    def faced_south(self):
        print("Turned towards south wall\n\n")
        self.vision.expectedDistance = self.drone.guideDistance
        self.goalTracker.setDistanceTarget(3, False, self.begin_look_for_accident_site)

    def begin_look_for_accident_site(self):
        self.begin_search_swipe(self.look_for_accident_site, self.LANDING_HEIGHT)

    def look_for_accident_site(self):
        # Keep track of how many places were searched
        self.currentSearchGround += 1
        if self.groundSwipeCount == 0:
            print("Searching for accident site...")
            self.drone.cameraControl(-30, 0)    # Mid range
        elif self.groundSwipeCount == 1:
            self.drone.cameraControl(-50, 0)    # Close range
        elif self.groundSwipeCount == 2:
            self.drone.cameraControl(-20, 0)    # Long range

        if self.groundSwipeCount < 3:
            self.goalTracker.setSwipeTarget(self.look_for_accident_site)
        else:
            print("Couldn't find accident site\n\n")
            if  self.currentSearchGround > 1:
                # Just return to base
                self.return_to_base()
            else:
                self.goalTracker.setOrientationTarget(self.visualScale.southWall, True, self.faced_south)
            
        self.groundSwipeCount += 1

    def prepare_to_land(self):
        print("preparing to land\n\n")
        self.intent = self.INTENT_LAND
        self.goalTracker.setHeightTarget(self.LANDING_HEIGHT, False, self.drone.land)
    #-----------------------------------------------------------------------
    # End autonomouse control routines : Explore
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Visibility checks
    #-----------------------------------------------------------------------
    def robot_in_view(self):
        return self.vision.groundRobotVisible and self.vision.groundRobotAngle is not None

    def site_in_view(self):
        return self.drone.vehicleFound and self.drone.siteAngle is not None

    def east_in_view(self):
        return self.vision.eastGateVisible and self.vision.eastGateAngle is not None

    def north_in_view(self):
        return self.vision.northGateVisible and self.vision.northGateAngle is not None

    def landing_in_view(self):
        return self.vision.landingPadVisible and self.vision.landingPadAngle is not None
    #-----------------------------------------------------------------------
    # End Visibility checks
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Navigate to north entry
    #-----------------------------------------------------------------------
    def look_for_landing_gate(self):

        if self.groundSwipeCount == 0:
            print("Searching for landing gate...")
            self.drone.cameraControl(-15, 0)
        elif self.groundSwipeCount == 1:
            self.drone.cameraControl(-30, 0)
        elif self.groundSwipeCount == 2:
            self.drone.cameraControl(-50, 0)

        if self.groundSwipeCount < 3:
            self.goalTracker.setSwipeTarget(self.look_for_landing_gate)
        else:
            print("Couldn't find landing gate\n\n")
            # If gate marker isn't visible it's safe to just land where you are
            self.drone.land()
        
        self.groundSwipeCount += 1

    def wait_and_navigate_to_north(self):
        self.landingGateTarget = 'north'
        self.goalTracker.setValueTarget(self.north_in_view, self.navigate_to_north)

    def navigate_to_north(self):
        print("Traveling to North Gate\n\n")
        # Adjust camera position
        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.vision.northFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
        self.drone.cameraControl(self.drone.camera_tilt - angular_error, self.drone.camera_pan)
        self.goalTracker.setPointTarget(self.get_north_gate_target, False, self.begin_look_for_landing_pad)

    def get_north_gate_target(self):
        if self.vision.northGateVisible:
            self.lastKnownNorthPosition = self.vision.northFramePosition
            self.frameKnownNorthValidity = 0
        else:
            # Only allow last known possition for next 3 frames
            self.frameKnownNorthValidity += 1
            if self.frameKnownNorthValidity > 3:
                self.lastKnownNorthPosition = None

        if self.lastKnownNorthPosition is not None:
            return np.array(self.lastKnownNorthPosition), np.array([self.vision.width/2, self.vision.height/2])
        else:
            return None, np.array([self.vision.width/2, self.vision.height/2])
    #-----------------------------------------------------------------------
    # End navigate to north entry
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Navigate to east entry
    #-----------------------------------------------------------------------
    def wait_and_navigate_to_east(self):
        self.landingGateTarget = 'east'
        self.goalTracker.setValueTarget(self.east_in_view, self.navigate_to_east)

    def navigate_to_east(self):
        print("Traveling to East Gate\n\n")
        # Adjust camera position
        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.vision.eastFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
        self.drone.cameraControl(self.drone.camera_tilt - angular_error, self.drone.camera_pan)
        self.goalTracker.setPointTarget(self.get_east_gate_target, False, self.begin_look_for_landing_pad)

    def get_east_gate_target(self):
        if self.vision.eastGateVisible:
            self.lastKnownEastPosition = self.vision.eastFramePosition
            self.frameKnownEastValidity = 0
        else:
            # Only allow last known possition for next 3 frames
            self.frameKnownEastValidity += 1
            if self.frameKnownEastValidity > 3:
                self.lastKnownEastPosition = None

        if self.lastKnownEastPosition is not None:
            return np.array(self.lastKnownEastPosition), np.array([self.vision.width/2, self.vision.height/2])
        else:
            return None, np.array([self.vision.width/2, self.vision.height/2])
    #-----------------------------------------------------------------------
    # End navigate to east entry
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Navigate to landing
    #-----------------------------------------------------------------------
    def begin_look_for_landing_pad(self):
        self.intent = self.INTENT_FIND_LANDING_PAD
        self.begin_search_swipe(self.look_for_landing_pad, self.LANDING_HEIGHT)

    def look_for_landing_pad(self):

        swipeSpeed = -0.1

        if self.landingGateTarget == 'north':
            swipeSpeed = 0.1

        if self.groundSwipeCount == 0:
            self.drone.cameraControl(-30, 0)
        elif self.groundSwipeCount == 1:
            self.drone.cameraControl(-40, 0)
        elif self.groundSwipeCount == 2:
            self.drone.cameraControl(-20, 0)

        if self.groundSwipeCount < 3:
            self.goalTracker.setSwipeTarget(self.look_for_landing_pad, swipeSpeed)
        else:
            print("Couldn't find landing pad\n\n")
            # If landing pad isn't visible, it's safe to land where the drone is
            self.drone.land()
        
        self.groundSwipeCount += 1

    def wait_and_navigate_to_landing(self):
        self.goalTracker.setValueTarget(self.landing_in_view, self.navigate_to_landing)

    def navigate_to_landing(self):
        print("Traveling to Landing\n\n")
        # Adjust camera position
        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.vision.landingFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
        self.drone.cameraControl(self.drone.camera_tilt - angular_error, self.drone.camera_pan)
        self.goalTracker.setPointTarget(self.get_landing_target, False, self.face_north_and_land)

    def face_north_and_land(self):
        self.goalTracker.setOrientationTarget(self.visualScale.northWall, False, self.prepare_to_land)

    def get_landing_target(self):
        if self.vision.landingPadVisible:
            self.lastKnownLandingPosition = self.vision.landingFramePosition
            self.frameKnownLandingValidity = 0
        else:
            # Only allow last known possition for next 3 frames
            self.frameKnownLandingValidity += 1
            if self.frameKnownLandingValidity > 3:
                self.lastKnownLandingPosition = None

        if self.lastKnownLandingPosition is not None:
            return np.array(self.lastKnownLandingPosition), np.array([self.vision.width/2, self.vision.height/2])
        else:
            return None, np.array([self.vision.width/2, self.vision.height/2])
    #-----------------------------------------------------------------------
    # End navigate to landing
    #-----------------------------------------------------------------------

    #-----------------------------------------------------------------------
    # Navigate to accident site
    #-----------------------------------------------------------------------
    def wait_and_navigate_to_site(self):
        self.goalTracker.setValueTarget(self.site_in_view, self.navigate_to_site)

    def navigate_to_site(self):
        print("Traveling to Accident site\n\n")
        # Adjust camera position
        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.drone.siteFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
        self.drone.cameraControl(self.drone.camera_tilt - angular_error, self.drone.camera_pan)
        self.goalTracker.setPointTarget(self.get_accident_site_target, False, self.accident_site_in_window)

    def accident_site_in_window(self):
        self.goalTracker.reset()
        print("On Accident site\n\n")
        self.take_a_photo("accident_site")

        self.intent = self.INTENT_FIND_THE_BEAR

        # Let's see if we can look to target
        print("Looking for the bear\n\n")
        self.vision.processBear = True
        self.goalTracker.setCentreLock(self.get_accident_site_target, True)
        self.goalTracker.setSwipeTarget(self.begin_look_for_ground_robot)
    
    def begin_look_for_ground_robot(self):
        # Still the intent is to find the bear?
        if self.intent == self.INTENT_FIND_THE_BEAR:
            self.bear_direction_pub.publish(String("unknown"))
        self.goalTracker.reset()
        self.vision.processBear = False
        self.intent = self.INTENT_FIND_GROUND_ROBOT
        print("Looking for ground robot\n\n")
        self.begin_search_swipe(self.look_for_ground_robot, self.SEARCH_HEIGHT)

    def look_for_ground_robot(self):

        if self.groundSwipeCount == 0:
            self.drone.cameraControl(-20, 0)
        elif self.groundSwipeCount == 1:
            self.drone.cameraControl(-40, 0)
        elif self.groundSwipeCount == 2:
            self.drone.cameraControl(-60, 0)

        if self.groundSwipeCount < 3:
            self.goalTracker.setSwipeTarget(self.look_for_ground_robot)
        else:
            print("Couldn't find ground robot\n\n")
            # Just return to base
            self.return_to_base()
        
        self.groundSwipeCount += 1

    def get_accident_site_target(self):
        if self.drone.siteFramePosition is not None:
            self.lastKnownPosition = self.drone.siteFramePosition
            self.frameKnownValidity = 0
        else:
            # Only allow last known possition for next 3 frames
            self.frameKnownValidity += 1
            if self.frameKnownValidity > 3:
                self.lastKnownPosition = None

        if self.lastKnownPosition is not None:
            return np.array(self.lastKnownPosition), np.array([self.vision.width/2, self.vision.height/2])
        else:
            '''
            Wait for site position to be available
                - move camera up and down
                - We could also move around or swipe
            '''
            # self.drone.cameraControl(self.drone.camera_tilt - random.randint(-2, 5), self.drone.camera_pan)
            return None, np.array([self.vision.width/2, self.vision.height/2])

    def wait_and_report_location(self):
        self.goalTracker.setValueTarget(self.robot_in_view, self.report_location)

    def report_location(self):
        # Publish target
        target = Twist()
        target.linear.x    = self.vision.groundRobotDistance
        target.linear.y    = 0
        target.angular.z   = self.vision.groundRobotOrientation
        self.target_pub.publish(target)
        # Return
        self.return_to_base()
        
    #-----------------------------------------------------------------------
    # End navigate to accident site
    #-----------------------------------------------------------------------

    def run(self):
        self.print_help()

        self.goalTracker.setHeightTarget(self.TAKE_OFF_HEIGHT, False, self.taken_off)

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.kb.kbhit():
                self.char = self.kb.getch()
                self.adjust_speed()
                self.takeoff_landing()
                self.navigate()
                self.move_cam()
            self.print_status()

            # Drone autonomy
            if self.autonomous:
                if self.drone.state == self.drone.FLIGHT_STATE_MANOEUVRING or self.drone.state == self.drone.FLIGHT_STATE_HOVERING:

                    self.goalTracker.process()
                    
                    # Battery low
                    if self.drone.battery > self.drone.battery and self.drone.battery < self.MIN_BATTERY_LEVEL and self.intent < self.INTENT_RETURN_TO_BASE:
                        print("Battery low\n\n")
                        self.return_to_base()

                    if self.intent == self.INTENT_FIND_THE_SITE and self.drone.vehicleFound and self.drone.siteAngle is not None and not self.site_found:
                        # Turn off line processing
                        self.vision.processLine = False
                        # Increase number of frames per second
                        self.drone.frame_skip = 10
                        print("Site Found\n\n")
                        self.goalTracker.reset()
                        self.site_found = True
                        self.intent = self.INTENT_NAVIGATE_TO_SITE
                        self.goalTracker.setOrientationTarget(self.drone.siteAngle, False, self.wait_and_navigate_to_site)
                    elif self.intent == self.INTENT_FIND_THE_BEAR and self.vision.bearVisible and self.vision.bearFramePosition is not None and self.drone.siteFramePosition is not None:

                        self.take_a_photo("mr.york_and_car")

                        self.intent = self.INTENT_FIND_GROUND_ROBOT
                        self.begin_look_for_ground_robot()

                        bear_direction = "unknown"

                        y_diff = self.vision.bearFramePosition[1] - self.drone.siteFramePosition[1]

                        angularDistances = {
                            'east' : abs(self.goalTracker.getAngularError(self.drone.yaw, self.visualScale.eastWall)),
                            'west' : abs(self.goalTracker.getAngularError(self.drone.yaw, self.visualScale.westWall)),
                            'north': abs(self.goalTracker.getAngularError(self.drone.yaw, self.visualScale.northWall)),
                            'south': abs(self.goalTracker.getAngularError(self.drone.yaw, self.visualScale.southWall)),
                        }

                        droneHeading = min(angularDistances, key=angularDistances.get)

                        print("Drone heading: {0} Y Diff: {1}\n\n".format(droneHeading, y_diff))

                        if droneHeading     == 'east':
                            bear_direction = 'east' if y_diff < 0 else 'west'
                        elif droneHeading   == 'west':
                            bear_direction = 'west' if y_diff < 0 else 'east'
                        elif droneHeading   == 'north':
                            bear_direction = 'north' if y_diff < 0 else 'south'
                        elif droneHeading   == 'south':
                            bear_direction = 'south' if y_diff < 0 else 'north'

                        self.bear_direction_pub.publish(String(bear_direction))

                    elif self.intent == self.INTENT_FIND_GROUND_ROBOT:
                        if self.vision.groundRobotVisible:  # Vehicle found, send vector from vehicle
                            print("Ground Vehicle Found\n\n")
                            self.goalTracker.reset()
                            self.intent = self.INTENT_DIRECT_GROUND_ROBOT
                            # Adjust camera position
                            angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.vision.groundFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
                            self.drone.cameraControl(self.drone.camera_tilt - int(angular_error), self.drone.camera_pan)
                            # Orientate towards the robot
                            self.goalTracker.setOrientationTarget(self.vision.groundRobotAngle, False, self.wait_and_report_location)
                        else:
                            if self.vision.eastGateVisible:   # Supporting possition based on east gate
                                target = Twist()
                                target.linear.x    = self.vision.eastGateDistance
                                target.linear.y    = 1
                                target.angular.z   = self.vision.eastGateOrientation
                                self.target_pub.publish(target)
                            if self.vision.northGateVisible:   # Supporting possition based on north gate
                                target = Twist()
                                target.linear.x    = self.vision.northGateDistance
                                target.linear.y    = 2
                                target.angular.z   = self.vision.northGateOrientation
                                self.target_pub.publish(target)
                            if self.vision.landingPadVisible:   # Supporting possition based on landing pad
                                target = Twist()
                                target.linear.x    = self.vision.landingPadDistance
                                target.linear.y    = 3
                                target.angular.z   = self.vision.landingPadOrientation
                                self.target_pub.publish(target)

                    elif self.intent == self.INTENT_RETURN_TO_BASE and (self.vision.eastGateVisible or self.vision.northGateVisible):
                        # first cancle all previous targets
                        self.goalTracker.reset()
                        self.intent = self.INTENT_RETURNING_TO_BASE
                        
                        if not self.vision.eastGateVisible:
                            print("Navigating to north gate\n\n")
                            self.goalTracker.setOrientationTarget(self.vision.northGateAngle, False, self.wait_and_navigate_to_north)
                        elif not self.vision.northGateVisible:
                            print("Navigating to east gate\n\n")
                            self.goalTracker.setOrientationTarget(self.vision.eastGateAngle, False, self.wait_and_navigate_to_east)
                        # Both gates are visible
                        elif self.vision.eastFrameDistance > self.vision.northFrameDistance:
                            print("Navigating to north gate\n\n")
                            self.goalTracker.setOrientationTarget(self.vision.northGateAngle, False, self.wait_and_navigate_to_north)
                        else:
                            print("Navigating to east gate\n\n")
                            self.goalTracker.setOrientationTarget(self.vision.eastGateAngle, False, self.wait_and_navigate_to_east)

                    elif self.intent == self.INTENT_FIND_LANDING_PAD and self.vision.landingPadVisible:
                        self.goalTracker.reset()
                        self.intent = self.INTENT_PREPARE_TO_LAND

                        print("Navigating to landing page\n\n")
                        self.goalTracker.setOrientationTarget(self.vision.landingPadAngle, False, self.wait_and_navigate_to_landing)

                    elif self.intent == self.INTENT_LAND and self.drone.state == self.drone.FLIGHT_STATE_NOT_FLYING:
                        # Just reset parameters & Prepare the drone to launch again
                        self.drone.cameraControl(0, 0)
                        self.vision.expectedDistance    = 6.50
                        self.intent                     = self.INTENT_FIND_THE_SITE
                        self.site_found                 = False
                        self.groundRobotFound           = False
                        self.autonomous                 = False
                        self.currentSearchGround        = 0
                        # Set the initial goal back (Won't be processed as robot is not in autonomous mode)
                        self.goalTracker.setHeightTarget(self.TAKE_OFF_HEIGHT, False, self.taken_off)

                self.intent_pub.publish(UInt8(self.intent))
                self.battery_status_pub.publish(UInt8(self.drone.battery))
            
            self.drone.process()
            # End of Autonomy
        
        self.kb.set_normal_term()

if __name__ == "__main__":
    try:
        dc = AutonomousController()
        dc.run()
    except rospy.ROSInterruptException:
        pass