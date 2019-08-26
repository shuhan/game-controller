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
    INTENT_FIND_GROUND_ROBOT    = 2
    INTENT_DIRECT_GROUND_ROBOT  = 3
    INTENT_RETURN_TO_BASE       = 4
    INTENT_RETURNING_TO_BASE    = 5
    INTENT_PREPARE_TO_LAND      = 6
    INTENT_LAND                 = 7

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
        self.directionFixed         = False
        self.goodFound              = False
        self.frontYaw               = 0
        self.rightYaw               = 0
        self.leftYaw                = 0
        self.backYaw                = 0
        self.targetWall             = 0
        self.autonomous             = False
        self.groundSwipeCount       = 0
        self.intent                 = self.INTENT_FIND_THE_SITE
        self.groundRobotFound       = False

        self.battery_status_pub     = rospy.Publisher('/drone/battery_status', UInt8, queue_size=1)
        self.target_pub             = rospy.Publisher('/drone/target', Twist, queue_size=1)
        self.intent_pub             = rospy.Publisher('/drone/intent', UInt8, queue_size=1)
        self.landrobot_visual_sub   = rospy.Subscriber('/landrobot/object_found', String, self.landrobot_found_object, queue_size=1)

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
        print("Speed {0} Altitude {1:.2f} Tilt {2} Battery {3}% Status {4} Back Yaw {5:.2f}".format(self.drone.movement_speed, self.drone.altitude, self.drone.camera_tilt, self.drone.battery, self.drone.getStateStr(), self.backYaw))

    def landrobot_found_object(self, data):
        pass

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
            self.directionFixed = False
            self.goodFound  = False
            self.inMiddle   = False
            self.drone.takeoffLanding()
            self.kb.clear()
        elif self.char == 'x':
            self.autonomous = not self.autonomous
    
    def navigate(self):
        
        if self.char == 'w':                # Forward
            if self.inMiddle:
                self.targetWall     = self.rightYaw
            else:
                self.drone.forward()
        elif self.char == 's':              # Reverse
            if self.inMiddle:
                self.targetWall     = self.leftYaw
            else:
                self.drone.reverse()
        elif self.char == 'a':              # Left
            if self.inMiddle:
                self.targetWall     = self.frontYaw
            else:
                self.drone.left()
        elif self.char == 'd':              # Right
            if self.inMiddle:
                self.targetWall     = self.backYaw
            else:
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

    def taken_off(self):
        print("Adjusted height\n\n")
        self.goalTracker.setVisualOrientationTarget(VisualMeasurement.NORTH, self.intial_orientate)

    def intial_orientate(self):
        self.visualScale.setNorth(self.drone.yaw)
        self.goodFound      = True
        print("Orientation done\n\n")
        self.drone.cameraControl(-20, 0)
        self.goalTracker.setOrientationTarget(self.visualScale.northWall)
        self.goalTracker.setDistanceTarget(3, False, self.moved_in_middle)

    def go_up_high(self):
        self.goalTracker.setHeightTarget(1.7, False, self.moved_in_middle)

    def moved_in_middle(self):
        # Now turn right
        print("Moved in the middle\n\n")
        self.goalTracker.setOrientationTarget(self.visualScale.eastWall, True, self.turned_on_right)
        self.inMiddle   = True

    def turned_on_right(self):
        print("Turned towards right\n\n")
        self.vision.expectedDistance = self.drone.guideDistance
        self.goalTracker.setDistanceTarget(3, False, self.swipe_the_ground)

    def swipe_the_ground(self):
        print("Will swipe the ground\n\n")
        self.goalTracker.setSwipeTarget(self.measure_distance)

    def prepare_to_land(self):
        print("preparing to land\n\n")
        self.goalTracker.setHeightTarget(1.0, False, self.measure_distance)

    # >>> Visibility checks >>>

    def site_in_view(self):
        return self.drone.vehicleFound and self.drone.siteAngle is not None

    def east_in_view(self):
        return self.vision.eastGateVisible and self.vision.eastGateAngle is not None

    def north_in_view(self):
        return self.vision.northGateVisible and self.vision.northGateAngle is not None

    def landing_in_view(self):
        return self.vision.landingPadisible and self.vision.landingPadAngle is not None

    # <<< End Visibility checks <<<

    def wait_and_navigate_to_north(self):
        self.goalTracker.setValueTarget(self.site_in_view, self.navigate_to_north)

    def wait_and_navigate_to_east(self):
        self.goalTracker.setValueTarget(self.site_in_view, self.navigate_to_east)

    def wait_and_navigate_to_site(self):
        self.goalTracker.setValueTarget(self.site_in_view, self.navigate_to_site)

    def navigate_to_north(self):
        pass

    def navigate_to_east(self):
        pass

    def take_a_photo(self):
        home = expanduser("~")
        image_path  = os.path.join(home, "accident_site.png")
        cv2.imwrite(image_path, self.drone.frame)

    def navigate_to_site(self):
        print("Traveling to Accident site\n\n")
        # Adjust camera position
        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.drone.siteFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
        self.drone.cameraControl(self.drone.camera_tilt - angular_error, self.drone.camera_pan)
        self.goalTracker.setPointTarget(self.get_point_target, False, self.target_in_window)

    def target_in_window(self):
        self.goalTracker.reset()
        print("On Accident site\n\n")
        self.take_a_photo()
        self.intent = self.INTENT_FIND_GROUND_ROBOT
        self.drone.cameraControl(-20, 0)
        print("Looking for ground robot\n\n")
        self.goalTracker.setHeightTarget(1.5, False)
        self.goalTracker.setSwipeTarget(self.keep_swiping_ground)

    def keep_swiping_ground(self):
        self.groundSwipeCount += 1

        if self.groundSwipeCount < 3:
            self.goalTracker.setSwipeTarget(self.keep_swiping_ground)
        else:
            print("Couldn't find ground robot\n\n")

    def get_point_target(self):
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

    def measure_distance(self):

        if self.site_found:
            self.siteDistance = self.drone.siteDistance
            self.siteAngle  = self.drone.siteAngle

        print("Measuring distance\n\n")
        self.visualScale.getFastLocation(self.get_location)

    def get_location(self, x, y):
        print("Location is: {0:.2f}, {1:.2f}\n\n".format(x, y))

        if self.site_found:
            angularDistance = self.goalTracker.getAngularError(self.siteAngle, self.visualScale.southWall)
            siteX = x - (self.siteDistance * np.cos(angularDistance))
            siteY = y + (self.siteDistance * np.sin(angularDistance))
            print("Site Location is: {0:.2f}, {1:.2f}\n\n".format(siteX, siteY))
            print("Site distance: {0:.2f} and angle {1}\n\n".format(self.siteDistance, np.degrees(self.siteAngle)))

        self.drone.land()

    def run(self):
        self.print_help()

        self.site_found = False

        self.goalTracker.setHeightTarget(1.3, False, self.taken_off)

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

                    elif self.intent == self.INTENT_FIND_GROUND_ROBOT and self.vision.groundRobotVisible:
                        print("Ground Vehicle Found\n\n")
                        self.goalTracker.reset()
                        self.intent = self.INTENT_DIRECT_GROUND_ROBOT
                        # Adjust camera position
                        angular_error  = (self.vision.vertical_fov/2) - (((self.vision.height - self.vision.groundFramePosition[1]) / self.vision.height) * self.vision.vertical_fov)
                        self.drone.cameraControl(self.drone.camera_tilt - int(angular_error), self.drone.camera_pan)
                        # Orientate towards the robot
                        self.goalTracker.setOrientationTarget(self.vision.groundRobotAngle, False)

                        # Publish target
                        target = Twist()
                        target.linear.x    = self.vision.groundRobotDistance
                        target.angular.z   = self.vision.groundRobotOrientation

                        self.target_pub.publish(target)

                    elif self.intent == self.INTENT_DIRECT_GROUND_ROBOT and self.vision.groundRobotVisible:
                        self.goalTracker.setOrientationTarget(self.vision.groundRobotAngle, False)
                        
                        # Publish target
                        # target = Twist()
                        # target.linear.x    = self.vision.groundRobotDistance
                        # target.angular.z   = self.vision.groundRobotOrientation

                        # self.target_pub.publish(target)

                    elif self.intent == self.INTENT_RETURN_TO_BASE and (self.vision.eastGateVisible or self.vision.northGateVisible):
                        self.intent = self.INTENT_RETURNING_TO_BASE
                        if self.vision.eastFrameDistance > self.vision.northFrameDistance:
                            self.goalTracker.setOrientationTarget(self.vision.northGateAngle, False, self.wait_and_navigate_to_north)
                        else:
                            self.goalTracker.setOrientationTarget(self.vision.eastGateAngle, False, self.wait_and_navigate_to_east)
                        


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