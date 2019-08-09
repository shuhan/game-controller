#!/usr/bin/env python
import sys
import rospy
import numpy as np
from std_msgs.msg import Empty
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

    def orientate(self):
        if self.drone.goodGuide and abs(self.drone.guideAngularError) > 0.01:
            self.drone.turn(1.0*self.drone.guideAngularError)
            return False
        else:
            return True

    def intial_orientate(self):
        if not self.directionFixed and self.drone.state != self.drone.FLIGHT_STATE_NOT_FLYING:
            self.directionFixed = self.orientate()
            if self.directionFixed:
                self.visualScale.setNorth(self.drone.yaw)
                self.goodFound      = True
                self.goalTracker.setOrientationTarget(self.visualScale.northWall)
                self.goalTracker.setDistanceTarget(3, False, self.moved_in_middle)
        return self.directionFixed

    def moved_in_middle(self):
        # Now turn right
        self.drone.cameraControl(-15, 0)
        self.goalTracker.setOrientationTarget(self.visualScale.eastWall, True, self.turned_on_right)
        self.inMiddle   = True

    def turned_on_right(self):
        self.vision.expectedDistance = self.drone.guideDistance
        self.goalTracker.setDistanceTarget(3, False, self.go_up_high)

    def go_up_high(self):
        self.goalTracker.setHeightTarget(1.5, True, self.prepare_to_land)

    # def swipe_the_ground(self):
    #     self.goalTracker.setSwipeTarget(self.prepare_to_land)

    def prepare_to_land(self):
        self.goalTracker.setHeightTarget(1.0, False, self.measure_distance)

    def measure_distance(self):
        self.visualScale.getFastLocation(self.get_location)

    def get_location(self, x, y):
        print("Location is: {0:.2f}, {1:.2f}\n\n\r".format(x, y))
        self.drone.land()

    def run(self):
        self.print_help()

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
                    if not self.directionFixed:
                        self.intial_orientate()
                    else:
                        ''' '''    
                self.goalTracker.process()

                if self.drone.vehicleFound:
                    print("Accident Site found\n\n")
                    self.autonomous = False

            self.drone.process()
            # End of Autonomy
        
        self.kb.set_normal_term()

if __name__ == "__main__":
    try:
        dc = AutonomousController()
        dc.run()
    except rospy.ROSInterruptException:
        pass