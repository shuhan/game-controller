#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged, Ardrone3CameraStateOrientation, CommonCommonStateBatteryStateChanged
from keyboard import KBHit
from collections import OrderedDict

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

class DroneController:

    def __init__(self):

        rospy.init_node('drone_controller')

        self.speed              = 0.2
        self.altitude           = 0
        self.camera_tilt        = 0
        self.camera_pan         = 0
        self.battery            = 0
        self.status             = ON_GROUND
        self.char               = ''
        self.kb                 = KBHit()
        # Publishers
        self.takeoff_pub        = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub           = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub           = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.takeoff_relay_pub  = rospy.Publisher('/bebop_relay/takeoff', Empty, queue_size=1)
        self.land_relay_pub     = rospy.Publisher('/bebop_relay/land', Empty, queue_size=1)
        self.navi_relay_pub     = rospy.Publisher('/bebop_relay/cmd_vel', Twist, queue_size=1)
        self.cam_control        = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        # Subscribers
        self.altitude_sub       = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.altitude_changed)
        self.orientation_sub    = rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.cammera_orientation_changed)
        self.battery_sub        = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.battery_status_changed)

        self.rate               = rospy.Rate(100)
        self.status_init        = True
        self.cam_cmd            = Twist()

    def altitude_changed(self, data):
        self.altitude = data.altitude

    def cammera_orientation_changed(self, data):
        self.camera_tilt = data.tilt
        self.camera_pan  = data.pan

    def battery_status_changed(self, data):
        self.battery = data.percent

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
        print("Speed {0} Altitude {1} Tilt {2} Pan {3} Battery {4}% Status {5}".format(self.speed, self.altitude, self.camera_tilt, self.camera_pan, self.battery, self.status))

    def adjust_speed(self):
        if self.char == '+':
            self.speed += 0.1
        if self.char == '-':
            self.speed -= 0.1
        
        if self.speed > 1.0:
            self.speed = 1.0
        if self.speed < 0.1:
            self.speed = 0.1

    def takeoff_landing(self):
        if self.char == ' ':
            if self.status == ON_GROUND:
                self.takeoff_pub.publish()
                self.takeoff_relay_pub.publish()
                self.status = TAKINGOFF
                self.print_status()
                rospy.sleep(2)
                self.kb.clear()
                self.status = ON_AIR
                
            elif self.status == ON_AIR:
                self.land_pub.publish()
                self.land_relay_pub.publish()
                self.status = LANDING
                self.print_status()
                rospy.sleep(2)
                self.kb.clear()
                self.status = ON_GROUND
        
    def navigate(self):
        if self.status != ON_AIR:
            return
        
        cmd = Twist()
        
        if self.char == 'w':                # Forward
            cmd.linear.x    = self.speed
        elif self.char == 's':              # Reverse
            cmd.linear.x    = -1 * self.speed
        elif self.char == 'a':              # Left
            cmd.linear.y   = self.speed
        elif self.char == 'd':              # Right
            cmd.linear.y   = -1 * self.speed
        elif self.char == 'q':              # YAW Left
            cmd.angular.z   = self.speed
        elif self.char == 'e':              # YAW Right
            cmd.angular.z   = -1 * self.speed
        elif self.char == 'r':              # Ascend
            cmd.linear.z    = self.speed
        elif self.char == 'f':              # Descend
            cmd.linear.z    = -1 * self.speed   
        
        self.navi_pub.publish(cmd)
        self.navi_relay_pub.publish(cmd)

    def move_cam(self):
        # 0, -25, -48, -70
        # Don't want contenious changes
        if self.char == '8':
            self.camera_tilt += 1
            self.cam_cmd.angular.y = self.camera_tilt
            self.cam_cmd.angular.z = self.camera_pan
            self.cam_control.publish(self.cam_cmd)
        if self.char == '2':
            self.camera_tilt -= 1
            self.cam_cmd.angular.y = self.camera_tilt
            self.cam_cmd.angular.z = self.camera_pan
            self.cam_control.publish(self.cam_cmd)
        if self.char == '4':
            self.camera_pan -= 1
            self.cam_cmd.angular.y = self.camera_tilt
            self.cam_cmd.angular.z = self.camera_pan
            self.cam_control.publish(self.cam_cmd)
        if self.char == '6':
            self.camera_pan += 1
            self.cam_cmd.angular.y = self.camera_tilt
            self.cam_cmd.angular.z = self.camera_pan
            self.cam_control.publish(self.cam_cmd)

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
        
        self.kb.set_normal_term()

if __name__ == "__main__":
    try:
        dc = DroneController()
        dc.run()
    except rospy.ROSInterruptException:
        pass