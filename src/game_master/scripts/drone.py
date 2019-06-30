'''
Module contains classes and interfaces for operating bebop drone using bebop_autonomy ros package
'''

import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Controller:
    '''
    Impliments controller for the bebop drone

    ** Spin and rate shall be implimented by user module
    ** Controller.process() shall be called on every tick to process pending messages
    '''

    CAMERA_PT_SPEED = 0.5

    def __init__(self, movement_speed=0.3, turning_speed=0.5, annonymous=False):
        self.movement_speed = movement_speed
        self.turning_speed  = turning_speed
        self.status         = -1
        self.takeoff_pub    = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub       = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub       = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        rospy.init_node('drone__controller', anonymous=annonymous)

    def setSpeed(self, movement_speed=0.3, turning_speed=0.5):
        '''
        Set the speed components of the drone
        '''
        self.movement_speed = movement_speed
        self.turning_speed  = turning_speed

    def onStatusChanged(self, data):
        '''
        Callback function for drone status change
        '''

    def onBatteryLevelChanged(self, data):
        '''
        Callback function for battery level change
        '''

    def onWifiSignalChnaged(self, data):
        '''
        Callback function for wifi signal status change
        '''

    def onPositionChanged(self, data):
        '''
        Callback function for drone position changed
        '''

    def onAltitudeChanged(self, data):
        '''
        Callback function for altitude changed
        '''

    def takeoffLanding(self):
        '''
        if On Ground: Turn on the roters and start flying
        else if flying: Turn the roters off and land the drone
        '''

    def forward(self):
        '''
        Fly the drone forward
        '''

    def reverse(self):
        '''
        Reverse the drone
        '''

    def left(self):
        '''
        Fly it on the left
        '''

    def right(self):
        '''
        Fly it on the right
        '''

    def yawLeft(self):
        '''
        Yaw the drone on left
        '''

    def yawRight(self):
        '''
        Yaw the drone on right
        '''

    def ascend(self):
        '''
        Make the drone ascend
        '''

    def descend(self):
        '''
        Make the drone descend
        '''
    
    def cameraPanLeft(self):
        '''
        Pan the camera on left
        '''

    def cameraPanRight(self):
        '''
        Pan the camera on right
        '''

    def cameraTiltDown(self):
        '''
        Tilt the camera downward
        '''

    def cameraTiltUp(self):
        '''
        Tilt the camera upward
        '''

    def process(self):
        '''
        Send all pending messages and spin once
        '''
        