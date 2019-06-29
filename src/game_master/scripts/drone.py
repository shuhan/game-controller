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
    '''

    CAMERA_PT_SPEED = 0.5

    def __init__(self, movement_speed = 0.3, turning_speed = 0.5):
        self.movement_speed = movement_speed
        self.turning_speed  = turning_speed
        self.status         = -1
        self.takeoff_pub    = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub       = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub       = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.rate           = rospy.Rate(100)
        rospy.init_node('drone__controller')

    def setSpeed(self, movement_speed = 0.3, turning_speed = 0.5):
        '''
        Set the speed components of the drone
        '''
        self.movement_speed = movement_speed
        self.turning_speed  = turning_speed

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
        