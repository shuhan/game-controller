'''
Module contains classes and interfaces for operating bebop drone using bebop_autonomy ros package
'''

import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged, Ardrone3PilotingStateAltitudeChanged, Ardrone3PilotingStateAttitudeChanged, Ardrone3PilotingStateSpeedChanged, Ardrone3CameraStateOrientation, CommonCommonStateBatteryStateChanged

class BebopDrone:
    '''
    Impliments Stats and Controller for the bebop drone

    ** Spin and rate shall be implimented by user module
    ** BebopDrone.process() shall be called on every tick to process pending messages
    '''

    # Camera pan and tilt speed, lets set it to 1 degree.
    CAMERA_PT_SPEED = 1

    # Flaying State Values
    FLIGHT_STATE_NOT_FLYING = 0
    FLIGHT_STATE_TAKING_OFF = 1
    FLIGHT_STATE_HOVERING   = 2
    FLIGHT_STATE_MANOEUVRING= 3
    FLIGHT_STATE_LANDING    = 4
    FLIGHT_STATE_UNKNOWN    = -1

    def __init__(self, frame_callback=None, skip=10, frequency=100, movement_speed=0.1, turning_speed=0.2, annonymous=False):
        '''
        Rospy node must be initialized before instantiating BebopDrone
        '''
        self.bridge             = CvBridge()
        self.movement_speed     = movement_speed
        self.turning_speed      = turning_speed
        self.state              = self.FLIGHT_STATE_UNKNOWN
        self.frame_callback     = frame_callback
        self.frame_skip         = skip
        # Default drone status
        self.altitude           = 0.03
        self.camera_tilt        = 0
        self.camera_pan         = 0
        self.battery            = 0
        self.roll               = 0
        self.pitch              = 0
        self.yaw                = 0
        self.speedX             = 0
        self.speedY             = 0
        self.speedZ             = 0
        self.frame              = None
        self.guideLine          = None
        self.goodGuide          = False
        self.guideDistance      = 0
        self.guideTheta         = 0
        self.guideAngularError  = 0
        self.distanceChanged    = None
        self.bearFound          = False
        self.vehicleFound       = False
        self.siteAngle          = None
        self.siteDistance       = None
        self.siteFramePosition  = None

        # Publishers
        self.takeoff_pub    = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub       = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub       = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.cam_control    = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.record_control = rospy.Publisher('/bebop/record', Bool, queue_size=1)
        # Subscribers
        self.speed_sub      = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, self.onSpeedChanged)
        self.altitude_sub   = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.onAltitudeChanged)
        self.attitude_sub   = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged, self.onAttitudeChanged)
        self.orientation_sub= rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.onCameraOrientationChanged)
        self.battery_sub    = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.onBatteryLevelChanged)
        self.status_sub     = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.onStateChanged)
        self.image_sub      = rospy.Subscriber("/bebop/image_raw", Image, self.onFrameReceived)
        # Common setup
        self.cam_cmd        = Twist()
        self.navi_cmd       = Twist()

    def setSpeed(self, movement_speed=0.3, turning_speed=0.5):
        '''
        Set the speed components of the drone
        '''
        self.movement_speed = movement_speed
        self.turning_speed  = turning_speed

    def getStateStr(self):
        if self.state == self.FLIGHT_STATE_UNKNOWN:
            return "Unknown   "
        elif self.state == self.FLIGHT_STATE_TAKING_OFF:
            return "Taking off"
        elif self.state == self.FLIGHT_STATE_NOT_FLYING:
            return "On Ground "
        elif self.state == self.FLIGHT_STATE_MANOEUVRING:
            return "Piloting  "
        elif self.state == self.FLIGHT_STATE_LANDING:
            return "Landing  "
        elif self.state == self.FLIGHT_STATE_HOVERING:
            return "Hovering  "

    #-----------------------------------------------------------------------
    # Status and update from drone
    #-----------------------------------------------------------------------

    def onStateChanged(self, data):
        '''
        Callback function for drone status change
        '''
        self.state = data.state

    def onBatteryLevelChanged(self, data):
        '''
        Callback function for battery level change
        '''
        self.battery = data.percent

    def onSpeedChanged(self, data):
        '''
        Callback function for drone speed changed
        '''
        self.speedX = data.speedX
        self.speedY = data.speedY
        self.speedZ = data.speedZ

    def onAltitudeChanged(self, data):
        '''
        Callback function for altitude changed
        '''
        self.altitude = data.altitude

    def onAttitudeChanged(self, data):
        '''
        Callback function for altitude changed
        '''
        self.roll   = data.roll
        self.pitch  = data.pitch
        self.yaw    = data.yaw

    def onCameraOrientationChanged(self, data):
        '''
        Callback function for camera orientation changed
        '''
        self.camera_tilt = data.tilt
        self.camera_pan  = data.pan

    def onFrameReceived(self, data):
        '''
        Callback function for camera frame received
        '''
        try:
            img =  self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.frame = img.copy()
            if self.frame_callback is not None:
                seq = data.header.seq
                # For now lets process every 5 frames
                if self.frame_skip == 0 or seq % self.frame_skip == 0:
                    frameTime = float(data.header.stamp.secs + float(data.header.stamp.nsecs) / 1e9)
                    self.frame_callback(self.frame, frameTime)
            cv2.imshow('Bebop', img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def setDistance(self, disatnce):
        
        if callable(self.distanceChanged):
            self.distanceChanged(disatnce, self.guideDistance)  #Ignore the not-callable error, we won't call if it's not callable
        
        self.guideDistance = disatnce

    #-----------------------------------------------------------------------
    # Control the drone
    #-----------------------------------------------------------------------

    def takeoffLanding(self):
        '''
        if On Ground: Turn on the roters and start flying
        else if flying: Turn the roters off and land the drone
        '''
        self.record_control.publish(False)
        if self.state == self.FLIGHT_STATE_NOT_FLYING or self.state == self.FLIGHT_STATE_UNKNOWN:
            self.takeoff_pub.publish()
        elif self.state == self.FLIGHT_STATE_HOVERING or self.state == self.FLIGHT_STATE_MANOEUVRING:
            self.land_pub.publish()

    def takeoff(self):
        self.takeoff_pub.publish()

    def land(self):
        self.land_pub.publish()

    #-----------------------------------------------------------------------
    # Piloting Control
    #-----------------------------------------------------------------------

    def forward(self):
        '''
        Fly the drone forward
        '''
        self.navi_cmd.linear.x = self.movement_speed
        return self

    def reverse(self):
        '''
        Reverse the drone
        '''
        self.navi_cmd.linear.x = -1 * self.movement_speed
        return self

    def moveX(self, speed):
        '''
        Move the drone forward or reverse with set speed
        '''
        self.navi_cmd.linear.x = speed
        return self

    def left(self):
        '''
        Fly it on the left
        '''
        self.navi_cmd.linear.y = self.movement_speed
        return self

    def right(self):
        '''
        Fly it on the right
        '''
        self.navi_cmd.linear.y = -1 * self.movement_speed
        return self

    def turn(self, speed):
        self.navi_cmd.angular.z = speed
        return self

    def yawLeft(self):
        '''
        Yaw the drone on left
        '''
        return self.turn(self.turning_speed)

    def yawRight(self):
        '''
        Yaw the drone on right
        '''
        return self.turn(-1 * self.turning_speed)

    def ascend(self):
        '''
        Make the drone ascend
        '''
        self.navi_cmd.linear.z = self.movement_speed
        return self

    def descend(self):
        '''
        Make the drone descend
        '''
        self.navi_cmd.linear.z = -1 * self.movement_speed
        return self

    def moveZ(self, speed):
        '''
        Move the drone up or down with set speed
        '''
        self.navi_cmd.linear.z = speed
        return self

    #-----------------------------------------------------------------------
    # Camera Control
    #-----------------------------------------------------------------------
    
    def cameraPanLeft(self):
        '''
        Pan the camera on left
        '''
        self.camera_pan -= 1
        self.cam_cmd.angular.y = self.camera_tilt
        self.cam_cmd.angular.z = self.camera_pan
        self.cam_control.publish(self.cam_cmd)
        return self

    def cameraPanRight(self):
        '''
        Pan the camera on right
        '''
        self.camera_pan += 1
        self.cam_cmd.angular.y = self.camera_tilt
        self.cam_cmd.angular.z = self.camera_pan
        self.cam_control.publish(self.cam_cmd)
        return self

    def cameraTiltDown(self):
        '''
        Tilt the camera downward
        '''
        self.camera_tilt -= 1
        self.cam_cmd.angular.y = self.camera_tilt
        self.cam_cmd.angular.z = self.camera_pan
        self.cam_control.publish(self.cam_cmd)
        return self

    def cameraTiltUp(self):
        '''
        Tilt the camera upward
        '''
        self.camera_tilt += 1
        self.cam_cmd.angular.y = self.camera_tilt
        self.cam_cmd.angular.z = self.camera_pan
        self.cam_control.publish(self.cam_cmd)
        return self

    def cameraControl(self, tilt, pan):
        self.camera_tilt = tilt
        self.camera_pan  = pan
        self.cam_cmd.angular.y = self.camera_tilt
        self.cam_cmd.angular.z = self.camera_pan
        self.cam_control.publish(self.cam_cmd)
        return self


    #-----------------------------------------------------------------------
    # Process Control
    #-----------------------------------------------------------------------

    def navigationCmdChanged(self):
        return self.navi_cmd.linear.x != 0 or self.navi_cmd.linear.y != 0 or self.navi_cmd.linear.z != 0 or self.navi_cmd.angular.x != 0 or self.navi_cmd.angular.y != 0 or self.navi_cmd.angular.z != 0
    
    def process(self):
        '''
        Send all pending messages and spin once
        '''
        # Check if Rospy shall be running
        if rospy.is_shutdown():
            return

        # Process Navigation Command
        if self.navigationCmdChanged():
            self.navi_pub.publish(self.navi_cmd)
            self.navi_cmd = Twist()