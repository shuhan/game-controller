
from drone import BebopDrone

class Goal:
    '''
    Helps drone to achive distance, height and orientation goal
    '''

    def __init__(self, drone):
        self.drone              = drone
        #
        self.enableHeight       = False
        self.holdHeight         = False
        self.heightTarget       = 0
        self.onHeight           = False
        #
        self.enableOrientation  = False
        self.holdOrientation    = False
        self.orientationTarget  = 0
        self.onOrientation      = False
        #
        self.enableDistance     = False
        self.holdDistance       = False
        self.distanceTarget     = 0
        self.onDistance         = False

    def setHeightTarget(self, targetHeight, hold=True):
        self.enableHeight       = True
        self.holdHeight         = hold
        self.onHeight           = False
        self.heightTarget       = targetHeight

    def unsetHeightTarget(self):
        self.enableHeight       = False
        self.holdHeight         = False
        self.heightTarget       = 0
        self.onHeight           = False
        
    def setOrientationTarget(self, targetOrientation, hold=True):
        self.enableOrientation  = True
        self.holdOrientation    = hold
        self.onOrientation      = False
        self.orientationTarget  = targetOrientation

    def unsetOrientationTarget(self):
        self.enableOrientation  = False
        self.holdOrientation    = False
        self.orientationTarget  = 0
        self.onOrientation      = False

    def setDistanceTarget(self, targetDistance, hold=True):
        self.enableDistance     = True
        self.holdDistance       = hold
        self.onDistance         = False
        self.distanceTarget     = targetDistance

    def unsetDistanceTarget(self):
        self.enableDistance     = False
        self.holdDistance       = False
        self.distanceTarget     = 0
        self.onDistance         = False

    def process(self):
        '''
        This shall be called on every tick so that it can update the target
        '''
