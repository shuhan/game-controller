
import numpy as np
from drone import BebopDrone

class GoalTracker:
    '''
    Helps drone to achive distance, height and orientation goal
    '''

    def __init__(self, drone):
        self.drone      = drone
        self.reset()
        self.pi         = np.radians(180)

    def setHeightTarget(self, targetHeight, hold=True, callback=None):
        '''
        Adjust hight of the drone, go up or down.
        Hodling the height may interfere with swipe and land.
        Landing and Swipe shall not be combined with hold height.
        '''
        self.enableHeight       = True
        self.holdHeight         = hold
        self.heightAchived      = False
        self.heightTarget       = targetHeight
        self.heightCallback     = callback

    def resetHeightTarget(self):
        self.enableHeight       = False
        self.holdHeight         = False
        self.heightTarget       = 0
        self.heightAchived      = False
        self.heightCallback     = None
        
    def setOrientationTarget(self, targetOrientation, hold=True, callback=None, max_speed=0.1):
        self.resetSwipeTarget()
        self.enableOrientation  = True
        self.holdOrientation    = hold
        self.orientationAchived = False
        self.orientationTarget  = targetOrientation
        self.orientationCallback= callback
        self.orientationMaxSpeed=max_speed

    def resetOrientationTarget(self):
        self.enableOrientation  = False
        self.holdOrientation    = False
        self.orientationTarget  = 0
        self.orientationAchived = False
        self.orientationCallback= None
        self.orientationMaxSpeed=0.1

    def setDistanceTarget(self, targetDistance, hold=True, callback=None):
        self.enableDistance     = True
        self.holdDistance       = hold
        self.distanceAchived    = False
        self.distanceTarget     = targetDistance
        self.distanceCallback   = callback

    def resetDistanceTarget(self):
        self.enableDistance     = False
        self.holdDistance       = False
        self.distanceTarget     = 0
        self.distanceAchived    = False
        self.distanceCallback   = None

    def setSwipeTarget(self, callback=None):
        self.resetOrientationTarget()
        self.enableSwipe        = True
        self.swipeStarted       = False
        self.doneSwipe          = False
        self.swipeTarget        = self.drone.yaw
        self.swipeCallback      = callback

    def resetSwipeTarget(self):
        self.enableSwipe        = False
        self.swipeStarted       = False
        self.doneSwipe          = False
        self.swipeTarget        = 0
        self.swipeCallback      = None

    def reset(self):
        self.resetHeightTarget()
        self.resetOrientationTarget()
        self.resetDistanceTarget()
        self.resetSwipeTarget()

    def getAngularError(self, currentOrientation, targetOrientation):
        error = self.drone.yaw - self.orientationTarget
        if abs(error) > self.pi:
            sign = error/abs(error)
            error = -1 * sign * (2*self.pi - abs(error))
        return error

    def adjustHeight(self):
        '''
        Adjust drone height
        '''
        error = self.heightTarget - self.drone.altitude
        if abs(error) > 0.1:
            sign = error/abs(error)
            move = sign * min([0.1, abs(error)])
            self.drone.moveZ(move)
        else:
            if not self.heightAchived:
                self.heightAchived = True
                if self.heightCallback is not None:
                    self.heightCallback()
            
        return self.heightAchived

    def adjustOrientation(self):
        '''
        Adjust drone orientation
        '''
        error = self.getAngularError(self.drone.yaw, self.orientationTarget)
        if abs(error) > 0.1:
            sign = error/abs(error)
            turn = sign * min([self.orientationMaxSpeed, abs(error)])
            self.drone.turn(turn)
        else:
            if not self.orientationAchived:
                self.orientationAchived = True
                if self.orientationCallback is not None:
                    self.orientationCallback()
            
        return self.orientationAchived
        
    def adjustDistance(self):
        '''
        Adjust drone distance from guide line
        '''
        error = self.drone.guideDistance - self.distanceTarget
        if abs(error) > 0.2:
            sign = error/abs(error)
            move = sign * min([0.05, abs(error)])
            self.drone.moveX(move)
        else:
            if not self.distanceAchived:
                self.distanceAchived = True
                if self.distanceCallback is not None:
                    self.distanceCallback()
            
        return self.distanceAchived

    def adjustSwipe(self):
        '''
        Adjust drone swipe
        '''
        error = self.drone.yaw - self.swipeTarget
        turn = -0.1
        if self.swipeStarted:
            if abs(error) > 0.2:
                self.drone.turn(turn)
            else:
                if not self.doneSwipe:
                    self.doneSwipe = True
                    if self.swipeCallback is not None:
                        self.swipeCallback()
        else:
            # Start swiping and move away from orientation
            # Then set swipe started to True so that it doesn't assume convergence immediately
            if abs(error) > 0.5:
                self.swipeStarted = True
            
            self.drone.turn(turn)
            
        return self.doneSwipe

    def process(self):
        '''
        This shall be called on every tick so that it can update the target
        '''
        if self.enableHeight and (self.holdHeight or not self.heightAchived):
            self.adjustHeight()

        if self.enableOrientation and (self.holdOrientation or not self.orientationAchived):
            self.adjustOrientation()

        if self.enableDistance and (self.holdDistance or not self.distanceAchived):
            self.adjustDistance()

        if self.enableSwipe and not self.doneSwipe:
            self.adjustSwipe()
