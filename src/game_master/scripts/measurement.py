import numpy as np

class VisualMeasurement:

    EAST    = 0
    NORTH   = 1
    WEST    = 2
    SOUTH   = 3

    def __init__(self, goalTracker, northToSouth=7.50, eastToWest=6.90):
        self.goalTracker        = goalTracker
        self.northToSouth       = northToSouth
        self.eastToWest         = eastToWest
        self.maxDistance        = max([northToSouth, eastToWest])
        self.wallInitialized    = False
        self.northWall          = 0
        self.southWall          = 0
        self.eastWall           = 0
        self.westWall           = 0
        self.walls              = []
        self._distanceCallback  = None
        self._distances         = []
        self._locateCallback    = None
        self._locatingWall      = 0
        self._localizationDists = []
        self._localizationWalls = []
        self.halfPi             = np.radians(90)

    def setNorth(self, north):
        self.wallInitialized    = True
        self.northWall          = north
        self.eastWall           = north + self.halfPi
        self.southWall          = north + (2*self.halfPi)    #Consider the rotation of Pi
        if(self.southWall > 2*self.halfPi):
            self.southWall      = north - (2*self.halfPi)
        self.westWall           = north - self.halfPi
        self.walls              = [self.eastWall, self.northWall, self.westWall, self.southWall]

    def setWall(self, wall, angle):
        '''
        Reset wall angles based on wall orientation
        '''
        if wall == self.EAST:
            north = angle - self.halfPi
            self.setNorth(north)
        elif wall == self.NORTH:
            self.setNorth(angle)
        elif wall == self.WEST:
            north = angle + self.halfPi
            self.setNorth(north)
        elif wall == self.SOUTH:
            north = angle + (2*self.halfPi)
            if(north > 2*self.halfPi):
                north = angle - (2*self.halfPi)
            self.setNorth(north)

    def _takeMeasurement(self, distance, old_distance):
        if distance < self.maxDistance + 0.5:
            self._distances.append(distance)
        if len(self._distances) >= 10:
            self.goalTracker.drone.distanceChanged = None                 # Unset distance callback
            if callable(self._distanceCallback):
                distances = np.array(self._distances)
                self._distanceCallback(np.average(distances))

    def _startMeasurements(self):
        self._distances = []
        self.goalTracker.drone.distanceChanged = self._takeMeasurement    # Set distance callback

    def getWallDistance(self, wall, callback):
        if not callable(callback):
            raise ValueError("Invalid callback function!")
        
        if not self.wallInitialized:
            raise ValueError("Wall directions aren't initialized!")

        if wall < 0 or wall >= len(self.walls):
            raise ValueError("Invalid wall distance requested!")

        self._distanceCallback = callback
        self._distances = []
        self.goalTracker.setOrientationTarget(self.walls[wall], True, self._startMeasurements, 0.1)
        
    def _onLocateWall(self, distance):
        
        self._localizationDists.append(distance)

        if self._locatingWall == self.SOUTH:
            # Normalize distances
            x = (self._localizationDists[self.SOUTH] / (self._localizationDists[self.NORTH] + self._localizationDists[self.SOUTH])) * self.northToSouth
            y = (self._localizationDists[self.WEST] / (self._localizationDists[self.EAST] + self._localizationDists[self.WEST])) * self.eastToWest
            
            if callable(self._locateCallback):
                self._locateCallback(x, y)
        else:
            self._locatingWall += 1
            self.getWallDistance(self._locatingWall, self._onLocateWall)

    def getSelfLocation(self, callback):
        if not callable(callback):
            raise ValueError("Invalid callback function!")
        
        if not self.wallInitialized:
            raise ValueError("Wall directions aren't initialized!")

        self._locateCallback    = callback
        self._locatingWall      = 0
        self._localizationDists = []
        
        self.getWallDistance(self._locatingWall, self._onLocateWall)

    def _onFastLocateWall(self, distance):
        
        self._localizationDists.append(distance)

        if self._locatingWall == self._localizationWalls[1]:
            
            if self._localizationWalls[0] == self.SOUTH:
                x = self._localizationDists[0]
            else:
                x = self.northToSouth - self._localizationDists[0]

            if self._localizationWalls[1] == self.WEST:
                y = self._localizationDists[1]
            else:
                y = self.eastToWest - self._localizationDists[1]

            if callable(self._locateCallback):
                self._locateCallback(x, y)
        else:
            self._locatingWall = self._localizationWalls[1]
            self.getWallDistance(self._locatingWall, self._onFastLocateWall)

    def getFastLocation(self, callback):
        if not callable(callback):
            raise ValueError("Invalid callback function!")
        
        if not self.wallInitialized:
            raise ValueError("Wall directions aren't initialized!")

        self._locateCallback    = callback
        self._locatingWall      = 0
        self._localizationDists = []
        self._localizationWalls = []

        #North or South
        if(abs(self.goalTracker.getAngularError(self.goalTracker.drone.yaw, self.southWall)) >
        abs(self.goalTracker.getAngularError(self.goalTracker.drone.yaw, self.northWall))):
            self._localizationWalls.append(self.NORTH)
        else:
            self._localizationWalls.append(self.SOUTH)

        #East or West
        if(abs(self.goalTracker.getAngularError(self.goalTracker.drone.yaw, self.eastWall)) >
        abs(self.goalTracker.getAngularError(self.goalTracker.drone.yaw, self.westWall))):
            self._localizationWalls.append(self.WEST)
        else:
            self._localizationWalls.append(self.EAST)

        self._locatingWall = self._localizationWalls[0]
        
        self.getWallDistance(self._locatingWall, self._onFastLocateWall)