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

    def setNorth(self, north):
        halfPi                  = np.radians(90)
        self.wallInitialized    = True
        self.northWall          = north
        self.eastWall           = north + halfPi
        self.southWall          = north + (2*halfPi)    #Consider the rotation of Pi
        if(self.southWall > 2*halfPi):
            self.southWall      = north - (2*halfPi)
        self.westWall           = north - halfPi
        self.walls              = [self.eastWall, self.northWall, self.westWall, self.southWall]

    def _takeMeasurement(self, distance, old_distance):
        if distance < max([self.northToSouth, self.eastToWest]):
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
        self.goalTracker.setOrientationTarget(self.walls[wall], True, self._startMeasurements, 0.8)
        
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