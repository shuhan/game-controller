import numpy as np

class VisualMeasurement:

    EAST    = 0
    NORTH   = 1
    SOUTH   = 2
    WEST    = 3

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

    def setNorth(self, north):
        halfPi                  = np.radians(90)
        self.wallInitialized    = True
        self.northWall          = north
        self.eastWall           = north + halfPi
        self.southWall          = north + (2*halfPi)    #Consider the rotation of Pi
        if(self.southWall > 2*halfPi):
            self.southWall      = north - (2*halfPi)
        self.westWall           = north - halfPi
        self.walls              = [self.eastWall, self.northWall, self.southWall, self.westWall]

    def _takeMeasurement(self, distance, old_distance):
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
        self.goalTracker.setOrientationTarget(self.walls[wall], True, self._startMeasurements)
        
    def _onLocateWall(self, distance):
        
        self._localizationDists.append(distance)

        if self._locatingWall == self.WEST:
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