#!/usr/bin/env python
import sys
import os
import datetime
import time

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from scipy.spatial import distance as ED
import math
from cv_bridge import CvBridge, CvBridgeError
from object_detector import Detector

class DroneVision:

    EAST_ENTRANCE_MARKER_ID     = 39
    NORTH_ENTRANCE_MARKER_ID    = 41
    LANDING_PAD_MARKER_ID       = 40
    GROUND_ROBOT_MARKER_ID      = 1
    RED_VEHICLE_MARKER_ID       = 10
    ALL_MARKER_IDS              = [GROUND_ROBOT_MARKER_ID, EAST_ENTRANCE_MARKER_ID, NORTH_ENTRANCE_MARKER_ID, LANDING_PAD_MARKER_ID, RED_VEHICLE_MARKER_ID]

    def __init__(self, drone, vfov=45, hfov=80, expectedDistance=8.0, northToSouth=8.0, eastToWest=8.0):
        self.drone                  = drone
        self.vertical_fov           = vfov
        self.horizontal_fov         = hfov
        self.expectedDistance       = expectedDistance
        self.northToSouth           = northToSouth
        self.eastToWest             = eastToWest
        self.maxDistance            = max([northToSouth, eastToWest])
        self.detector               = Detector(drone, vfov, hfov)
        self.frameTime              = 0
        self.width                  = 0
        self.height                 = 0
        self.pi                     = np.radians(180)
        # Ground robot detection parameters
        self.groundRobotVisible     = False
        self.groundRobotOrientation = None
        self.groundRobotDistance    = None
        self.groundRobotAngle       = None
        self.groundFramePosition    = None
        self.groundFrameDistance    = None
        # East gate
        self.eastGateVisible        = False
        self.eastGateOrientation    = None
        self.eastGateDistance       = None
        self.eastGateAngle          = None
        self.eastFramePosition      = None
        self.eastFrameDistance      = None
        # North gate
        self.northGateVisible       = False
        self.northGateOrientation   = None
        self.northGateDistance      = None
        self.northGateAngle         = None
        self.northFramePosition     = None
        self.northFrameDistance     = None
        # Landing pad
        self.landingPadVisible      = False
        self.landingPadOrientation  = None
        self.landingPadDistance     = None
        self.landingPadAngle        = None
        self.landingFramePosition   = None
        self.landingFrameDistance   = None
        # Bear
        self.bearVisible            = False
        self.bearDistance           = None
        self.bearAngle              = None
        self.bearFramePosition      = None
        self.bearFrameDistance      = None
        # New frame flag
        self.newFrameProcessed      = False
        self.processLine            = True
        self.processMarker          = False
        self.processBear            = False

    def angle_between(self, a, b, c):
        ang = math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0])

        if ang > self.pi:
            ang = self.pi - ang
        elif ang < -1 * self.pi:
            ang = (2*self.pi) + ang

        return ang

    def calculateFrontalDistance(self, origImg, frameTime, Display=True):

        height, width, _        = origImg.shape

        self.height             = height
        self.width              = width
        theta                   = self.drone.camera_tilt
        zeta                    = np.degrees(self.drone.pitch)

        self.newFrameProcessed  = True

        vehicle_found, vehicle_bounding_box = (False, None)
        # On every frame set visiblitiy to false and then calculate them again
        self.groundRobotVisible     = False
        self.northGateVisible       = False
        self.eastGateVisible        = False
        self.landingPadVisible      = False
        self.bearVisible            = False
        droneCentre = np.array([width/2, height])

        if self.processMarker or self.processBear:
            self.detector.setImage(origImg, frameTime)

        if self.processBear:
            # Find MR York
            bear_found, bear_bounding_box = self.detector.findMrYork(False)
            if bear_found and bear_bounding_box is not None:
                self.bearVisible        = True
                self.bearFramePosition  = ((bear_bounding_box[0] + bear_bounding_box[2])/2, (bear_bounding_box[1] + bear_bounding_box[3])/2)
                mPhi                    = (self.vertical_fov/2) - (((height - self.bearFramePosition[1]) / height) * self.vertical_fov)
                mAngle                  = np.radians(90 - (mPhi - theta - zeta))
                self.bearDistance       = self.drone.altitude * np.tan(mAngle)
                v2 = np.array(droneCentre) - np.array(self.bearFramePosition)
                self.bearFrameDistance  = np.linalg.norm(v2)
                self.bearAngle          = (float((width/2) - self.bearFramePosition[0])/float(width/2)) * (float(self.horizontal_fov)/2)

        if self.processMarker:

            corners, ids = self.detector.getMarkers()
            # initialize frame position to none
            siteFramePosition           = None

            # Marker identification
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i][0] not in self.ALL_MARKER_IDS:
                        continue
                    markerCentre = (corners[i][0][:, 0].mean(), corners[i][0][:, 1].mean())
                    cv2.circle(origImg, markerCentre, 3, (0, 255, 255), -1)
                    markerFront = (corners[i][0][0:2, 0].mean(), corners[i][0][0:2, 1].mean())
                    cv2.circle(origImg, markerFront, 3, (255, 255, 0), -1)
                    markerAngleDeg = (float((width/2) - markerCentre[0])/float(width/2)) * (float(self.horizontal_fov)/2)

                    # v1 = np.array(markerFront) - np.array(markerCentre)
                    v2 = np.array(droneCentre) - np.array(markerCentre)

                    markerOrientation   = self.angle_between(markerFront, markerCentre, droneCentre)
                    mPhi                = (self.vertical_fov/2) - (((height - markerCentre[1]) / height) * self.vertical_fov)
                    mAngle              = np.radians(90 - (mPhi - theta - zeta))
                    markerDistance      = self.drone.altitude * np.tan(mAngle)
                    markerFrameDistance = np.linalg.norm(v2)

                    if ids[i][0] == self.RED_VEHICLE_MARKER_ID:
                        vehicle_found           = True
                        vehicle_bounding_box    = [corners[i][0][0][0], corners[i][0][0][1], corners[i][0][0][0] + 40, corners[i][0][0][1] + 40]
                        siteFramePosition       = markerCentre
                    elif ids[i][0] == self.GROUND_ROBOT_MARKER_ID:
                        self.groundRobotVisible     = True
                        self.groundRobotAngle       = self.drone.yaw - np.radians(markerAngleDeg)
                        self.groundRobotDistance    = markerDistance
                        self.groundRobotOrientation = markerOrientation
                        self.groundFramePosition    = markerCentre
                        self.groundFrameDistance    = markerFrameDistance
                    elif ids[i][0] == self.EAST_ENTRANCE_MARKER_ID:
                        self.eastGateVisible        = True
                        self.eastGateAngle          = self.drone.yaw - np.radians(markerAngleDeg)
                        self.eastGateDistance       = markerDistance
                        self.eastGateOrientation    = markerOrientation
                        self.eastFramePosition      = markerCentre
                        self.eastFrameDistance      = markerFrameDistance
                    elif ids[i][0] == self.NORTH_ENTRANCE_MARKER_ID:
                        self.northGateVisible       = True
                        self.northGateAngle         = self.drone.yaw - np.radians(markerAngleDeg)
                        self.northGateDistance      = markerDistance
                        self.northGateOrientation   = markerOrientation
                        self.northFramePosition     = markerCentre
                        self.northFrameDistance     = markerFrameDistance
                    elif ids[i][0] == self.LANDING_PAD_MARKER_ID:
                        self.landingPadVisible      = True
                        self.landingPadAngle        = self.drone.yaw - np.radians(markerAngleDeg)
                        self.landingPadDistance     = markerDistance
                        self.landingPadOrientation  = markerOrientation
                        self.landingFramePosition   = markerCentre
                        self.landingFrameDistance   = markerFrameDistance

            site_found = False
            accident_site_angle = None

            if vehicle_bounding_box is not None:
                site_found              = vehicle_found
                accident_site_position  = (vehicle_bounding_box[0] + vehicle_bounding_box[2])/2
                accident_site_degree    = (float((width/2) - accident_site_position)/float(width/2)) * (float(self.horizontal_fov)/2)
                accident_site_angle     = self.drone.yaw - np.radians(accident_site_degree)

                # Site distance
                yvals                           = np.array([float(vehicle_bounding_box[1])])
                phi                             = (self.vertical_fov/2) - (((height - yvals) / height) * self.vertical_fov)
                angle                           = np.radians(90 - (phi - theta - zeta))
                distance                        = self.drone.altitude * np.tan(angle)
                self.drone.siteDistance         = np.average(distance) * 0.6
        
            self.drone.vehicleFound             = site_found
            self.drone.siteAngle                = accident_site_angle
            self.drone.siteFramePosition        = siteFramePosition
        # End of marker and site processing

        if self.processLine:
            # Navigate
            guideLine, guideTheta               = self.findFrontGuide(origImg, frameTime, False)

            # Always update the guide line here
            self.drone.guideLine                = guideLine

            if guideLine is not None:
                # Calculate distance to front guide
                yvals                           = np.array([float(guideLine[0][1]), float(guideLine[1][1])])
                phi                             = (self.vertical_fov/2) - (((height - yvals) / height) * self.vertical_fov)
                angle                           = np.radians(90 - (phi - theta - zeta))
                distance                        = self.drone.altitude * np.tan(angle)
                averageDistance                 = np.average(distance)
                if averageDistance < self.maxDistance + 0.5:       # Ignore if the distance is out of bound
                    self.drone.guideLine        = guideLine
                    self.drone.guideTheta       = guideTheta
                    self.drone.guideAngularError= np.radians(90) - guideTheta
                    self.drone.goodGuide        = averageDistance >= self.expectedDistance - 1 and averageDistance <= self.expectedDistance + 1
                    self.drone.setDistance(averageDistance)

                # Evaluate if guide is good
                # A good guide gives rough estimation with 20 cm error
                # How do we determine if the guide is a good guide

                if self.frameTime > 0 and self.drone.state != self.drone.FLIGHT_STATE_NOT_FLYING:
                    self.expectedDistance = self.expectedDistance - (self.drone.speedX * (frameTime - self.frameTime))
                    self.frameTime = self.frameTime - frameTime

                if Display:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    line_color = (0, 255, 0)
                    if not self.drone.goodGuide:
                        line_color = (0, 0, 255)
                    else:
                        if averageDistance < self.expectedDistance:
                            self.expectedDistance = averageDistance
                    cv2.line(origImg, guideLine[0], guideLine[1], line_color, 2)
                    cv2.putText(origImg,'%.2f : %.2f - %.2f : %.2f - %.2f' % (distance[0], distance[1], averageDistance, self.expectedDistance, self.drone.speedX), (10,50), font, 1, (255,255,255), 2, cv2.LINE_AA)

        if Display:
            cv2.imshow('Distance', origImg)

    def calculateIntersects(self, rho, theta, width, height):
        a = np.cos(theta)
        b = np.sin(theta)

        points = []
        intersects = [None, None, None, None]

        if b != 0:
            # Check x = 0 crossing
            x1 = 0
            y1 = int(rho/b)
            if y1 >= 0 and y1 <= height:
                points.append((x1, y1))
                intersects[0] = 1

        if a != 0:
            # Check y = 0 crossing
            x1 = int(rho/a)
            y1 = 0
            if x1 >=0 and x1 <= width:
                points.append((x1, y1))
                intersects[1] = 1

        if b != 0:
            # Check x = width
            x1 = width
            y1 = int((rho - a*width)/b)
            if y1 >= 0 and y1 <= height:
                points.append((x1, y1))
                intersects[2] = 1

        if a != 0:
            # Check y = height
            x1 = int((rho - (b*height))/a)
            y1 = height
            if x1 >=0 and x1 <= width:
                points.append((x1, y1))
                intersects[3] = 1

        return points, intersects

    def findFrontGuide(self, origImg, frameTime, Display=True):
        # start = time.time()

        im = np.float32(origImg) / 255.0

        # Calculate gradient 
        gx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=1)
        gy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=1)

        mag, _ = cv2.cartToPolar(gx, gy, angleInDegrees=True)

        _, thresh = cv2.threshold(cv2.cvtColor(mag * 255, cv2.COLOR_BGR2GRAY), 10, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(np.uint8(thresh), 100, 200, apertureSize = 3)

        lines = cv2.HoughLines(edges, 1, np.pi/180, 210)

        height, width, _ = origImg.shape

        guideLine   = None
        guideTheta  = None

        halfPi      = math.pi/2.0

        if lines is not None:
            # print("Lines found: ", lines.shape)
            for line in lines:
                for rho,theta in line:

                    if theta < halfPi - 0.2 or theta > halfPi + 0.2:
                        break

                    points, _ = self.calculateIntersects(rho, theta, width, height)

                    if guideLine is None:
                        guideLine = points
                        guideTheta = theta
                    else:
                        if (points[0][1] + points[1][1]) > guideLine[0][1] + guideLine[1][1]:
                            guideLine = points
                            guideTheta = theta

                    #cv2.line(origImg, points[0], points[1], (0,0,255), 2)
        else:
            '''
            Line not found
            '''
            # print("No line detected")

        if guideLine is not None and Display:
            cv2.line(origImg, guideLine[0], guideLine[1], (0,255,0), 2)

        if Display:
            cv2.imshow('Front Guide Line', origImg)
            cv2.imshow('Edges', edges)
            cv2.imshow('Magnitude', mag)
        
        # end = time.time()

        # print(end - start)

        return guideLine, guideTheta