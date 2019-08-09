'''
Contains the MR. York and His vehicle detector
'''

import cv2
import numpy as np

class Detector:

    def __init__(self, drone, vfov=45, hfov=80):
        self.drone              = drone
        self.vertical_fov       = vfov
        self.horizontal_fov     = hfov
        self.hsvImage           = None
        self.expectedFloorMask  = None

    def getKMeanImage(self, origImg, K):
        img = cv2.cvtColor(origImg, cv2.COLOR_BGR2HSV)

        Z = img.reshape((-1,3))

        # convert to np.float32
        Z = np.float32(Z)

        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 1.0)
        _,label,center=cv2.kmeans(Z,K,None,criteria, 2,cv2.KMEANS_PP_CENTERS)

        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))

        return cv2.cvtColor(res2, cv2.COLOR_HSV2BGR), label, center

    def findMrYork(self, origImg, frameTime, Display=True):
        self.hsvImage    = cv2.cvtColor(origImg, cv2.COLOR_BGR2HSV)

        floor_mask  = cv2.inRange(self.hsvImage, np.array([0, 0, 120]), np.array([100, 90, 210]))

        height, width = floor_mask.shape

        # filter anything above large block for not floor as not floor
        normalized_mask = floor_mask/255

        indices = list(range(height))
        indices.reverse()

        self.expectedFloorMask = np.zeros(floor_mask.shape, normalized_mask.dtype)

        for i in indices:
            if sum(normalized_mask[i, :]) < width/2:
                break
            else:
                self.expectedFloorMask[i, :] = np.ones((1, width), normalized_mask.dtype)

        threash = 90

        self.expectedFloorMask *= 255

        bear_mask   = cv2.inRange(self.hsvImage, np.array([16, 64, 60]), np.array([30, 150, 130])) & self.expectedFloorMask
        tshirt_mask = cv2.inRange(self.hsvImage, np.array([40, 40, 32]), np.array([75, 113, 66])) & self.expectedFloorMask

        kernel = np.ones((16,16),np.uint8)

        _, bear_mask = cv2.threshold(cv2.blur(bear_mask, (5,5)), 127, 255, cv2.THRESH_BINARY)

        _, tshirt_mask = cv2.threshold(cv2.blur(tshirt_mask, (5,5)), 127, 255, cv2.THRESH_BINARY)
        tshirt_region = cv2.dilate(tshirt_mask, kernel, iterations=2)

        bear_found = np.sum(np.bitwise_and(bear_mask, tshirt_region)) > threash

        bear_bounding_box = None

        if bear_found:
            _, contours, _ = cv2.findContours(tshirt_region, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                contour = cv2.convexHull(contour)
                x,y,w,h = cv2.boundingRect(contour)
                area = cv2.contourArea(contour)
                wh_ratio = float(w)/float(h)
                
                if area > threash and area > 0.6 * w * h and wh_ratio > 0.7 and wh_ratio < 1.40 and np.sum(bear_mask[y:y+h, x:x+w])/255 > threash and np.sum(self.expectedFloorMask[y:y+h, x:x+w])/255 > (2*area)/3:
                    bear_bounding_box = (x, y, x + w, y + h)

        if Display:
            if bear_found and bear_bounding_box is not None:
                cv2.rectangle(origImg, (bear_bounding_box[0], bear_bounding_box[1]), (bear_bounding_box[2], bear_bounding_box[3]), (255, 0, 0), 2)
            cv2.imshow("Bear Found", origImg)
            cv2.imshow("Floor Mask", self.expectedFloorMask)

        return bear_found and bear_bounding_box is not None, bear_bounding_box