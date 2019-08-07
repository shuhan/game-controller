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

    def getKMeanImage(self, origImg, K):
        img = cv2.cvtColor(origImg, cv2.COLOR_BGR2HSV)

        Z = img.reshape((-1,3))

        # convert to np.float32
        Z = np.float32(Z)

        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 1.0)
        ret,label,center=cv2.kmeans(Z,K,None,criteria, 2,cv2.KMEANS_PP_CENTERS)

        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))

        return cv2.cvtColor(res2, cv2.COLOR_HSV2BGR), label, center

    def findMrYork(self, origImg, frameTime, Display=True):

        clustImg,_,_    = self.getKMeanImage(origImg, 2)

        # hsvImage    = cv2.cvtColor(origImg, cv2.COLOR_BGR2HSV)

        # threash     = 100
        # bear_mask   = cv2.inRange(hsvImage, np.array([16, 64, 60]), np.array([30, 150, 130]))
        # tshirt_mask = cv2.inRange(hsvImage, np.array([40, 40, 32]), np.array([75, 113, 66]))
        # floor_mask  = cv2.inRange(hsvImage, np.array([0, 0, 120]), np.array([100, 90, 210]))


        # #Erode the possible tshirt mask onece and then dialate it 10 times to create a bigger region
        # kernel = np.ones((5,5),np.uint8)
        # _, tshirt_mask = cv2.threshold(cv2.blur(tshirt_mask, (5,5)), 127, 255, cv2.THRESH_BINARY)
        # tshirt_region = cv2.dilate(tshirt_mask, kernel, iterations=10)

        # #Does the region also covers bear mask
        # bear_found = np.sum(np.bitwise_and(bear_mask, tshirt_region)) > threash and np.sum(np.bitwise_and(floor_mask, tshirt_region)) > threash

        # final_region = cv2.erode(cv2.dilate(np.bitwise_and(floor_mask, tshirt_region), kernel, iterations=1), kernel, iterations=1)
        

        # if bear_found:
        #     _, contours, _ = cv2.findContours(final_region, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #     accepted_contures = []
            
        #     for contour in contours:
        #         contour = cv2.convexHull(contour)
        #         x,y,w,h = cv2.boundingRect(contour)
        #         area = cv2.contourArea(contour)
        #         wh_ratio = float(w)/float(h)
                
        #         if area > threash and area > 0.6 * w * h and np.sum(final_region[y:y+h, x:x+w])/255 > (w * h)/3 and wh_ratio > 0.7 and wh_ratio < 1.40:
        #             accepted_contures.append(contour)

        #             cv2.rectangle(origImg, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Set up the detector with default parameters.
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector()
        else : 
            detector = cv2.SimpleBlobDetector_create()

        im = cv2.cvtColor(clustImg, cv2.COLOR_BGR2GRAY)
        
        # Detect blobs.
        keypoints = detector.detect(im)
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if Display:
            cv2.imshow("Cluster", clustImg)
            cv2.imshow("Keypoints", im_with_keypoints)
            # cv2.imshow("region", final_region)
            # cv2.imshow("Bear Found", origImg)