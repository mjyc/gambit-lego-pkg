import os
import numpy as np

import cv2.cv as cv
import cv2
from cv2 import imshow, imwrite, waitKey, destroyWindow

# Local
import util2 as util
from config import DEFAULT_PARAMS_PATH


class interactiveBaseTable:
    def __init__(self,videoSize,nTableFrames):
        self.nTableFrames = nTableFrames
        self.depArr = [np.zeros(videoSize, "uint16"),] * self.nTableFrames
        self.ind = 0
        
    def setDep(self,dep):
        if self.ind < self.nTableFrames:
            self.depArr[self.ind] = dep
            self.ind += 1
            print self.ind
        else:
            print "[interactiveBaseTable] collecting more data then we need!"

    def computeMinMap(self):
        flatTable = util.buildMinMapFrames(self.depArr)

        # Show for verification
        imshow("Flat table depths", flatTable*10)
        waitKey(0)

        # Clean up
        imwrite(os.path.join(DEFAULT_PARAMS_PATH, "flatTable.png"), flatTable)
        for frame in self.depArr: del frame

        destroyWindow("Flat table depths")
        return flatTable
        

# def interactiveBaseTable((img,dep),videoSize,liveNTableFrames):
#     tmpDepArr = [np.zeros(videoSize, "uint16"),] * liveNTableFrames
    
#     for i in range(liveNTableFrames):
#         tmpDepArr[i] = dep
    
#     flatTable = util.buildMinMapFrames(tmpDepArr)

#     # Show for verification
#     imshow("Flat table depths", flatTable*10)
#     waitKey(0)

#     # Clean up
#     imwrite(os.path.join(DEFAULT_PARAMS_PATH, "flatTable.png"), flatTable)
#     for frame in tmpDepArr: del frame

#     destroyWindow("Flat table depths")
#     return flatTable


def interactiveMask((img,dep)):
    # Event handler
    pointQueue = []    
    def handleMouseClick(event, x, y, flags, parameters):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            pointQueue.append((x,y))
            #print pointQueue

    # Ask user to click points on depth image
    #imshow("Click mask", dep*15)
    imshow("Click mask", img)
    
    cv2.namedWindow("Click mask", 1)
    cv.SetMouseCallback("Click mask", handleMouseClick, None);
    key = 0
    print "[interactiveMask] Press SPACE when finished (ESC to quit):"
    while (key) != 32:
        key = waitKey(10) % 256
        if key == 27:
            print "Quit program"
            exit(1)
    cv2.destroyWindow("Click mask")
    print "[interactiveMask] final point queue:", pointQueue
    print ""

    # Show masked for verification
    mask = np.zeros(dep.shape, dtype="uint8")
    cv2.fillPoly(mask, [np.array(pointQueue, dtype="int32")], (255,255,255))
    #imshow("mask", mask)
    
    img2 = img.copy()
    imshow("masked img",img2)
    img2[mask == 0] = 0;
    imshow("masked img", img2)
    
    dep2 = dep.copy()
    dep2[mask == 0] = 0;
    imshow("masked dep", dep2*15)
    waitKey(0)

    # Write to file
    imwrite(os.path.join(DEFAULT_PARAMS_PATH, "mask.png"), mask)

    # Clean finish
    destroyWindow("Click mask")
    destroyWindow("masked img")
    destroyWindow("masked dep")
    return ~mask

# NOTE: make sure the line created by 2nd and 4th points are crossing with hands
def interactiveLines((img,dep), mask):
    # Event handler
    pointQueue = []    
    def handleMouseClick(event, x, y, flags, parameters):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            pointQueue.append((x,y))
            #print pointQueue

    # Ask user to click points on depth image
    dep[mask > 0] = 0
    imshow("Click lines", dep*15)
    
    cv2.namedWindow("Click lines", 1)
    cv.SetMouseCallback("Click lines", handleMouseClick, None);    
    key = 0    
    print "[interactiveLines] Click 4 points. Press SPACE when finished (ESC to quit):"
    while (key) != 32:
        key = waitKey(10) % 256
        if key == 27:
            print "Quit program"
            exit(1)
    print "[interactiveLines] final point queue:", pointQueue

    # lines
    print "[interactiveLines] pointQueue",pointQueue
    plines = zip(pointQueue[:-1], pointQueue[1:])
    print "[interactiveLines] plines",plines
    print ""    

    # write return
    linepointsfile = open(os.path.join(DEFAULT_PARAMS_PATH, "linePoints.txt"), "w")
    for p in pointQueue: linepointsfile.write("%i %i\n" % (p[0], p[1]))
    linepointsfile.close()

    # clean finish
    destroyWindow("Click lines")
    return plines


def interactiveDisplayArea((img,dep), mask):
    pointQueue = []    
    def handleMouseClick(event, x, y, flags, parameters):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            pointQueue.append((x,y))
            #print pointQueue

    # Ask user to click points on depth image
    dep[mask > 0] = 0
    imshow("Click 2 corners", dep*15)
    
    cv2.namedWindow("Click 2 corners", 1)
    cv.SetMouseCallback("Click 2 corners", handleMouseClick, None);    
    print "[interactiveDisplayArea] Press SPACE when finished (ESC to quit):"
    print "[interactiveDisplayArea] Click upper left first, then upper right"
    key = 0
    while (key) != 32:
        key = waitKey(10) % 256
        if key == 27:
            print "Quit program"
            exit(1)
    print "[interactiveDisplayArea] final point queue:", pointQueue                

    rect = (pointQueue[0][0], pointQueue[0][1], pointQueue[1][0] - pointQueue[0][0], pointQueue[1][1]-pointQueue[0][1])
    print "[interactiveDisplayArea] rect:",rect
    print ""

    # Show  for verification
    displayareafile = open(os.path.join(DEFAULT_PARAMS_PATH, "displayAreaRect.txt"), "w")
    displayareafile.write("%i %i %i %i" % rect)

    destroyWindow("Click 2 corners")
    return rect



