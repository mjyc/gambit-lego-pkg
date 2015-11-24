import cv2.cv as cv
import cv2; from cv2 import imshow, imread, imwrite, waitKey
import time
import numpy as np

import util2 as util
from color import *

class FPSCalc:
    def __init__(self):
        self.last = time.time()
        self.numFrames = 0
    
    # needs to be called in 
    def printFPS(self, name):
        if time.time() - self.last >= 1:
            now = time.time()
            print "Average framerate(" + str(name) + "): " + str(1.0 * self.numFrames / (now - self.last)) + " Hz"
            self.last = now
            self.numFrames = 0
        self.numFrames += 1

def downsample(img,dep,factor=2):
    img = img[::factor, ::factor, :]*1
    dep = dep[::factor, ::factor]*1
    return (img,dep)

def objectColor(obj):
    if obj.occluded:
        color = orange
    if obj.touched:
        color = yellow
    if not obj.touched and not obj.occluded:
        color = blue
    if obj.offtable:
        color = gray
    return color

def displayImage(displayArea, img, state, timestep):
    disp = img.copy()
    
    factor = 2;
    for obj in state.objects:
        x0 = obj.rect[0]+obj.rect[2]
        y0 = (obj.rect[1]+obj.rect[3])
        x0 = x0*factor; y0 = y0*factor
        
        if obj.name.find('Hand') < 0 and not obj.offtable:
            if obj.touched:
                s2 = 'Touched'
            elif obj.occluded:
                s2 = 'Occluded'
            else:
                s2 = 'Idle' 
            s = 'Obj %i: %s' % (int(obj.name[6:]), s2)

            x = x0 - 50
            y = y0 + 10
            color = objectColor(obj)
            fontSize = 1
            cv2.putText(disp, s, (x,y), cv2.FONT_HERSHEY_COMPLEX, fontSize, color, thickness=2)
        
        elif obj.name.find('Hand') < 0 and timestep - obj.offtableTime < 15:
            s = 'Obj %i: Being handled' % int(obj.name[6:])
            x = x0 - 50
            y = y0 + 10
            color = red
            fontSize = 1
            cv2.putText(disp, s, (x,y), cv2.FONT_HERSHEY_COMPLEX, fontSize, color, thickness=2)

    area = [i*factor for i in displayArea]
    disp = disp[area[1]:(area[1]+area[3]), area[0]:(area[0]+area[2])]
    imshow('Image display', disp)
    
    
def scaleConvertDepth(depimg):
    low = 500.
    high = 1200.
        
    depimg2 = (depimg - low)
    depimg2 = depimg2 * (255./(high-low))
    depimg2 = np.uint8(depimg2)
    
    return depimg2

def displayDepth(displayArea, dep, state, hands, timestep):

    # converting and shaping display area        
    disp = scaleConvertDepth(dep)
    disp = np.repeat(disp, 3, axis=1)
    disp = np.reshape(disp, (dep.shape[0], dep.shape[1], 3))
    # disp = np.repeat(disp, 2, axis=0)
    # disp = np.repeat(disp, 2, axis=1)
    
    factor = 2;
    liveObjects = [o for o in state.objects if o.name.find('Hand') < 0 and not o.offtable]
    for obj in liveObjects:
        color = objectColor(obj)
        blob = obj.blob
        blob = [(p[0]*factor, p[1]*factor) for p in blob]
        util.drawBlob(disp, blob, color=color)

    for hand in hands:
        color = green
        blob = hand.blob
        blob = [(p[0]*factor, p[1]*factor) for p in blob]
        util.drawBlob(disp, blob, color=color)
    
    area = [i*factor for i in displayArea]
    disp = disp[area[1]:(area[1]+area[3]), area[0]:(area[0]+area[2]), :]
    imshow('Depth display', disp)
