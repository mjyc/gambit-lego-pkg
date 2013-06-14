#! /usr/bin/python

import cv2.cv as cv
import cv2 as cv2
from cv2 import imshow, imread, imwrite, waitKey
from numpy.ctypeslib import ndpointer
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

import util2 as util
import imageprocessing2 as imageprocessing
from config import MIN_OBJSIZE_FRAC
from coord_transform import toPointCloud, scatterPointCloud, transformCloud, toPointXYZ

numObjs = 0
global numObjs


def resetCount():
    global numObjs
    numObjs = 0


touchScoreBufferLen = 5
class TableObject(object):
    def __init__(self, img, dep, mask, timestep, name=None):
        self.touched = False
        self.occluded = False
        self.offtable = False
        
        self.initImages(img, dep, mask)
        
        # automatically assign name
        if name:
            self.name = name
        else:
            global numObjs
            self.name = 'object%04i' % numObjs
            numObjs+=1
        
        self.timeStatusChange = timestep
        self.creationTime = timestep
        self.offtableTime = -1
        self.touchedToggleTime = -1
        self.occludedToggleTime = -1
        self.touchScoreBuffer = 65000*np.ones((touchScoreBufferLen,), 'float32')
        
    def initImages(self, img, dep, mask):
        # extract bounding rect from mask
        inds_i, inds_j = mask.nonzero()
        rect_y = inds_i.min()
        rect_x = inds_j.min()
        wid = inds_j.max() - inds_j.min()
        ht = inds_i.max() - inds_i.min()
        rect_y1 = inds_i.max()
        rect_x1 = inds_j.max()
        # rect is in (x, y, wid, ht) format
        self.rect = (rect_x, rect_y, wid, ht)
        #img2 = img.copy()
        #cv2.rectangle(img2, (self.rect[0], self.rect[1]), (self.rect[0]+self.rect[2], self.rect[1]+self.rect[3]), (255,0,0))
        #imshow('location', img2)
        #waitKey()


        # Save rotated rectangle information
        contours, trash = cv2.findContours(np.uint8(mask), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        rect = cv2.minAreaRect(cnt)
        self.rotRect = rect

        # Save PointCloud information
        if not (img.shape[0] == 10 and img.shape[1] == 10):
            Tcw = np.genfromtxt("params/Tcw.txt")
            cnt2 = []
            for p in cnt:
                cnt2.append((p[0][0], p[0][1]))
            rotRectMask = util.region2mask(cnt2)
            cloud = toPointCloud(img, dep*rotRectMask) # depth channel work as a mask
            newCloud = transformCloud(cloud,Tcw)
            # compute centroid and peak
            centroid = np.mean(newCloud[:,0:3],0)[0:3]
            peak = newCloud[newCloud[:,2].argmax(),0:3]

            # attemp to coordinate transform bounding box
            box = cv2.cv.BoxPoints(rect)
            pts = np.int16(box)
            
            tempDep = dep[rect_y:rect_y1, rect_x:rect_x1]
            avg =  np.mean(tempDep)
            #print np.mean(tempDep)

            pts2 = [toPointXYZ(pt[0],pt[1],avg,240,320) for pt in pts]
            pts2 = np.hstack((np.array(pts2), np.ones((4,1))))
            #print pts
            #print pts2
            boundingbox = np.dot(Tcw,np.array(pts2).T).T
            print boundingbox

            x = boundingbox[1,0] - boundingbox[0,0]
            y = boundingbox[1,1] - boundingbox[0,1]
            h = np.sqrt(x*x + y*y)
            print np.sin(1.0*y/h)
            print centroid
            print np.sin(1.0*y/h) - 3.14 + 1.57, centroid[0], centroid[1], 0.071 # take 1.57 out for perpendicular grasp
            
            # fig = plt.figure()
            # axis = fig.add_subplot(111, projection='3d')
            # scatterPointCloud(axis,newCloud,1)
            # axis.scatter(centroid[0],centroid[1],centroid[2],c='r')
            # axis.scatter(peak[0],peak[1],peak[2],c='r')
            
            # axis.scatter(boundingbox[0,0],boundingbox[0,1],centroid[2],c='r')
            # axis.scatter(boundingbox[1,0],boundingbox[1,1],centroid[2],c='r')
            # axis.scatter(boundingbox[2,0],boundingbox[2,1],centroid[2],c='r')
            # axis.scatter(boundingbox[3,0],boundingbox[3,1],centroid[2],c='r')
            # axis.scatter(boundingbox[0,0],boundingbox[0,1],0,c='r')
            # axis.scatter(boundingbox[1,0],boundingbox[1,1],0,c='r')
            # axis.scatter(boundingbox[2,0],boundingbox[2,1],0,c='r')
            # axis.scatter(boundingbox[3,0],boundingbox[3,1],0,c='r')
            # plt.show()
        
        
        DEBUG = True
        if DEBUG: # Display box
            im = img.copy()
            box = cv2.cv.BoxPoints(rect)
            pts = np.int16(box)
            for j in range(len(pts)):
                cv2.line( im, tuple(pts[j]), tuple(pts[(j+1)%4]), (0,0,255),2)
            imshow("boundingbox", im)

        self.mask = mask[rect_y:rect_y1, rect_x:rect_x1]
        self.depth = dep[rect_y:rect_y1, rect_x:rect_x1]
        self.color = img[rect_y:rect_y1, rect_x:rect_x1, :]
        self.blob = None
        blobs = util.traceContoursCV2(mask)
        if len(blobs) > 0:
            self.blob = blobs[0]

        #imshow('dep'+self.name, self.depth*15)
        #imshow('col'+self.name, self.color)

        self.noDepth = (self.depth <= 0) * self.mask        
        
        
    def isIdle(self):
        return (not self.touched) and (not self.occluded) and (not self.offtable)
    
    
    def status(self):
        status = ('Touched ' if self.touched else '') \
               + ('Occluded ' if self.occluded else '') \
               + ('Offtable ' if self.offtable else '') \
               + ('Idle ' if self.isIdle() else '')
        status = status.rstrip()
        return status
    
    
    def __str__(self):
        s = '(' 
        s += self.name 
        s += ' ' + self.status()
        s += ')'
        return s
    
    def fullMask(self, shape=(480,640)):
        mask = np.zeros(shape, 'bool8')
        mask[self.rect[1]:self.rect[1]+self.rect[3], self.rect[0]:self.rect[0]+self.rect[2]] = self.mask
        return mask

    def fullDepth(self, shape=(480,640)):
        dep = np.zeros(shape, 'uint16')
        dep[self.rect[1]:self.rect[1]+self.rect[3], self.rect[0]:self.rect[0]+self.rect[2]] = self.depth
        return dep

    def fullNoDepth(self,shape=(480,640)):
        mask = np.zeros(shape, 'bool8')
        mask[self.rect[1]:self.rect[1]+self.rect[3], self.rect[0]:self.rect[0]+self.rect[2]] = self.noDepth
        return mask

    def center(self):
        return (self.rect[0]+self.rect[2]/2, self.rect[1]+self.rect[3]/2)

            
    
def makeObjects(img, dep, tablemodel, mask, tstep=1):
    vidArea = dep.shape[0]*dep.shape[1]
    
    fgMask = imageprocessing.subtractTable(img, dep, tablemodel, mask)
#     imshow('fgmask', fgMask)
    
    regions = imageprocessing.traceContoursCV2(fgMask)
    #for b in regions:
    #     print len(b) > 2, util.blobsize(b) >= MIN_OBJSIZE_FRAC*vidArea, not util.isNegative(b, dep)
    regions = filter(lambda b: len(b) > 2 and util.blobsize(b, ignoreLessThan=MIN_OBJSIZE_FRAC*vidArea) >= MIN_OBJSIZE_FRAC*vidArea and not util.isNegative(b, dep), regions)
    
    tobjs = []
    i = 0
    for r in regions:
        objmask = util.region2mask(r)
        t = TableObject(img, dep, objmask, tstep)
        tobjs.append(t)
        i+=1
    
    return tobjs

def makeObject(img, dep, mask):
    t = TableObject(img, dep, mask, 1)
    return t


Hand = TableObject(np.zeros((10,10,3), 'uint8'), np.zeros((10,10), 'uint16'), np.ones((10,10), 'bool8'), 1, name='Hand')
LeftHand = TableObject(np.zeros((10,10,3), 'uint8'), np.zeros((10,10), 'uint16'), np.ones((10,10), 'bool8'), 1, name='Left Hand')

if __name__ == '__main__':
    img = imread('/home/jinna/kitchen_data/dataCake/cake3/image_1000_rgb.png', -1)
    dep = imread('/home/jinna/kitchen_data/dataCake/cake3/image_1000_dep.png', -1)
    tablemodel = util.buildMinMap('/home/jinna/kitchen_data/dataCake/cake3/table')
    mask = imread('/home/jinna/kitchen_data/dataCake/cake3/mask.png', -1)
    mask = ~mask
    
    objs = makeObjects(img, dep, tablemodel, mask)
    
   
    
