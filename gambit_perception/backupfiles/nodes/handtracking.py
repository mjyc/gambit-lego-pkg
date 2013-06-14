#! /usr/bin/python

# OpenCV imports
from numpy.ctypeslib import ndpointer
import numpy as np
import cv2.cv as cv
import cv2 as cv2
from cv2 import imshow, imread, imwrite, waitKey
from ctypes import *
from imageprocessing2 import traceContoursCV2 as traceContours
#import matplotlib.pyplot as plt

# Local
import util2 as util

# System imports
import sys
import os
import time
import math
import random
import traceback
from config import MIN_HANDSIZE_FRAC
import pdb

NCLUSTERS = 5



def getVectorList(handCalibrationPath):
    lst = os.listdir(handCalibrationPath)
    lst = filter(lambda f: f.find('rgb.png') > -1, lst)
    lst.sort()

    lim = 1e7
    vecs = np.zeros((lim, 3), 'uint8')    
    kk = 0
    for f in lst:
        g = os.path.join(handCalibrationPath, f)
        #print g
        im = imread(g)
        #imshow('calibration image', im)
        #waitKey(10)


        # extract the colors
        ii, jj, trash = im.nonzero()
        mv = cv2.split(im)
        r,g,b = mv
        rs = r[ii, jj]
        gs = g[ii, jj]
        bs = b[ii, jj]
        
        kk2 = kk + len(rs)                
        if len(rs) > 0 and kk2 < lim:
            vecs[kk:kk2, 0] = rs
            vecs[kk:kk2, 1] = gs
            vecs[kk:kk2, 2] = bs
            
        kk += len(rs)
        
    vecs = vecs[:kk]
    print len(vecs)
    return np.float32(vecs)
        


def getVectorListHSV(handCalibrationPath):
    lst = os.listdir(handCalibrationPath)
    lst = filter(lambda f: f.find('rgb.png') > -1, lst)
    lst.sort()

    lim = 1e7
    vecs = np.zeros((lim, 3), 'uint8')    
    kk = 0
    for f in lst:
        g = os.path.join(handCalibrationPath, f)
        #print g
        im = imread(g)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        #imshow('calibration image', im)
        

        # extract the colors
        ii, jj, trash = im.nonzero()
        mv = cv2.split(im)
        r,g,b = mv
        rs = r[ii, jj]
        gs = g[ii, jj]
        bs = b[ii, jj]
        
        kk2 = kk + len(rs)                
        if len(rs) > 0 and kk2 < lim:
            vecs[kk:kk2, 0] = rs
            vecs[kk:kk2, 1] = gs
            vecs[kk:kk2, 2] = bs
            
        kk += len(rs)
        
    vecs = vecs[:kk]
    print len(vecs)
    return np.float32(vecs)


def getIndepHistsHSV(handCalibrationPath):
    lst = os.listdir(handCalibrationPath)
    lst = filter(lambda f: f.find('rgb.png') > -1, lst)
    lst.sort()

    lim = 1e7
    histr = np.zeros((256,), 'int32')
    histg = np.zeros((256,), 'int32')
    histb = np.zeros((256,), 'int32')
    
    for f in lst:
        g = os.path.join(handCalibrationPath, f)
        #print g
        im = imread(g)
        #imshow('calibration image', im)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        

        # extract the colors
        ii, jj, trash = im.nonzero()
        mv = cv2.split(im)
        r,g,b = mv
        rs = r[ii, jj]
        gs = g[ii, jj]
        bs = b[ii, jj]

        histr[rs] = histr[rs] + 1
        histg[gs] = histg[gs] + 1
        histb[bs] = histb[bs] + 1

    return histr, histg, histb


# can't seem to set the params via Python.
# Using 10 clusters, which is the default
def buildGMM(handCalibrationPath, save=''):
    print handCalibrationPath
    samples = getVectorList(handCalibrationPath)
    
    gmmlib = cdll.LoadLibrary('./libgmmwrapper.so')
    gmmlib.train.argtypes = [ndpointer(c_float), ]
    gmmlib.predict.argtypes = [ndpointer(c_uint8), ndpointer(c_float)]
    
    gmmlib.init(c_int(NCLUSTERS))
    gmmlib.train(samples, c_int(samples.shape[0]))

    if save:
        gmmlib.save(c_char_p(save))
    
    return gmmlib

    
def getGMMProbs(im, gmm):
    imVec = im.ravel('F').reshape(im.shape[0]*im.shape[1], 3)
    print imVec
    probsVec = np.zeros(imVec.shape, 'float32')

    gmm.predict(imVec, probsVec)

    probs = probsVec.reshape(im.shape).ravel('F').reshape(im.shape)
    imshow('probs', probs)
    return probs

    
def getHandmask(im, dep, tablemodel, **kwargs):
    return getHandmaskHSV(im, dep, tablemodel)


def getColorMasks4(im, bmean=116, bsig=34, gmean=153, gsig=39, rmean=190, rsig=32):
    b = im[:,:,0]*1
    g = im[:,:,1]*1
    r = im[:,:,2]*1
    
    bsig=float(bsig)
    gsig=float(gsig)
    rsig=float(rsig)
    
    rlogp = -.5*((r-rmean)/rsig )**2 + np.log(1./(rsig*np.sqrt(2*np.pi)))
    print rlogp.min(), rlogp.max()
    imshow('rlog', np.uint8(rlogp - rlogp.min())*2)
    
    glogp = -.5*((g-gmean)/gsig )**2 + np.log(1./(gsig*np.sqrt(2*np.pi)))
    print glogp.min(), glogp.max()
    imshow('glog', np.uint8(glogp-glogp.min())*2)
    
    blogp = -.5*((b-bmean)/bsig )**2 + np.log(1./(bsig*np.sqrt(2*np.pi)))
    print blogp.min(), blogp.max()
    imshow('blog', np.uint8(blogp - blogp.min())*2)

    weirdlogp = rlogp + glogp + blogp
    print weirdlogp.min(), weirdlogp.max()
        
    disp = np.uint8(weirdlogp-weirdlogp.min())
    print disp.min(), disp.max()
    imshow('weirdlog', disp)
    


def testGetColorMasks():
    folder = sys.argv[1]
    handCalibrationPath = os.path.join(folder, 'hand-video')
    #getVectorList(handCalibrationPath)
    #gmm = buildGMM(handCalibrationPath, save='gmm')
    tablePath = os.path.join(folder, 'table')
    tablemodel = imread(os.path.join(tablePath, 'image_10_dep.png'), -1);
    maskPath = os.path.join(folder, 'mask.png')
    print maskPath
    mask = imread(maskPath, -1)
    mask = ~mask
    #imshow('tablemodel', tablemodel*10)
    #waitKey()
    
    
    lst = os.listdir(folder)
    lst = filter(lambda f:f.find('rgb.png') > -1 and f.find('_0_') < 0, lst)
    lst.sort(key=lambda f:int(f.split('_')[1]))

    for f in lst:
        print os.path.join(folder, f)
        testim = imread(os.path.join(folder, f))
        getColorMasks(testim)


def getColorMasks3(im, hueTarget=20, hueDelta1=20, hueDelta2=10, \
                  satTarget=100, satDelta1=60, satDelta2=30, \
                  valTarget=190, valDelta1=60, valDelta2=30):

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    hue = hsv[:,:,0]*1
    sat = hsv[:,:,1]*1
    val = hsv[:,:,2]*1
    
    def threshold(arr, target, delta):
        mask = (arr > (target-delta)) & (arr < (target+delta))
        return mask
    
    def circThreshold(arr, target, delta):
        diff1 = np.abs(np.int32(arr) - target)
        diff2 = np.abs(np.int32(arr) - 180 - target)
        diff = np.minimum(diff1, diff2)
        mask = diff < delta
        return mask
    
    #imshow('hsv', hsv)
    #imshow('hue', np.uint8(hue)*1)
    #imshow('sat', np.uint8(sat)*1)
    #imshow('val', np.uint8(val)*1)
    #waitKey()
    
    valMaskLow = threshold(val, valTarget, valDelta1)
    #imshow('value mask', np.uint8(valMask)*255)
    valMaskHigh = threshold(val, valTarget, valDelta2)
    
    satMaskLow = threshold(sat, satTarget, satDelta1)
    #imshow('sat mask low', np.uint8(satMaskLow)*255)
    
    satMaskHigh = threshold(sat, satTarget, satDelta2)
    #imshow('sat mask high', np.uint8(satMaskHigh)*255)
    
    hueMaskLow = circThreshold(hue, hueTarget, hueDelta1)
    #imshow('hue mask low', np.uint8(hueMaskLow)*255)

    hueMaskHigh = circThreshold(hue, hueTarget, hueDelta2)
    #imshow('hue mask high', np.uint8(hueMaskHigh)*255)
        
    maskLow = hueMaskLow & satMaskLow & valMaskLow
    #imshow('mask low', np.uint8(maskLow)*255)
    
    maskHigh = hueMaskHigh & satMaskHigh & valMaskHigh
    #imshow('mask high', np.uint8(maskHigh)*255)
    
    return maskLow, maskHigh



HUE_TARGET = 0
HUE_THRESH = 20
VAL_THRESH = 50
#SAT_THRESH_LOW = 0
SAT_THRESH_LOW = 50
SAT_THRESH_HIGH = 120
#SAT_THRESH_HIGH = 150

def getColorMasks(im):

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    hue = hsv[:,:,0]*1
    sat = hsv[:,:,1]*1
    val = hsv[:,:,2]*1
    
    # imshow('hsv', hsv)
    # imshow('hue', np.uint8(hue)*1)
    # imshow('sat', np.uint8(sat)*1)
    # imshow('val', np.uint8(val)*1)
    # waitKey()
    # cv2.destroyAllWindows()
    
    valMask = val > VAL_THRESH
    #imshow('value mask', np.uint8(valMask)*255)
    
    satMaskLow = sat > SAT_THRESH_LOW
    #imshow('sat mask low', np.uint8(satMaskLow)*255)
    
    satMaskHigh = sat > SAT_THRESH_HIGH
    #imshow('sat mask high', np.uint8(satMaskHigh)*255)
    #waitKey()
    #cv2.destroyAllWindows()
    
    hueDiff1 = np.abs(np.int32(hue) - HUE_TARGET)
    # imshow('huediff1', hueDiff1*255)
    
    hueDiff2 = np.abs((np.int32(hue) - 180) - HUE_TARGET)
    # imshow('huediff2', hueDiff2*255)
    
    hueDiff = np.minimum(hueDiff1, hueDiff2)
    # imshow('huediff', hueDiff*255)
    # waitKey()
    # cv2.destroyAllWindows()

    hueMask = hueDiff < HUE_THRESH
    # imshow('hue thresh', np.uint8(hueMask)*255)
    # waitKey()
    # cv2.destroyAllWindows()
    
    maskLow = valMask & satMaskLow & hueMask
    # imshow('mask low', np.uint8(maskLow)*255)
    maskHigh = valMask & satMaskHigh & hueMask
    # imshow('mask high', np.uint8(maskHigh)*255)
    # waitKey()
    # cv2.destroyAllWindows()
    
    return maskLow, maskHigh
    
    
def getHandmask(im, dep, tablemodel, mask):
    return getHandmaskHSV(im, dep, tablemodel, mask)


TABLE_NOISE_THRESH = 15
def getHandmaskHSV(im, dep, tablemodel, mask):

    DEBUG = False

    vidArea = dep.shape[0]*dep.shape[1]
    
    # get colormasks
    colormaskLow, colormaskHigh = getColorMasks(im)
    if DEBUG:
        imshow('colormaskLow', np.uint8(colormaskLow)*255)
        imshow('colormaskHigh', np.uint8(colormaskHigh)*255)

    #colormaskLow = np.bool8(np.ones(dep.shape))
    #colormaskHigh = np.bool8(np.ones(dep.shape))
    
    # get depth foreground seg...
    depthmask = (np.int32(tablemodel) - np.int32(dep) > TABLE_NOISE_THRESH)
    #imshow('above table', np.uint8(depthmask)*255)

    # table masking
    #depthmask = depthmask & mask
    #imshow('above table', np.uint8(depthmask)*255)
    
    # filter depthmask regions for ones that cross the mask
    depthmask2 = filterRegions(depthmask, mask)
    
    # get candidate regions from depth and handmaskLow
    regions = traceContours(depthmask2 & colormaskLow & mask)
    #regions = traceContours(depthmask2)
    
    # do a standard filtering for size and negative regions
    regions2 = [b for b in regions if len(b) > 10 and util.blobsize(b, ignoreLessThan=MIN_HANDSIZE_FRAC*vidArea) >= MIN_HANDSIZE_FRAC*vidArea and not util.isNegative(b, dep)]
    regions = regions2

    #imshow('hand region', np.uint8(util.regions2mask(regions))*255)
    
    # # filter regions by presence of high match pixels
    handmask = np.zeros(dep.shape, 'bool')
    for b in regions:
        regionmask = util.region2mask(b)
        #imshow('region', np.uint8(regionmask)*255)
        tmp = regionmask & colormaskHigh
        
        if cv2.countNonZero(np.uint8(tmp)) > 0:
        #if len(tmp.nonzero()) > 0:
            #print 'TRUE'
            handmask = handmask | regionmask
    
    #imshow('handmask', np.uint8(handmask)*255)
    return handmask


def superHandMask(dep, tablemodel, mask, dispName=''):
    depthmask = (np.int32(tablemodel) - np.int32(dep) > TABLE_NOISE_THRESH)
    #imshow('superHandMask tablemodel '+dispName, tablemodel*15)
    #imshow('above table', np.uint8(depthmask)*255)
    
    # filter depthmask regions for ones that cross the mask
    depthmask2 = filterRegions(depthmask, mask)
    #imshow('super hand mask '+dispName, np.uint8(depthmask2)*255)
    return depthmask2


def filterRegions(regionsMask, mask):
    vidArea = mask.shape[0]*mask.shape[1]
    
    regions = traceContours(regionsMask)
    regions = filter(lambda b: len(b) > 2 and util.blobsize(b, ignoreLessThan=MIN_HANDSIZE_FRAC*vidArea) >= MIN_HANDSIZE_FRAC*vidArea, regions)
    #imshow('after size filtering', np.uint8(util.regions2mask(regions)*255))
    #imshow('mask',mask)
    regions = filter(lambda b: crossesMask(b,mask), regions)
    #imshow('after cross filtering', np.uint8(util.regions2mask(regions)*255))

    regionsMask2 = np.zeros(regionsMask.shape, 'uint8')
    cv2.fillPoly(regionsMask2, [np.array(blob, dtype="int32") for blob in regions], 1)
    regionsMask2 = np.bool8(regionsMask2)
    
    #imshow('above table 2', np.uint8(regionsMask2)*255)

    return regionsMask2


def crossesMask(blob, mask):
    # get a slightly eroded mask
    #mask = cv2.erode(np.uint8(mask), None)
    mask = np.bool8(mask)
        
    mask1 = util.region2mask(blob)
    area0 = cv2.countNonZero(np.uint8(mask1)) #len(mask1.nonzero()[0]) # number of nonzero pixels in the original region
    #print area0
    
    # check if masking cut off the blob somehow
    # check if the masked area is smaller than the original area
    mask2 = mask1 & mask
    #imshow('blob mask', np.uint8(mask1)*255)
    #imshow('mask subtracted', np.uint8(mask2)*255)
    
    area1 = cv2.countNonZero(np.uint8(mask2)) #len(mask2.nonzero()[0]) 
    #print 'area1', area1
    #print 'area0', area0
    #waitKey()
    
    if area1 < area0 and area1 > 0:
        return True
    
    return False

def findHands(im, dep, tablemodel, plines, mask, truncate=False):
    mask0 = superHandMask(dep, tablemodel, mask)
    nMask0 = cv2.countNonZero(np.uint8(mask0))
    if nMask0 <= 0:
        return []
    
    handmask = getHandmask(im, dep, tablemodel, mask)
    
    regions = traceContours(handmask)
    res = []

    if truncate:
        regions2 = [truncateHand(regions[i], plines, np.zeros(im.shape, 'uint8'), id=i) for i in range(len(regions))]    
        regions = regions2
        
    for b,i in zip(regions, range(len(regions))):
        blobMask = util.region2mask(b)
        #imshow('sub hand', np.uint8(blobMask)*255)
        dep2 = dep.copy()
        dep2 = dep2 & (np.uint16(blobMask)*(0xffff)) 
        
        inds_i, inds_j = blobMask.nonzero()
        pi = np.mean(inds_i)
        pj = np.mean(inds_j)
        pz = np.average(dep2, weights = np.uint8(blobMask))
        
        h = Hand(pi, pj, pz, dep, blobMask)
        res.append(h)

    # if len(res) == 2:
    #imshow("handtracking", np.uint8(handmask)*255)
        
    return res


def truncateHand(handblob, plines, disp, id=0):
    disp = disp.copy()
    for p1, p2 in zip(handblob[:-1], handblob[1:]):
       cv2.line(disp, p1, p2, (125,125,125))
    
    # get from table mask
    #tablemask = cv2.erode(np.uint8(tablemask), None)
    ##imshow('eroded tablemask', np.uint8(tablemask)*(255/tablemask.max()))
    #print 'maskblob', maskblob
    #print 'lines xy', lines
    if disp != None:
        for l in plines:
            cv2.line(disp, l[0], l[1], (255,255,255))
    lines = [util.convertXYtoMB(l[0], l[1]) for l in plines]        
    #print 'lines mb', lines
    
#    #imshow('truncateHand', disp)
#    waitKey(1)
    
    # get relevant points
    crossPts, line = util.getCrossPoints(lines, handblob, returnLine=True)
    pline = [plines[i] for i in range(len(lines)) if lines[i] == line]
    pline = plines[0]
    print 'cross line', line, pline
    cv2.line(disp, pline[0], pline[1], (255,255,0))
#    #print 'cross points', crossPts
#    
#    for pt in crossPts:
#        cv2.circle(disp, pt, 3, (255,255,0))
#    #imshow('truncateHand', disp)
#    waitKey(1)
   
    
    #print np.array(crossPts, 'float32')
#    basept = np.array(crossPts, 'float32').mean(axis=0)
    basept = np.array(handblob, 'float32').mean(axis=0)
    #print 'base point', basept

    cv2.circle(disp, (basept[0], basept[1]), 3, (255,0,0))
#    #imshow('truncateHand', disp)
#    waitKey(1)
   
    
    farpt = max(handblob, key=lambda pt: util.distanceFromLinePt(pt, pline))
    #print 'far point', farpt
    cv2.circle(disp, farpt, 3, (0,255,255))
#    #imshow('truncateHand', disp)
#    waitKey(1)

    # get intersection pts
    from config import HAND_LEN_PX as handLenPx
    v = util.normalize(farpt - basept)
    wristPt = np.array(farpt,'float32') - handLenPx*np.array(v,'float32')
    #print 'wrist point', wristPt
    
    cv2.circle(disp, (int(wristPt[0]), int(wristPt[1])),  3, (0,255,0))
#    #imshow('truncateHand', disp)
#    waitKey(1)

    perpVect = np.array((-v[1], v[0]), 'float32')
    cv2.line(disp, (int(wristPt[0]), int(wristPt[1])),(int(wristPt[0]+perpVect[0]), int(wristPt[1]+perpVect[1])), (0,0,255))
    
    truncatePts, blobIdxs = util.getCrossPoints([util.convertXYtoMB(wristPt, perpVect+wristPt),], handblob, returnIdxs=True)
    print 'blob inds', blobIdxs
    print 'truncate points', truncatePts
    for pt in truncatePts:
        cv2.circle(disp, pt, 3, (0,0,255))
#    #imshow('truncateHand', disp)
#    waitKey(1)

    if 0:
        #imshow('truncateHand', disp)
        print 'Wrong number of truncation points:', truncatePts
        waitKey(0)
        wrong_number_of_points

    if len(truncatePts) >= 2:   
        oldblob = handblob
        newblob = oldblob[blobIdxs[0]:blobIdxs[-1]]
    else:
        oldblob = handblob
        newblob = handblob
    
    for p1, p2 in zip(newblob[:-1], newblob[1:]):
        cv2.line(disp, p1, p2, (255,0,255))
    #imshow('truncateHand' + str(id), disp)
#    waitKey(1)
    
    return newblob

class Hand(object):
    def __init__(self, pi, pj, centerDepth, dep, mask):
        self.Ei = pi
        self.Ej = pj
        self.Ez = centerDepth

        self.pi = int(pi)
        self.pj = int(pj)

        # get cropped mask
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
        
        self.mask = mask[rect_y:rect_y1, rect_x:rect_x1]
        self.depth = dep[rect_y:rect_y1, rect_x:rect_x1]
        
        self.blob = None
        blobs = util.traceContoursCV2(mask)
        if len(blobs) > 0:
            self.blob = blobs[0]
            

    def fullMask(self, shape=(480,640)):
        mask = np.zeros(shape, 'uint8')
        mask[self.rect[1]:self.rect[1]+self.rect[3], self.rect[0]:self.rect[0]+self.rect[2]] = self.mask
        return mask
        
        
    
    
def histogram(folder):
    lst = getVectorList(folder)
    h = lst[:,0]
    s = lst[:,1]
    v = lst[:,2]
    
    plt.figure()
    bins = np.arange(0, 180, 5)
    plt.hist(h, bins=bins)
    print 'h orig mean', h.mean(), 'h orig std', h.std()
    
    h2 = (h - h.mean() + 90) % 180
    #plt.hist(h2, bins=bins)
    #print 'h mean', (h.mean() + (h2.mean()-90) )% 180, 'h new mean', h2.mean(), 'h std', h2.std()
    #hmu = (h.mean() + (h2.mean()-90) )% 180
    
    plt.figure()
    bins = np.arange(0, 256, 5)
    plt.hist(s, bins=bins)
    print 's mean', s.mean(), 's std', s.std()

    plt.figure()
    bins = np.arange(0, 256, 5)
    plt.hist(v, bins=bins)
    print 'v mean', v.mean(), 'v std', v.std()

    plt.show()

    #return ((hmu, h2.std()), (s.mean(), s.std()), (v.mean(), v.std()))
    return ((h.mean(), h.std()), (s.mean(), s.std()), (v.mean(), v.std()))


def getParamsHSV(folder):
    (hp, sp, vp) = histogram(folder)
    
    hmu = hp[0]
    hueLowDelta = .5*hp[1]
    hueHighDelta = 1*hp[1]
    
    smu = sp[0]
    satLowDelta = .5*sp[1]
    satHighDelta = 1*sp[1]
    
    vmu = vp[0]
    valLowDelta = .5*vp[1]
    valHighDelta = 1*vp[1]
    
    return ((hmu, hueLowDelta, hueHighDelta), (smu, satLowDelta, satHighDelta), (vmu, valLowDelta, valHighDelta))

    


def testHistogram():
    folder = sys.argv[1]
    handCalibrationPath = os.path.join(folder, 'hand-video')
    histogram(handCalibrationPath)
    

def testAll(folder):
    handCalibrationPath = os.path.join(folder, 'hand-video')
    #getVectorList(handCalibrationPath)
    #gmm = buildGMM(handCalibrationPath, save='gmm')
    tablePath = os.path.join(folder, 'table')
    tablemodel = imread(os.path.join(tablePath, 'image_10_dep.png'), -1);
    maskPath = os.path.join(folder, 'mask.png')
    print maskPath
    mask = imread(maskPath, -1)
    mask = ~mask
    maskblob = util.traceContoursCV2(tablemask)
    maskblob = maskblob[0]
    plines = zip(maskblob[1:], maskblob[:-1])
    #imshow('tablemodel', tablemodel*10)
    #waitKey()
    
    
    lst = os.listdir(folder)
    lst = filter(lambda f:f.find('rgb.png') > -1 and f.find('_0_') < 0, lst)
    lst.sort(key=lambda f:int(f.split('_')[1]))

    for f in lst:
        print os.path.join(folder, f)
        testim = imread(os.path.join(folder, f))
        deppath = os.path.join(folder, f[:-7]+'dep.png')
        print deppath
        testdep = imread(deppath,-1)
        #imshow('testdep', testdep*10)

        hands = findHands(testim, testdep, tablemodel, plines, mask)
        for h in hands:
            print (h.pj, h.pi), h.Ez, int(10000./h.Ez)
            cv2.circle(testim, (h.pj, h.pi), int(10000./h.Ez), (255,0,0))
        imshow('testim', testim)
        
        if hands:
            waitKey(0)
            
        waitKey(10)
       

def writeMasks(folder):
    handCalibrationPath = os.path.join(folder, 'hand-video')
    #getVectorList(handCalibrationPath)
    #gmm = buildGMM(handCalibrationPath, save='gmm')
    tablePath = os.path.join(folder, 'table')
    tablemodel = imread(os.path.join(tablePath, 'image_10_dep.png'), -1);
    maskPath = os.path.join(folder, 'mask.png')
    print maskPath
    mask = imread(maskPath, -1)
    mask = ~mask
    #imshow('tablemodel', tablemodel*10)
    #waitKey()
    
    
    lst = os.listdir(folder)
    lst = filter(lambda f:f.find('rgb.png') > -1 and f.find('_0_') < 0, lst)
    lst.sort(key=lambda f:int(f.split('_')[1]))

    for f in lst:
        print os.path.join(folder, f)
        testim = imread(os.path.join(folder, f))
        deppath = os.path.join(folder, f[:-7]+'dep.png')
        print deppath
        testdep = imread(deppath,-1)
        #imshow('testdep', testdep*10)
	num = int(f.split('_')[1])
	print num

        handmask = getHandmask(testim, testdep, tablemodel, mask)
	imwrite('output/handmasks/handmask%04i.png' % num, 255*np.uint8(handmask))
	waitKey(10)
        
if __name__ == '__main__':
    import config
    config.videoHeight = 480
    config.videoWidth=640
    folder = sys.argv[1]
    #testAll(folder)
    writeMasks(folder)


