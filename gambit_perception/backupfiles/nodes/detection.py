#!/usr/bin/python

# OpenCV imports
import numpy as np
import cv2.cv as cv
import cv2 as cv2
from cv2 import imshow, imread, imwrite, waitKey

# System imports
from ctypes import *
import sys
import os
import time
import math
import random
import traceback
import cPickle as pickle

# Local imports
import imageprocessing2
imageprocessing = imageprocessing2
import util2 as util
import handtracking
from config import BACKTRACK_LIM, \
                   NOMINAL_MIN_DEPTH, \
                   NOMINAL_MAX_DEPTH, \
                   DISPLAY_FLAG, \
                   DEPTH_NOISE_MARGIN, \
                   OBJ_DEPTH_NOISE_MARGIN, \
                   HAND_SIZE, \
                   HAND_SMOOTHING_PARAM, \
                   WORLD_SNAPSHOT_LIM, \
                   MIN_OBJSIZE_FRAC
import config
from tableobjects2 import TableObject, Hand
import tableobjects2 as tableobjects
import state
from tableevents import *



def initStateTablemodel(flatTable, mask, img, dep, tstep):
    st = state.Tablestate('test', tstep)

    objs, _  =  createNew(img, dep, [], st, flatTable, flatTable, mask, tstep)
    crEvents = [TableEvent(CREATE_EVENT, tstep, obj) for obj in objs]
    tablemodel  = updateTableModel(flatTable, flatTable, st, crEvents, mask, tstep)
    
    st.objects = objs
    return st, tablemodel


TOUCH_EUC_THRESH = 15
MIN_TOUCH_TOGGLE_DURATION = 15

def isTouching(dep, obj, hand, tablemodel, mask, timestep):
    return isOverlapAndTouching(dep, obj, hand, tablemodel, mask, timestep)

def isOverlapAndTouching(dep, obj, hand, tablemodel, mask, timestep):
    # get candidate subsets from obj and hand to reduce computation time
    objMask = obj.fullMask(dep.shape)     
    objMask2 = objMask.copy()
    handMask = hand.fullMask(dep.shape)
    handMask2 = handMask.copy()

    overlapArea = objMask & handMask
    disp = np.uint8(overlapArea)*255
    part1 = cv2.countNonZero(np.uint8(overlapArea)) > 0
    if not part1:
        cv2.putText(disp, "N/A", (0,100), cv2.FONT_HERSHEY_SIMPLEX,1, 255)
        #imshow('isTouching: %s' % obj.name, disp)
        return False
    
    overlapArea = np.uint8(overlapArea)        
    depObjOverlapOnly = obj.fullDepth(dep.shape) * overlapArea
    depHandOverlapOnly = dep * overlapArea
    
    diffs = np.int32(depObjOverlapOnly) - np.int32(depHandOverlapOnly)
    diffs[diffs == 0] = 65000
    mindiff = np.abs(diffs).min()
    
    part2 = mindiff < TOUCH_EUC_THRESH
#    cv2.putText(disp, "%f" % (mindiff), (5,100), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
#    imshow('isTouching', disp)

    part3 = obj.touchScoreBuffer.mean() < TOUCH_EUC_THRESH
    cv2.putText(disp, "%f"% mindiff, (0,100), cv2.FONT_HERSHEY_SIMPLEX,1, 255)
    #imshow('isTouching: %s' % obj.name, disp)
    tmp = obj.touchScoreBuffer.tolist()[1:] + [mindiff,]
    obj.touchScoreBuffer = np.array(tmp, obj.touchScoreBuffer.dtype)


    return part1 and part3


OCC_THRESH = 10
OCC_AREA_FRAC_THRESH = .8
def isOccluded(dep, obj, hand, timestep):
    #return isOccludedLargePatch(dep, obj, timestep)[0]
    return isOccludedSimple(dep, obj, hand, timestep)

def isOccludedSimple(dep, obj, hand, timestep):
    # get candidate subsets from obj and hand to reduce computation time
    objMask = obj.fullMask(dep.shape)     
    objMask2 = objMask.copy()
    handMask = hand.fullMask(dep.shape)
    handMask2 = handMask.copy()

    overlapArea = objMask & handMask
    disp = np.uint8(overlapArea)*255
    #imshow('isOccluded', disp)
    part1 = cv2.countNonZero(np.uint8(overlapArea)) > 0
    
    return part1

# NOT USED SO MUCH
def isOccludedLargePatch(dep, obj, timestep):
    vidArea = dep.shape[0]*dep.shape[1]
    occAreaThresh = .75*MIN_OBJSIZE_FRAC*vidArea
    
    expected = obj.depth
    actual = util.cropImage(dep, obj.rect)

    occMask = actual <    (expected - OCC_THRESH)
    occMask &= obj.mask
    occMask &= actual > 0
    
    #imshow(obj.name+'occMask', np.uint8(occMask)*255)
    
    areaOcc = cv2.countNonZero(np.uint8(occMask)) #len(occMask.nonzero()[0])
    areaObj = cv2.countNonZero(np.uint8(obj.mask)) #len(obj.mask.nonzero()[0])
    fracArea = float(areaOcc)/areaObj
    
    #print 'is occluded:', areaOcc > occAreaThresh, areaOcc
    
    if areaOcc > occAreaThresh:
        #print 'occluded', obj.name
        #waitKey()
        pass
    
    return areaOcc > occAreaThresh, areaOcc, occMask


REMOVAL_DEPTH_THRESH = 10
def isRemoved(dep, obj, timestep):
    vidArea = dep.shape[0]*dep.shape[1]
    removalAreaThresh = .5*MIN_OBJSIZE_FRAC*vidArea
    
    expected = obj.depth*obj.mask # expected depth map of an obejct
    actual = util.cropImage(dep, obj.rect)*obj.mask # a depth map of an actual object
    
    # Look for "moved" amount of pixels
    underMask = np.int32(actual) > np.int32(expected) + REMOVAL_DEPTH_THRESH
    #imshow('under mask', np.uint8(underMask)*255)

    nonzeroMask = (expected > 0) & (actual > 0)
    #imshow('nonzero mask', np.uint8(nonzeroMask) * 255)
    underMask = underMask * np.uint8(nonzeroMask)
    #imshow('under mask over zero', np.uint8(underMask)*255)
    
    areaUnder = cv2.countNonZero(np.uint8(underMask))
    #print 'isRemoved?', areaUnder, '>', removalAreaThresh, '=', areaUnder > removalAreaThresh
    
    retval = areaUnder > removalAreaThresh
    return retval, areaUnder, underMask


def createNew(img, dep, hands, state, flatTable, tablemodel, mask, timestep):
    vidArea = dep.shape[0]*dep.shape[1]
    
    fgMask = imageprocessing.subtractTableSimple(img, dep, tablemodel, mask)
    #imshow('fgmask orig', np.uint8(fgMask)*255)
    fgMask = refineForegroundZeroDepths(fgMask, state.objects, dep, timestep)
    #imshow('fgmask no zerodepth', np.uint8(fgMask)*255)
    
    handMask = handtracking.superHandMask(dep, flatTable, mask)
    fgMask = fgMask & ~handMask
    #imshow('fgmask no hand', np.uint8(fgMask)*255)
    
    fgMask = refineForegroundContained(fgMask, state.objects)
    #imshow('fgmask wo contained', np.uint8(fgMask)*255)
    
    #fgMask2 = np.uint8(fgMask)*255 + np.uint8(handMask)*85
    #imshow('fgmask w hand', fgMask2)

#    imshow('depth, masked', dep*np.uint8(fgMask)*10)
#    tmp = imageprocessing.subtractTableSimple(img, tablemodel, flatTable, mask)
#    imshow('depth, masked without old foreground', tablemodel*np.uint8(~tmp)*10)
    regions = imageprocessing.traceContoursCV2(np.uint8(fgMask))
#    for b in regions:
#         print len(b) > 2, util.blobsize(b) >= MIN_OBJSIZE_FRAC*vidArea, not util.isNegative(b, dep)
    
    # MEAN SIZE FILTERING
    #print MIN_OBJSIZE_FRAC*vidArea
    regions = filter(lambda b: len(b) > 2 and util.blobsize(b, ignoreLessThan=MIN_OBJSIZE_FRAC*vidArea) >= MIN_OBJSIZE_FRAC*vidArea, regions)
    #imshow('obj. candidate regions1', np.uint8(util.regions2mask(regions))*255)
    
    regions = filter(lambda b: not util.isNegative(b, dep), regions)
#    print 'blob sizes', [util.blobsize(b) for b in regions]
    #imshow('obj. candidate regions2', np.uint8(util.regions2mask(regions))*255)

    # istc demo hack
    # BIG ASSUMPTION: that there is only one new thing created
    # BA 2: that it is the last disappeared thing.
    if len(regions) > 1:
        print 'UH OH! More than one new region', regions
    
    # OBJECT RE-ENTERING EVENT DETECTION
    # BREAKS: 1. take off object, another hand appeared, then replace the object => new object.
        # incorporate better LOGIC when handtracking work
    # Simple logic => time two hands separetly then use the lastHandEnterTime from earliest one.
    reObjs = []
    for obj in state.objects:
        # if len(regions) == 1:
        #     print obj.name
        #     print state.lastHandEnterTime
        #     print obj.offtableTime
        
        # ICRA2012 heck - OBEJCT will naver be removed!!!
        #if obj.offtable and state.lastHandEnterTime < obj.offtableTime and len(regions) > 0:
        if obj.offtable and len(regions) > 0:
            reObjs.append(obj)
            region = regions[0]
            obj.initImages(img, dep, util.region2mask(region))
            obj.offtable = False
            regions = []
    
    tobjs = []
    i = 0
    for r in regions:
        objmask = util.region2mask(r)
        t = TableObject(img, dep, objmask, timestep)
        tobjs.append(t)
        i+=1

    #tobjs = [no for no in tobjs if all([not isContainedIn(no, oo) for oo in state.objects])]
    
    return tobjs, reObjs


def detectEvents(img, dep, oldstate, hands, flatTable, tablemodel, mask, timestep):
    events = []

    # Changes to hands
    handInFrameCurr = len([h for h in hands if h != None]) > 0           # numbrt of hands in current time
    handInFrameLast = len([h for h in oldstate.hands if h != None]) > 0  # number of hands in prev time

    # MAKE SURE TO RECORD HAND IN AND OUT EVENT - keeps track of two hands TOGETHER
    if handInFrameCurr and not handInFrameLast:
        ev = TableEvent(HAND_IN_EVENT, timestep, tableobjects.Hand)
        # print 'hand in event'
        events.append(ev)        
    if handInFrameLast and not handInFrameCurr:
        ev = TableEvent(HAND_OUT_EVENT, timestep, tableobjects.Hand)
        # print 'hand out event'
        events.append(ev)

        
    # Changes to objects - if no hands in the scene, there will be NO CHNAGE to objects
    mask0 = handtracking.superHandMask(dep, tablemodel, mask)
    nMask0 = cv2.countNonZero(np.uint8(mask0))
    if nMask0 >= 0:
        for obj in oldstate.objects:
            if obj.offtable: # if object is off the table, don't need to process
                continue;
            
            isr, uarea, umask = isRemoved(dep, obj, timestep)
            
            ist = False # does any hand touching any object right now?
            for hand in hands: ist |= isTouching(dep, obj, hand, tablemodel, mask, timestep)
            
            iso = False # does any hand occluding any object right now?
            for hand in hands: iso |= isOccluded(dep, obj, hand, timestep)
            
            if ist != obj.touched and timestep - obj.touchedToggleTime < MIN_TOUCH_TOGGLE_DURATION:
                ist = obj.touched
             
            if iso and not obj.occluded: # occlusion just happened
                ev = TableEvent(OCCLUDE_EVENT, timestep, obj)
                print 'occlusion event', obj
                events.append(ev) 
    
            if ist and not obj.touched: # touch just happened
                ev = TableEvent(TOUCH_EVENT, timestep, obj)
                print 'touch event', obj
                events.append(ev)
    
            if not iso and obj.occluded:
                ev = TableEvent(UNOCCLUDE_EVENT, timestep, obj)
                print 'un-occlusion event', obj
                events.append(ev) 
                
            if not ist and obj.touched:
                ev = TableEvent(UNTOUCH_EVENT, timestep, obj)
                print 'un-touch event', obj
                events.append(ev)
    
            if isr:
                ev = TableEvent(TAKEAWAY_EVENT, timestep, obj)
                print 'removal event', obj
                events.append(ev)

        # This function is being called ALL THE TIME!
        newObjs, reObjs  =  createNew(img, dep, hands, oldstate, flatTable, tablemodel, mask, timestep)
        
        for obj in newObjs: # New object appeared on the scene
            ev = TableEvent(CREATE_EVENT, timestep, obj)
            print 'created new', obj, cv2.countNonZero(np.uint8(obj.mask))
            events.append(ev)
        
        oldstate.reappeared = reObjs
        
    return events


def isReceivingContainer(obj):
    #if not obj.offtable and (obj.name == 'object0001' or obj.name == 'object0009'):
    return True


def refineForegroundZeroDepths1(fgmask, objects, tmp1, tmp2):
    fgmask2 = fgmask.copy()
    minObjsize = config.videoWidth * config.videoHeight * MIN_OBJSIZE_FRAC
    blobs = imageprocessing.traceContoursCV2(fgmask)
    
    for b in  blobs:
        blobmask = util.region2mask(b)
        #imshow('blobmask', np.uint8(blobmask)*255)
        nAreaBlob = cv2.countNonZero(np.uint8(blobmask))
        
        for obj in objects:
            objZeroMask = obj.fullNoDepth(fgmask.shape)
            #imshow(obj.name+' zero mask', np.uint8(objZeroMask)*255)

            areaInside = blobmask & objZeroMask
            nAreaInside = cv2.countNonZero(np.uint8(areaInside))
            
            if (nAreaBlob - nAreaInside) < minObjsize:
                fgmask2 &= ~blobmask
        
    #imshow('fgmask refined', np.uint8(fgmask2)*255)
    return fgmask2
            

# NOTE: Does not seemed to help much on removing more points
def refineForegroundZeroDepths(fgmask, objects, dep, timestep):
    fgmask2 = fgmask.copy()
    
    for obj in objects:
        isOcc, _, _ = isOccludedLargePatch(dep, obj, timestep) # just use to reduce computation
        # if object is ON the table, and IS NOT occuluded, then create object mask
        if (not obj.offtable) and  (not isOcc):
            # ignore this object's zero depths
            objmask = (~obj.fullMask(fgmask.shape)) | (~obj.fullNoDepth(fgmask.shape))
            #imshow('objmask '+obj.name, np.uint8(objmask)*255)
            fgmask2 &= objmask
            #imshow('fgmask after '+obj.name, np.uint8(fgmask2)*255)
            
    #imshow('fgmask refined', np.uint8(fgmask2)*255)
    return fgmask2
    
    
def isContainedIn(potIngredients, potContainer):
    vidArea = config.videoHeight*config.videoWidth
    MIN_OBJSIZE = vidArea * MIN_OBJSIZE_FRAC

    if isReceivingContainer(potContainer):
        
        areaIngWithinContainer = potIngredients.fullMask(shape=(config.videoHeight,config.videoWidth))\
             * potContainer.fullMask(shape=(config.videoHeight,config.videoWidth))
        
        nAreaIngWithinContainer = cv2.countNonZero(np.uint8(areaIngWithinContainer))
        nAreaIng = cv2.countNonZero(np.uint8(potIngredients.mask))
        
        if nAreaIng - nAreaIngWithinContainer <= MIN_OBJSIZE*.5:
            
            print potIngredients.name, 'is contained in', potContainer.name
            return True
        
    return False

        
def isContainedInB1(blob, potContainer):
    vidArea = config.videoHeight*config.videoWidth
    minObjsize = vidArea * MIN_OBJSIZE_FRAC

    if not potContainer.offtable and isReceivingContainer(potContainer):
        areaIng = util.region2mask(blob)
        nAreaIng = cv2.countNonZero(np.uint8(areaIng))
        areaIngWithinContainer = areaIng & potContainer.fullMask(shape=(config.videoHeight,config.videoWidth))          
        nAreaIngWithinContainer = cv2.countNonZero(np.uint8(areaIngWithinContainer))
        
        if (nAreaIngWithinContainer > minObjsize*.5) and (nAreaIng - nAreaIngWithinContainer <= minObjsize*.5):
            #imshow('areaIng', np.uint8(areaIng)*255)
            #print 'some blob is contained in', potContainer.name
            return True
        
    return False


def isContainedInB(blob, potContainer):
    vidArea = config.videoHeight*config.videoWidth
    minObjsize = vidArea * MIN_OBJSIZE_FRAC

    if not potContainer.offtable:
        areaIng = util.region2mask(blob)
        nAreaIng = cv2.countNonZero(np.uint8(areaIng))
        areaIngWithinContainer = areaIng & potContainer.fullMask(shape=(config.videoHeight,config.videoWidth))          
        nAreaIngWithinContainer = cv2.countNonZero(np.uint8(areaIngWithinContainer))
        
        # if enough overlap and ...not too much lies outside
        #print potContainer.name, nAreaIngWithinContainer, '>', minObjsize*.5
        #print potContainer.name, nAreaIng - nAreaIngWithinContainer, '<=', minObjsize*.5
        if (nAreaIngWithinContainer > minObjsize*.25) and (nAreaIng - nAreaIngWithinContainer <= minObjsize*.75):
            #imshow('areaIng', np.uint8(areaIng)*255)
            #print 'some blob is contained in', potContainer.name
            return True
        
    return False


# Removes fgmask that has blobs which contains other blobs
def refineForegroundContained(fgmask, oldobjects):
    minObjsize = config.videoHeight*config.videoWidth*MIN_OBJSIZE_FRAC # minimum allowed object size
    blobs = imageprocessing.traceContoursCV2(fgmask)
    receiverObjs = [obj for obj in oldobjects if isReceivingContainer(obj)]

    blobsContained = []
    for obj in receiverObjs:
        for b in blobs:
            if len(b) > 2 and util.blobsize(b, ignoreLessThan=minObjsize) >= minObjsize and isContainedInB(b, obj):
                blobsContained.append(b)
                #print '! FOUND contained blob'

    #imshow('fgmask before contained', np.uint8(fgmask)*255) 
    containedMask = util.regions2mask(blobsContained)
    #imshow('containedMask', np.uint8(containedMask)*255)
    fgmask2 = fgmask & ~containedMask
    
    #imshow('fgmask refined contained', np.uint8(fgmask2)*255)
    return fgmask2
        

# Update a tablemodel by removing a object
def cutTableModel(oldtablemodel, flatTable, obj):
    print 'removing', obj.name, 'from table'
    
    i0 = obj.rect[1]
    j0 = obj.rect[0]      
    i1 = obj.rect[1]+obj.rect[3]
    j1 = obj.rect[0]+obj.rect[2]
    
    #imshow('flat table patch', util.scaleImage(flatTable[i0:i1, j0:j1]))
    #imshow('object depth patch', util.scaleImage(obj.depth))
    # the difference between a flatTable and a object depthMap around the object rectangle area.
    correction = flatTable[i0:i1, j0:j1] - obj.depth # resulting output will be positive - object top to bottom
    #imshow('1', util.scaleImage(correction))
    correction = correction * obj.mask * (obj.depth > 0) # obj.mask => binary map, (obj.depth > 0) binary map
    #imshow('2', correction*50)
        
    newtablemodel = oldtablemodel.copy()
    newtablemodel[i0:i1, j0:j1] = oldtablemodel[i0:i1, j0:j1] + correction
    return newtablemodel


MAX_DEPTH = 65535
def augmentTableModel(oldtablemodel, obj, mask):
    newtablemodel = util.raiseZeros(oldtablemodel, MAX_DEPTH, mask)
    
    i0 = obj.rect[1]
    j0 = obj.rect[0]      
    i1 = obj.rect[1]+obj.rect[3]
    j1 = obj.rect[0]+obj.rect[2]
    
    # Get object depthMap
    objDepth = obj.depth * obj.mask
    objDepth = util.raiseZeros(objDepth, MAX_DEPTH)
    
    # Paste an object's depthMap to the tablemodel
    patch = np.minimum(newtablemodel[i0:i1, j0:j1], objDepth)
    newtablemodel[i0:i1, j0:j1] = patch
    newtablemodel = util.lowerToZero(newtablemodel, MAX_DEPTH, mask) # set 0 values back
    #imshow('object depth', obj.depth*10)
    #imshow('patch', patch*10)

    return newtablemodel


def updateTableModel2(oldtablemodel, flatTable, objects, events, mask, timestep):
    newtablemodel = oldtablemodel.copy()
    for ev in events:
        if ev.eventType == TAKEAWAY_EVENT and ev.timestep == timestep:
            newtablemodel = cutTableModel(newtablemodel, flatTable, ev.object)
            #imshow('Table model', util.scaleImage(newtablemodel))
            
    for ev in events:
        if ev.eventType == CREATE_EVENT and ev.timestep == timestep:
            # print ' Adding to table model', ev.object.name
            newtablemodel = augmentTableModel(newtablemodel, ev.object, mask)
            #imshow('Table model', util.scaleImage(newtablemodel))
    
    return newtablemodel


def updateTableModelReappear(oldtablemodel, flatTable, newObjects, events, mask, timestep):
    newtablemodel = oldtablemodel.copy()
    for obj in newObjects:
        print ' Re-adding to table model', obj.name
        newtablemodel = augmentTableModel(newtablemodel, obj, mask)
        #imshow('Table model', util.scaleImage(newtablemodel))
    
    return newtablemodel


def updateTableModel(oldtablemodel, flatTable, state, events, mask, timestep):
    newtablemodel = updateTableModelReappear(oldtablemodel, flatTable, state.reappeared, events, mask, timestep)
    newtablemodel = updateTableModel2(newtablemodel, flatTable, state.objects, events, mask, timestep)
    return newtablemodel



if __name__ == '__main__':
    import cProfile
    #folder = sys.argv[1]
    #cProfile.run('testIsRemoved()', 'profOut')
    testIsRemoved2()
    
