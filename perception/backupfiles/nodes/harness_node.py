#!/usr/bin/env python

# ROS Modules
import roslib; roslib.load_manifest("gambit_perception")
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import message_filters

# System Modules
import cv2.cv as cv
import cv2; from cv2 import imshow, imread, imwrite, waitKey
import time
import copy
import os
import pdb
import numpy as np
from argparse import ArgumentParser

# Local modules
import imageprocessing2
import util2 as util
import detection
from tableevents import *
import tableobjects2 as tableobjects; from tableobjects2 import Hand
import state; from state import Tablestate
import handtracking
import util3
from util3 import FPSCalc, objectColor, downsample
from color import *

default_params_path = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params_py";

#--------------------------------------#
#---- Global variables & functions ----#
#--------------------------------------#
def outlineObjects(depImg, objs):
    depImg2 = depImg.copy()
    
    for obj in objs:
        if not obj.offtable:
            b = imageprocessing2.traceContoursCV2(obj.fullMask())
            b = b[0]
            c = depImg.max()
            util.drawBlob(depImg2, b, color=int(c), thick=1)
        
    return depImg2  


def handleEvents(events, hands, state, timestep, img=None, dep=None, sessionName=None):
    """ 
    displays events
    """
    
    currentEvents = [ev for ev in events if ev.timestep == timestep]
    
    # display
    if img != None:
        disp = img.copy()
        y = 50
        x = 25
        color = (255,100,0)
        
        for ev in currentEvents:
            if ev.eventType == TOUCH_EVENT:
                text = "Touched " + ev.object.name
            elif ev.eventType == UNTOUCH_EVENT:
                text = "Untouched " + ev.object.name
            elif ev.eventType == OCCLUDE_EVENT:
                text = "Occluded " + ev.object.name
            elif ev.eventType == UNOCCLUDE_EVENT:
                text = "Unoccluded " + ev.object.name
            elif ev.eventType == CREATE_EVENT:
                text = "New object " + ev.object.name
            elif ev.eventType == TAKEAWAY_EVENT:
                text = "Took or moved " + ev.object.name
            elif ev.eventType == HAND_IN_EVENT: 
                text = "Hand in frame"
            elif ev.eventType == HAND_OUT_EVENT: 
                text = "Hand out of frame"
            else:
                text = ""
    
            print timestep, "handleEvents:", text


def updateModels(tablemodel, flatTable, oldstate, events, hands, mask, timestep):
    # update table model
    newtablemodel = detection.updateTableModel(tablemodel, flatTable, oldstate, events, mask, timestep)

    # update old model
    currentEvents = [ev for ev in events if ev.timestep == timestep]

    # ensure old objects are not contaminated
    prev_to_next = dict( [ (obj,copy.copy(obj)) for obj in oldstate.objects] )
    prev_to_next[Hand] = Hand
    
    # update new objects
    for ev in currentEvents:
        if ev.eventType == CREATE_EVENT:
            continue;
        
        obj_next = prev_to_next[ev.object]

        if ev.eventType == TOUCH_EVENT:
            obj_next.touched = True
            obj_next.touchedToggleTime = timestep
    
        elif ev.eventType == UNTOUCH_EVENT:
            obj_next.touched = False
            obj_next.touchedToggleTime = timestep
        
        elif ev.eventType == OCCLUDE_EVENT:
            obj_next.occluded = True
            obj_next.occludedToggleTime = timestep
            
        elif ev.eventType == UNOCCLUDE_EVENT:
            obj_next.occluded = False
            obj_next.occludedToggleTime = timestep
            
        elif ev.eventType == TAKEAWAY_EVENT:
            obj_next.offtable = True
            obj_next.offtableTime = timestep

    oldObjs = prev_to_next.values()
            
    # new objects
    newObjs = []
    for ev in events:
        if ev.eventType == CREATE_EVENT and ev.timestep == timestep:
            newObjs.append(ev.object)
            
    newstate = Tablestate(oldstate.sessionName, timestep) 
    newstate.objects = (oldObjs + newObjs)
    newstate.lastHandEnterTime = oldstate.lastHandEnterTime

    # update hands - that is one hack
    newstate.hands = hands
    #print len(newstate.hands)
    if len(newstate.hands) > len(oldstate.hands):
        #print "new hand timestep = ",timestep
        newstate.lastHandEnterTime = timestep
    
    # print newstate
    return (newtablemodel, newstate)



class Harness(object):
    def __init__(self, displayON=False, istcON=False, fps=False):
        # Display & Control variables
        self.displays = {}
        self.imgNum = 1
        self.dispNum = 1
        self.sessionname = time.strftime("%Y%b%d-%H:%M:%S")
        self.videoSize = (320, 240)
        self.displayON = displayON
        self.istcON = istcON
        self.fps = fps

        # Load up MASKING variables - use harness4.py to create them
        self.mask = imread("params_py/mask.png", -1)
        self.flatTable = imread("params_py/flatTable.png", -1)
        tmp = open("params_py/displayAreaRect.txt").readlines()
        self.displayArea = tuple([int(i) for i in tmp[0].split()])
        linePoints = open("params_py/linePoints.txt").readlines()
        linePoints = [(int(p.split()[0]), int(p.split()[1])) for p in linePoints]
        self.plines = zip(linePoints[1:], linePoints[:-1])

        # ROS variables
        rospy.init_node("harness_node")
        self.img_sub = message_filters.Subscriber("/camera/rgb/image_rect_color_sync", sensor_msgs.msg.Image)
        self.dep_sub = message_filters.Subscriber("/camera/depth_registered/image_rect_sync", sensor_msgs.msg.Image)
        self.br = CvBridge()
        queue_size = 5;
        self.ts = message_filters.TimeSynchronizer([self.img_sub,self.dep_sub],queue_size)
        self.ts.registerCallback(self.callback)

        # print object parameters
        print "Harness class create:"
        print "    mask.shape", self.mask.shape
        print "    faltTable.shape", self.flatTable.shape
        print "    displayArea", self.displayArea
        print "    plines", self.plines
        print "    sessionname",self.sessionname
        print "    frameGrabber.videoSize", self.videoSize
        print

        # Event related variables
        self.events = []
        self.state = Tablestate(self.sessionname, self.imgNum); self.state.hands = [None, None]
        self.tablemodel = self.flatTable.copy()
        self.fpsCalc = FPSCalc()
        
        imshow("Table model", util.scaleImage(outlineObjects(self.tablemodel, self.state.objects)))
        rospy.spin()

    def callback(self, imgmsg, depmsg):
        if self.fps:
            self.fpsCalc.printFPS("tableOnly")

        image = self.br.imgmsg_to_cv(imgmsg, "bgr8")
        depth = self.br.imgmsg_to_cv(depmsg, "32FC1")
        fullimg = np.asarray(image[:,:], dtype="uint8")
        fulldep = np.uint16( np.asarray(depth[:,:], dtype="float32") * 1000 ) # m -> mm
        (img,dep) = downsample(fullimg,fulldep,factor=2)
        if img == None:
            rospy.signal_shutdown("Program done")

        if self.displayON:
            # tmpimg = img.copy() # mask the img
            # tmpimg[self.mask < 0] = 0;
            # imshow("Input", tmpimg)
            imshow("Input", img)
            imshow("Depth", util.scaleImage(dep))

        # Hand detection
        hands = handtracking.findHands(img, dep, self.tablemodel, self.plines, self.mask)
        #print hands

        # Event detection
        self.events += detection.detectEvents(img, dep, self.state, hands, self.flatTable, self.tablemodel, self.mask, self.imgNum)
        handleEvents(self.events, hands, self.state, self.imgNum, img=img, sessionName=self.sessionname)
        (self.tablemodel, self.state) = updateModels(self.tablemodel, self.flatTable, self.state, self.events, hands, self.mask, self.imgNum)
        
        if self.displayON:
            imshow("Table model", util.scaleImage(outlineObjects(self.tablemodel, self.state.objects)))
            
        if self.istcON:
            util3.displayImage(self.displayArea, fullimg, self.state, self.imgNum)
            util3.displayDepth(self.displayArea, fulldep, self.state, hands, self.imgNum)
        
        # Keyboard handle
        self.handleKey(waitKey(30) % 256)
        
        # Increment count
        self.imgNum += 1
        self.dispNum = self.imgNum
        
    def handleKey(self, key):
        if key == 27:
            print "[INFO] Pressed ESC - Halt program NOW"
            rospy.signal_shutdown("")


#------------------------------#
#---- Program Main related ----#
#------------------------------#
if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("-p", "--profile", action="store_true", dest="profile", help="turn profiler on", default = False)
    parser.add_argument("-d", "--display", action="store_true", dest="display", help="show display windows", default = False)
    parser.add_argument("-f", "--fps", action="store_true", dest="fps", help="show ISTC-demo display windows", default = False)
    parser.add_argument("-i", "--istcDemo", action="store_true", dest="istcDemo", help="show ISTC-demo display windows", default = False)
    args = parser.parse_args()
    print args

    cmd = "Harness(args.display, args.istcDemo, args.fps)"
    if args.profile:
        import cProfile
        cProfile.run(cmd, "profiledHarness")
    else:
        eval(cmd)

