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
import copy

import util2 as util


class Tablestate(object):
    def __init__(self, sessionName, timestep):
        self.sessionName = sessionName
        self.timestep = timestep
        self.objects = []
        self.reappeared = []
        self.hands = [None, None]
        # lastHandEnterTime is the last time a hand appeared.
        # handIn event is the last time a hand appeared *after there were no hands in the frame*
        self.lastHandEnterTime = 1e7
        
        # todo replace these variables being passed back and forth all the time
        self.tablemodel = None
        self.flatTable = None
        self.mask = None
        

    def setObjects(self, objects):
        self.objects = copy.deepcopy(objects)

    def __str__(self):
        s = '<'
        s += self.sessionName 
        s += ' ' + str(self.timestep)
        
        for obj in self.objects:
            s += ' , ' + obj.name + ' ' + obj.status()
        
        s += ', hand entered ' + str(self.lastHandEnterTime)        
        s += '>'
        return s

    
    
