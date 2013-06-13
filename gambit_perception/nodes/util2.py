#===============================================================================
# util.py
# 
# Utility functions.
#===============================================================================

import cv2.cv as cv
import cv2
from cv2 import imread, imwrite, imshow, waitKey
from ctypes import *
import ctypes
import math
import numpy as np
import time
import sys
import config
import random
from numpy import array, dot
import os
#import matplotlib.pyplot as plt
from subtractTablePlane import getCoeffsFromPoints
from config import cam_x, cam_y, camAlpha, MIN_OBJSIZE_FRAC

camCenter = np.array([cam_x, cam_y])



class FrameGrabberFiles(object):
    def __init__(self, recordingPath):
        self.recordingPath = recordingPath
        
        depName = 'image_' + str(1) + '_dep.png'
        depPath = os.path.join(self.recordingPath, depName)
        dep = imread(depPath, -1)
        self.videoSize = (dep.shape[1], dep.shape[0])
        
        self.mask = ~np.bool8(imread(os.path.join(recordingPath, 'mask.png'), -1))
        self.flatTable = buildMinMap(os.path.join(recordingPath, 'table'))
        
        self.img = np.zeros((dep.shape[0], dep.shape[1],3 ), 'uint8')
        self.dep = np.zeros(dep.shape, 'uint8')
    
    def grab(self, num):
        imgName = 'image_' + str(num) + '_rgb.png'
        depName = 'image_' + str(num) + '_dep.png'
        
        imgPath = os.path.join(self.recordingPath, imgName)
        depPath = os.path.join(self.recordingPath, depName)
        
        self.img = imread(imgPath, -1)
        self.dep = imread(depPath, -1)
        self.fullimg = self.img
        self.fulldep = self.dep
        
        if self.img == None or self.dep == None:
            print 'End of video detected.'
            print imgPath
            print depPath
        
        return self.img, self.dep
    
    




#===============================================================================
# Find the point where 2 lines meet.
# @param l1: tuple representing a line, of the form (m, b)
# @param l2: a second line tuple
# @return: x,y tuple where the points meet
#===============================================================================
def traceContoursCV2(img):
    if img.dtype == 'bool':
        img = np.uint8(img)
    regions, trash = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    blobs = []
    
    for r in regions:
        r2 = []
        for p in r:
            r2.append((p[0][0], p[0][1]))
        blobs.append(r2)
        
    return blobs


def fitTable(img, dep):
    points = [(100,100), \
              (540,100), \
              (540,380), \
              (100,380), \
              ]
    xx = [p[0] for p in points]
    yy = [p[1] for p in points]
    
    res = getCoeffsFromPoints(dep, xx, yy)
    
    return res


def zeroTablePlane(img, dep, coeffs, thresh=10):
    dists = getDistTable(coeffs, dep)
    fgMask = dists > thresh
    
    return fgMask
    

def cropImages(img, dep, fgMask):
    contours = traceContoursCV2(fgMask)
    contours.sort(lambda c: blobsize(c))    

    rect = boundingRect(contours[0], reverse=True)
    cropImg = cropImage(img, rect)
    cropDep = cropImage(dep, rect)
    
    return cropImg, cropDep, rect


def intersectionPoint(l1, l2):
    if type(l1) == int and type(l2) == int and l1 != l2:
        raise Exception('Two parallel lines won\'t ever intersect, no matter how hard you try')
        return None
    elif len(l1) == 1:
        return (l1, solveForLine(l2, x=l1))
    elif len(l2) == 1:
        return (l2, solveForLine(l1, x=l2))
    elif l1[1] == l2[1]: # they meet at zero
        return (0,0)
    else:
        x2 = float(l2[1]-l1[1])/(l1[0]-l2[0])
        return (x2, solveForLine(l1, x=x2))


def depthBackgroundSubtract(depth, depth_diff_mask, staticMap=None):
    if not staticMap:
        staticMap = cv2.imread("staticmapCV2.png", cv2.CV_LOAD_IMAGE_UNCHANGED)
    np.zeros(depth.shape, dtype="uint16")
    nodepthmask = cv2.compare(depth, np.array(10), cv2.CMP_LE)
    depth[nodepthmask != 0] = 10000

    depth_diff_mask = cv2.compare(depth, staticMap, cv2.CMP_LT)

def segmentAndMask(rgb, depth, mask, rgb_final, depth_final, thresh, staticMap = None):
    if not staticMap:
        staticMap = cv2.imread("staticmapCV2.png", cv2.CV_LOAD_IMAGE_UNCHANGED)

    rgb_final = rgb.copy()
    depth_final = depth.copy()

    # only keep foreground
    foregroundMask = np.zeros(depth.shape, dtype="uint8")
    depthBackgroundSubtract(depth, foregroundMask)

    backgroundMask = np.zeros(depth.shape, dtype="uint8")

    cv2.bitwise_not(foregroundMask, backgroundMask)
    rgb_final[backgroundMask != 0] = 0
    depth_final[backgroundMask != 0] = 0
    
    # mask out the pixels with no depth
    noDepthMask = cv2.compare(depth, np.array(0), cv2.CMP_LE)
    rgb_final[noDepthMask != 0] = 0
    depth_final[noDepthMask != 0] = 0

    # mask out pixels specified in mask
    rgb_final[mask != 0] = 0
    depth_final[mask != 0] = 0


def buildMinMapFrames(frames):
    debug = False
    
    big = 65535
    minMap = np.ones(frames[0].shape, 'uint16')*big
    
    for frame in frames:
        if debug:
            print minMap.max(), minMap.min()
        frame[frame <= 0] = big
        minMap = np.minimum(minMap, frame)
        
    minMap[minMap == big] = 0
    return minMap
            

def buildMinMap(folder, needFlip=False):
    debug = False

    fnames = os.listdir(folder)
    fnames = [fn for fn in fnames if fn.find('dep.png') > 0]

    shp = None
    if fnames:
        frame = imread(os.path.join(folder, fnames[0]), -1)
        shp = frame.shape

    big = 65535
    minMap = np.ones(shp, 'uint16')*big
        
    for fn in fnames:
        if debug:
            print minMap.max(), minMap.min()
    
        path = os.path.join(folder, fn)
        frame = imread(path, -1)
                
        frame[frame <= 0] = big
        minMap = np.minimum(minMap, frame)
        
    minMap[minMap == big] = 0
    return minMap
    


def testBuildMinMap():
    folder = 'video-22/table'
    dst = buildMinMap(folder)
    
    print dst.max()
    mult = (256*256)/dst.max()
    print mult
    
    imshow('minmap', dst*mult)    
    waitKey()




def buildAverageMap(folder, needFlip=False):    
    fnames = os.listdir(folder)
    fnames = [fn for fn in fnames if fn.find('dep.png') > 0]

    shp = None
    if fnames:
        frame = imread(os.path.join(folder, fnames[0]), -1)
        shp = frame.shape

    sumMap = np.zeros(shp, 'uint32')
    for fn in fnames:
        path = os.path.join(folder, fn)
        frame = imread(path, -1)
        sumMap += frame

    avgMap = np.uint16(sumMap / len(fnames))

    return avgMap
    
                

def testBuildAverageMap():
    folder = 'video-22/table'
    dst = buildAverageMap(folder)    

    print dst.max()
    mult = (256*256)/dst.max()
    
    imshow('avgmap', dst*mult)    


def maskImage(cvimg, maskimg):
    #mask the images
    cvimg[maskimg != 0] = 0

def maskNoDepth(rgb, depth):
    depth_diff_mask = cv2.compare(depth, np.array(0), cv2.CMP_GT)
    rgb[depth_diff_mask == 0] = 0

def cv2array(im):
    depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
        }

    arrdtype=im.depth
    a = np.fromstring(im.tostring(), dtype=depth2dtype[im.depth], count=im.width*im.height*im.nChannels)
    a.shape = (im.height,im.width,im.nChannels)
      
    if im.nChannels == 1:
        a.shape = (im.height, im.width)
        
    return a

def array2cv(a):
      dtype2depth = {
            'uint8':   cv.IPL_DEPTH_8U,
            'int8':    cv.IPL_DEPTH_8S,
            'uint16':  cv.IPL_DEPTH_16U,
            'int16':   cv.IPL_DEPTH_16S,
            'int32':   cv.IPL_DEPTH_32S,
            'float32': cv.IPL_DEPTH_32F,
            'float64': cv.IPL_DEPTH_64F,
        }
      try:
            nChannels = a.shape[2]
      except:
            nChannels = 1

      cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]), dtype2depth[str(a.dtype)], nChannels)
      cv.SetData(cv_im, a.tostring(), a.dtype.itemsize*nChannels*a.shape[1])

      return cv_im

# return a string representation of the time: year, day, hour, minute, second
# returns a unique, ascending number for every second. good for file names
def timeString():
    return time.strftime('%Y%j%H%M%S')


def shapeMatch(obj1, obj2, thresh=.63):
    area1 = cv2.countNonZero(np.uint8(obj1.mask))
    area2 = cv2.countNonZero(np.uint8(obj2.mask))
    
    fracs = [float(area1)/area2, float(area2)/area1]
    fracs.sort()
    return fracs[0] > thresh



#===============================================================================
# Find homography wrapper from ctypes-opencv. Fixed a bug here too.
# @param src_points: python list of 4 source points
# @param dst_points: list of destination points
# @return: The homography as an OpenCV matrix
#===============================================================================
def cvFindHomography(src_points, dst_points, homography=None):
    if homography is None:
        homography = cv.CreateMat(3, 3, cv.CV_64FC1)
    
    # HELLO. THIS IS A BIG HACK.
    # currently os x only runs OCV 1.0. So if this is running on os x, use the old ocv 1.0 call
    if sys.platform == 'darwin':
        cv.FindHomography(src_points, dst_points, homography)
    else:    
        cv._cvFindHomography(src_points, dst_points, homography, 0, 0, None)
    return homography

#===============================================================================
# Calculate the Euclidean distance between two points.
# @param a: first point
# @param b: second point
# @param axis: if 0 or 1, find distance only along this axis
# @return: The distance as a float
#===============================================================================
def distance(a,b,axis=2):
    if axis != 2:
        return abs(a[axis]-b[axis])
    else:
        return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))

#===============================================================================
# Convert an opencv sequence to an array of points.
# @param seq: an OpenCV CvSeq object
# @return: a Python list with the same points.
#===============================================================================
def convertSeqToPoints(seq):
    return list(seq)

#===============================================================================
# Convert a list of python points to a (python) list of CV points. Used for some drawing functions.
# @param points: python list of python points/tuples
# @return: python list of CvPoint objects.
#===============================================================================
def convertPointsToCVPoints(points):
    if points is None: return None
    else:
        cvPoints = []
        for p in points:
            cvPoints.append((p[0],p[1]))
        return cvPoints

def convertPointsToNumpyPoints(points):
    if points is None: return None
    else: return np.array(points)

#===============================================================================
# Find the equation for a line, given 2 points.
# @param p1: a point
# @param p2: another point
# @return: the slope and y-intercept as a tuple (m, b)
#===============================================================================
def lineBetween(p1, p2):
    # if parallel, just return the x value
    if p1[1] == p2[1]:
        return (p2[0])
    else:
        if p1[0] == p2[0]:
            slope = 0
        else: slope = float(p2[1]-p1[1])/(p2[0]-p1[0])
        intercept = p1[1]-slope*p1[0]
        
        return (slope, intercept)

#===============================================================================
# Returns true if 2 lines are parallel.
# @param l1: tuple representing a line, of the form (m, b)
# @param l2: a second line tuple
# @return: True if lines are parallel, else False.
#===============================================================================
def linesAreParallel(l1, l2):
    return type(l1) == int or type(l2) == int or (l1[0] == l2[0] and (len(l1) == 1 or len(l2) == 1 or l1[1] != l2))

#===============================================================================
# Pass a line and either a y or x value, returns the point on the line with that value.
# @param line: tuple representing a line, of the form (m, b)
# @param x: optional x value
# @param y: optional y value
# @return: if given x, returns the y value. if given y, returns the x value.
#===============================================================================
def solveForLine(line, x=None, y=None):
    if len(line) == 1: # parallel to y axis
        print None
    else:
        m, b = line
        if x != None: # solve for y
            return m*x + b
        elif y != None: # solve for x
            return (y-b)/m
        else: # No argument specified
            raise Exception('No argument specified')
            return None

#===============================================================================
# Return a line that is parallel to the line, but goes through the point.
# @param line: tuple representing a line, of the form (m, b)
# @param point: a point
# @return: line that is parallel to the line, but goes through the point.
#===============================================================================
def findParallelLineThroughPoint(line, point):
    print line
    print point
    if len(line) == 1: #parallel to y axis 
        return point[0]
    else:
        m = line[0]
        b = point[1]-m*point[0]
        return (m,b)

        
#return true if the y value of the point is greater than the y value of the line at that x
# l = (m, b) for y = mx+b
def isPointAboveLine(p, l):
    return (p[1] > (l[0]*p[0] + l[1]))

def distanceFromLine(p, l, axis=2):
    bi = p[1] - p[0]/l[0]
    li = (1/l[0], bi)
    p1 = intersectionPoint(l, li)
    return distance(p,p1,axis)

def distanceFromLinePt(p, pts):
    """ p = point, pts = 2 points on the line 
        requires line in point-point form 
    """
    lv = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    lv = normalize(lv)
    
    pv = (p[0]-pts[0][0], p[1]-pts[0][1])
    
    parCoeff = pv[0]*lv[0] + pv[1]*lv[1]
    parComp = (parCoeff * lv[0], parCoeff * lv[1])
    perpComp = (pv[0] - parComp[0], pv[1] - parComp[1])
    
    return magnitude(perpComp)


def magnitude(vect):
    norm = math.sqrt(vect[0]*vect[0] + vect[1]*vect[1])
    return norm

def normalize(vect):
    norm = math.sqrt(vect[0]*vect[0] + vect[1]*vect[1])
    return (vect[0]/float(norm), vect[1]/float(norm))
    
def convertXYtoMB(point1, point2):
    """
    Converts a set of (x, y) points to an (m, b) point
    Returns this point as a python tuple
    """
    dx = (point1[0] - point2[0])
    dy = point1[1] - point2[1]

    if (dx == 0):
        dx = .00001

    m = float(dy) / dx
    b = point1[1] - (m * point1[0])

    return (m, b)
        

def getCrossPoints(lines, polygon, returnLine=False, returnIdxs=False):
    crossPoints = []
    crossIdxs = []
    crossLine = None

    # Also process every line
    for line in lines:
        above = False
        below = False

        # Process every point in every blob
        for point,i in zip(polygon, range(len(polygon))):
            isabove = isPointAboveLine(point, line)
            above |= isabove    
            below |= (not isabove)

            # The point crosses the line, so we add it to the list
            # Convert the point to (m, b) format before we do.
            if (above and below):
                mbPoint = convertXYtoMB(point, lastPoint)
                ip = intersectionPoint(mbPoint, line)
                crossPoints.append((int(round(ip[0], 0)), int(round(ip[1]))))

                above = False
                below = False
                crossLine = line
                crossIdxs.append(i)

            lastPoint = point
        
    # return the right things
    if returnLine and returnIdxs:
        return (crossPoints, crossLine, crossIdxs)
    elif returnLine:
        return crossPoints, crossLine
    elif returnIdxs:
        return crossPoints, crossIdxs
    else:
        return crossPoints
     


# from adaptors.py
def NumPy2Ipl(input):
      """Converts a numpy array to the OpenCV/IPL CvMat data format.
  
      Supported input array layouts:
         2 dimensions of numpy.uint8
         3 dimensions of numpy.uint8
         2 dimensions of numpy.float32
         2 dimensions of numpy.float64
      """
      
      if not isinstance(input, np.ndarray):
          raise TypeError, 'Must be called with numpy.ndarray!'
  
      # Check the number of dimensions of the input array
      ndim = input.ndim
      if not ndim in (2, 3):
          raise ValueError, 'Only 2D-arrays and 3D-arrays are supported!'
      
      # Get the number of channels
      if ndim == 2:
          channels = 1
      else:
          channels = input.shape[2]
      
      # Get the image depth
      if input.dtype == np.uint8:
          depth = cv.IPL_DEPTH_8U
      elif input.dtype == np.float32:
          depth = cv.IPL_DEPTH_32F
      elif input.dtype == np.float64:
          depth = cv.IPL_DEPTH_64F
      
      # supported modes list: [(channels, dtype), ...]
      modes_list = [(1, np.uint8), (3, np.uint8), (1, np.float32), (1, np.float64)]
      
      # Check if the input array layout is supported
      if not (channels, input.dtype) in modes_list:
          raise ValueError, 'Unknown or unsupported input mode'
      
      result = cv.CreateImage(
          (input.shape[1], input.shape[0]),  # size
          depth,  # depth
          channels  # channels
          )
      
      # set imageData
      # result.imageData = input.tostring()
      result.imageData = input#.tostring()
      
      return result

def arcLength(list, startPoint, endPoint):
    # we want to look both clockwise and CCW
    p1 = min(startPoint, endPoint)
    p2 = max(startPoint, endPoint)
    forward = list[p1:p2+1]
    forwardDist = 0
    for i in range(0,len(forward)-1):
        forwardDist += distance(forward[i],forward[i+1])
    
    # order the list around the end
    backward = list[p2:]+list[:p1+1]
    backDist = 0
    
    for i in range(0,len(backward)-1):
        backDist += distance(backward[i],backward[i+1])
    
    return min(forwardDist, backDist)

# find the average point from a list of x,y tuples
def averagePoint(list):

    xs = [x[0] for x in list]
    ys = [x[1] for x in list]
    
    return (float(sum(xs))/len(xs), float(sum(ys))/len(ys))

#return the point with the greatest value for the index provided
def maxPoint(list, index):
    return max(list, key=lambda x: x[index])

#returns a list consisting of points less than the line, provided as (m, b), for y = mx+b
def bisectPolygon(list, l):
    newpoints = []
    print "\n\n\nbisect:"
    lastPoint = list[len(list) - 1 ]
    
    crossingIndex = 0
    
    for p in list:
        #check if we crossed the line
        if (isPointAboveLine(p, l) != isPointAboveLine(lastPoint, l)) :
            m = (p[1] - lastPoint[1])/(p[0] - lastPoint[0])
            b = p[1] - m*p[0]
            intersection = intersectionPoint(l, (m, b))
            newpoints.append([intersection[0], intersection[1]])
            
            if not isPointAboveLine(p, l) : #we just entered
                crossingIndex = len(newpoints) - 1
                print "crossing in at:", intersection
            else :
                print "crossing out at:", intersection
        #add point if it belong
        if isPointAboveLine(p, l) == False:
            newpoints.append(p)
        
        lastPoint = p
    
    #we want a crossing point to be the at a[0]
    for i in range(0, crossingIndex):
        newpoints.append(newpoints[0])
        del newpoints[0]
    
    return newpoints
    
def crossesLine(polygon, line):
    above = False
    below = False
    for i in range(0, len(polygon)):
        isabove = isPointAboveLine(polygon[i], line)
        above |= isabove
        below |= (not isabove)
        if (above and below): return True
    return False

def crossesLineInArray(polygon, linearray):
    for i in range(0,len(linearray)):
        if crossesLine(polygon, linearray[i]) :
            return True
    return False
    
    
def pointOnCircle(radius, c_x, c_y, radians):
    radians -= 2*math.pi*(math.floor(radians/(2*math.pi)))
    dX = -radius*math.cos(radians)
    dY = radius*math.sin(radians)
    return (c_x+dX, c_y+dY)
    
# get the next power of two. used to size opengl textures, which must be powers of two on some card
def npot(num):
    val = 2
    while val < num:
        val *= 2
    return val

# reorder the corners to match the following: topleft, topright, bottomr
def reorderCorners(box):
    # first, sort by y distance (higher first)
    box2 = sorted(box,lambda x,y:int(x[1]-y[1]))
    # we may need to swap 0,1 and 2,3
    box3 = []
    if (box2[0][0] < box2[1][0]):
        box3.append(box2[0])
        box3.append(box2[1])
    else:
        box3.append(box2[1])
        box3.append(box2[0])
    if (box2[2][0] > box2[3][0]):
        box3.append(box2[2])
        box3.append(box2[3])
    else:
        box3.append(box2[3])
        box3.append(box2[2])
    return box3

# reduce the contour to an N pointed shape
def reduceToPoints(contour2, numPoints):
    contour = contour2[:]
    
    while len(contour) > numPoints:
        # find the 2 closest points and merge them
        firstPoint = contour[0]
        secondPoint = contour[1]
        minDistance = 10000000
        
        for i in range(0,len(contour)):
            for j in range(i+1,len(contour)):
                if distance(contour[i],contour[j])<minDistance:
                    firstPoint = contour[i]
                    secondPoint = contour[j]
                    minDistance = distance(contour[i],contour[j])
        # merge the 2 closest points
        newPoint = midPoint(firstPoint,secondPoint)
        contour.remove(firstPoint)
        contour.remove(secondPoint)
        contour.append(newPoint)
    return contour

def midPoint(a,b):
    return ((1.*a[0]+b[0])/2,(1.*a[1]+b[1])/2)

def growBlob(points, growby):
    avgPnt = averagePoint(points)
    points = map(lambda (x,y): (x + math.copysign(growby, x-avgPnt[0]),y + math.copysign(growby, y-avgPnt[1])), points)
    return points
    

#===============================================================================
# return true if the points are inside the rect.
# @param points: a list of points
# @param rect: a rectangle [x,y,width,height] 
#===============================================================================
def pointsInsideRect(points, rect):
    for p in points:
        x, y = p
        rx, ry, rw, rh = rect
        if x >= rx and x < rx+rw and y >= ry and y < ry+rh:
            pass
        else:
            return False
    return True  

#===============================================================================
# return true if ANY of the points are inside the rect.
# @param points: a list of points
# @param rect: a rectangle [x,y,width,height] 
#===============================================================================
def anyPointsInsideRect(points, rect):
    for p in points:
        x, y = p
        rx, ry, rw, rh = rect
        if x >= rx and x < rx+rw and y >= ry and y < ry+rh:
            return True
    return False
    
    
    
def pointInsidePolygon(poly, p) :
    insideRect = False
    for i in range(0, len(poly)) :
        p1 = poly[i]
        p2 = poly[0] if i == (len(poly) - 1) else poly[i+1]
        
        if (p1[1]<p[1] and p2[1]>=p[1]) or (p2[1]<p[1] and p1[1]>=p[1]) :
            if p1[0]+(p[1]-p1[1])/(p2[1]-p1[1])*(p2[0]-p1[0])<p[0] :
                insideRect = not insideRect
    
    return insideRect
    
def minPoints(screenObject, minP=(1E4000, 1E4000)):

    minP = ( min(minP[0], screenObject.x), min(minP[1], screenObject.y))
        
    for s in screenObject.getScreenObjects() :
        minN = minPoints(s)
        minP = ( min(minP[0], minN[0]), min(minP[1], minN[1]))
    
    return minP

def boundingRect(contour) :
    minx = reduce(lambda x, p : min(x, p[0]), contour, 1E4000)
    miny = reduce(lambda y, p : min(y, p[1]), contour, 1E4000)
    maxx = reduce(lambda x, p : max(x, p[0]), contour, -1E4000)
    maxy = reduce(lambda y, p : max(y, p[1]), contour, -1E4000)    
    return (minx, miny, (maxx-minx), (maxy-miny))

def boundingRectForList(blobs):
    minx = 1e6
    miny = 1e6
    maxx = -1e6
    maxy = -1e6
    for b in blobs:
        rect = boundingRect(b)
        if rect[0] < minx:
            minx = rect[0]
        if rect[1] < miny:
            miny = rect[1]
        if rect[0]+rect[2] > maxx:
            maxx = rect[0]+rect[2]
        if rect[1]+rect[3] > maxy:
            maxy = rect[1]+rect[3]

    rect = (int(minx), int(miny), int(maxx-minx), int(maxy-miny))
    return rect

def cvRectangle(img, rect, colortup=None):
    if not colortup:
        color = cv.Scalar(255,255,255)
    else:
        color = cv.Scalar(colortup[0], colortup[1], colortup[2])
    cv.Rectangle(img, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), color)


def middlePoint(contour) :
    rect = boundingRect(contour)
    return (rect[0] + rect[2]/2, rect[1] + rect[3]/2)

    
def avgPoint(contour) :
    tot = float(len(contour))
    
    return reduce(lambda avgp, p : (avgp[0]+(p[0]/tot), avgp[1]+(p[1]/tot) ), contour, (0,0))

def blobsize(contour, exact=True, ignoreLessThan=0):
    if not exact:
        rect = boundingRect(contour)
        return rect[2]*rect[3]
    else:
        rect = boundingRect(contour)
        blob = contour
        
        # first filter by rectangle size: this function gets called a lot, and countNonZero takes up a lot of time.
        areaUpperBound = rect[2]*rect[3]
        if areaUpperBound < ignoreLessThan:
            return 0
        else:
            tmpMask = np.zeros((config.videoHeight, config.videoWidth), dtype="uint8")
            cv2.fillPoly(tmpMask, [np.array(blob, dtype="int32")], 255)
            area = cv2.countNonZero(tmpMask)            
            return area

def angleBetweenPoints(p1, p2) :
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    

def closestPoint(lst, pnt):
    minDist = 1000000000
    minPnt = None
    for l in lst :
        dist = distance(l, pnt)
        if dist < minDist :
            minDist = dist
            minPnt = l
            
    return minPnt

    
def overlapFracSym(b1,b2, disp=False):
    """
        (fraction of b1, fraction of b2)
        slow; uses opencv masks
    """
    tmp1 = np.zeros((config.videoHeight, config.videoWidth), dtype="uint8")
    tmp2 = np.zeros((config.videoHeight, config.videoWidth), dtype="uint8")

    cvWhite = (255,255,255)
    cv2.fillPoly(tmp1, [np.array(b1, dtype="int32")], cvWhite)    
    cv2.fillPoly(tmp2, [np.array(b2, dtype="int32")], cvWhite)
    tmp3 = cv.bitwise_and(tmp1, tmp2)

    area1 = cv2.countNonZero(tmp1)
    area2 = cv2.countNonZero(tmp2)
    area3 = cv2.countNonZero(tmp3)
    
    cv2.putText(tmp3, str(float(area3)/area2), (450, 50), cv2.FONT_HERSHEY_PLAIN, 1, cvWhite)
    if config.DISPLAY_FLAG:
        cv2.imshow("overlapFrac", tmp3)
    
    return (float(area3)/area1, float(area3)/area2)

def matchByOverlap(objects, blobs, thresh, display=False):
    # returns dict that contains matches (1 to 1)
    # returns remaining blobs that haven't been matched
    result = {}
    matchedBlobs = [0,]*len(blobs)
    
    keys = range(len(objects))
    try:
        keys = objects.keys()
    except AttributeError:
        pass

    for j,b in zip(range(len(blobs)), blobs):
        for i in keys:
            (of1,of2) = overlapFracSym(objects[i].blob(), b)
            if of1 > thresh and of2 > thresh:
                result[i] = b
                matchedBlobs[j] = 1

    remainingBlobs = [blobs[j]  for j in range(len(blobs)) if not matchedBlobs[j]]
    return (result, remainingBlobs)

# untested
# difference between this and matchByOverlap is that this returns
# two lists of indices -- first list are the indices of arg2 that
# that match to arg1
# second list are the indices of arg1 that match to arg2
def matchByOverlap2(objects, blobs, thresh, display=False):
    inds1 = {}
    inds2 = {}
    
    keys = range(len(objects))
    try:
        keys = objects.keys()
    except AttributeError:
        pass

    keys1 = keys;
    keys2 = range(len(blobs))
    for k in keys1:
        inds1[k] = -100
    for k in keys2:
        inds2[k] = -100
    
    for j,b in zip(range(len(blobs)), blobs):
        for i in keys:
            (of1,of2) = overlapFracSym(objects[i].blob(), b)
            if of1 > thresh and of2 > thresh:
                inds1[i] = j
                inds2[j] = i
                
    return inds1, inds2


def matchBySize(objects, blobs, thresh):
    # return dict
    # assumes thresh is in [0,1]
    
    result = {}
    keys = range(len(objects))
    try:
        keys = objects.keys()
    except AttributeError:
        pass

    for i in keys:
        lst = []
        for j in range(len(blobs)):
            ratio = float(objects[i].size())/blobsize(blobs[j], exact=True)
            if ratio >= thresh or ratio <= 1/thresh:
                lst.append((j,blobs[j]))
        result[i] = lst
        
    return result
                

def prod(lst):
    return reduce(lambda x,y: x*y, lst);


def sampleFromDiscrete(table):
    # assumes value in table sum to 1
    result = None
    r = random.random();

    s = 0;
    for key in table:
        tmp = (s, s + table[key])
        if r >= s and r < s + table[key]:
            result = key
        s = s + table[key]

    return result

def l2norm(x,y):
    sq = (x[0] - y[0])*(x[0] - y[0]) + (x[1] - y[1])*(x[1] - y[1])
    return math.sqrt(sq)

def drawBlob(img, b, color=(255,255,255), thick=2):
    for i in range(0, len(b)):
        x,y = b[i]
        cv2.line(img, b[i], b[(i+1) % len(b)], color, thickness=thick)

def putText(img, text, x, y):
    try:
        nChannels = img.shape[2]
    except:
        nChannels = 1
    if nChannels == 3 and img.dtype == "uint8":
        colorVal = (255,255,255)
    elif nChannels == 1 and img.dtype == "uint16":
        colorVal = 65535
    else:
        colorVal = 255

    cv2.putText(img, text, (int(x), int(y)), cv2.FONT_HERSHEY_PLAIN, 1, colorVal)


# ============= Transform-specific ================= #
    
def invertTransform(arr):
    R = arr[0:3, 0:3];
    t = arr[0:3, 3];
    
    Rprime = R.transpose();
    tprime = -np.dot(Rprime,t)
    
    Tprime = np.eye(4)
    Tprime[0:3, 0:3] = Rprime
    Tprime[0:3, 3] = tprime
    return Tprime


def projectPoints(X_wor, Twc):
    if X_wor.ndim > 1:
        m = X_wor.shape[1]
    else:
        m = 1

    #X_wor = .3*array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 0]]);
    X_hom = np.ones((4,m));
    if m > 1:
        #X_wor = X_wor.transpose();
        X_hom[0:3, 0:m] = X_wor;
    else:
        X_hom[0] = X_wor[0]
        X_hom[1] = X_wor[1]
        X_hom[2] = X_wor[2]
        
    X_cam = dot(Twc, X_hom);
    X_im = array([ X_cam[0,:]/X_cam[2,:], X_cam[1,:]/X_cam[2,:] ]);
    X_draw = X_im*camAlpha + n.tile(camCenter, (X_im.shape[1], 1)).transpose();

    return X_draw


def worldPoint(j, i, d, Tcw):
    px = camCenter[0]
    py = camCenter[1]
    #print 'px, py', px, py
    x = (j - px)/camAlpha
    x *= d
    y = (i - py)/camAlpha
    y *= d
    z = d
    
    X = np.array([x, y, d, 1])
    X_ = np.dot(Tcw, X.transpose())
        
    return (X_[0], X_[1], X_[2])



def worldPoints(jj, ii, dd, Tcw):
    
    print 'world points'
    # print jj
    # print ii
    # print dd
    # print

    px = config.cam_x
    py = config.cam_y
    
    xx = (jj - px)/float(config.camAlpha)
    yy = (ii - py)/float(config.camAlpha)
    
#    plt.scatter(xx.copy(),yy.copy())
#    plt.figure(); 
#    plt.plot(np.arange(len(xx)), xx)
#    plt.figure()
#    plt.plot(np.arange(len(xx)), yy)
#    plt.show()

    
    yy = yy*(dd*.001)
    xx = xx*(dd*.001)
    
    #plt.scatter(xx,yy)
    #plt.figure(); plt.scatter(xx,yy)
#   #plt.figure()
    #plt.plot(xx)
    #plt.figure()
    #plt.plot(yy)
    print 'x,y camera frame'


    X = np.zeros((4, len(xx)))
    X[0,:] = xx
    X[1,:] = yy
    X[2,:] = dd
    
    disp = np.zeros((480,640), 'uint8')
    imshow(disp)
    waitKey()
    
    print 'x,y again'
    X = np.float64(X)

    
    X_ = np.dot(Tcw, X)
    
#    plt.scatter(X_[0,:], X_[1,:])
#    print 'x,y transformed'
#    plt.show()
    
    return X_



def testWorldPoints2():
    path = '/home/jinna/kitchen_data/dataCake/cake2/Tcw.txt'
    Tcw = open(path).readlines()
    #print Tcw
    Tcw = [l.strip().split() for l in Tcw]
    print Tcw
    Tcw = [np.array([float(n) for n in row]) for row in Tcw]
    print Tcw
    Tcw = np.asarray(Tcw)
    print 'Tcw', Tcw
    
    # unit square
    p0 = (0,0,0)
    p1 = (0,1,0)
    p2 = (1,1,0)
    p3 = (1,0,0)
    
    impath = '/home/jinna/kitchen_data/dataCake/cake2/image_2500_rgb.png'
    im = imread(impath, -1)
    deppath = '/home/jinna/kitchen_data/dataCake/cake2/image_2500_dep.png'
    dep = imread(deppath, -1)
    
    square = np.array([p0, p1, p2, p3]).transpose()
    square = square*.25
    print square
    drawPts = projectPoints(square, invertTransform(Tcw))
    print drawPts
    
    for i in range(4):
        q0 = drawPts[:,i]
        q0 = (int(q0[0]), int(q0[1]))
        q1 = drawPts[:,(i+1)%4]
        q1 = (int(q1[0]), int(q1[1]))
        cv2.line(im, q0, q1, (255,255,0), thickness=2)

    imshow('unit square', im)
    waitKey()


    
#cpputils = cdll.LoadLibrary("./libcpputils.so")
#def renderDepth(hsv, depthBox, ulCorner, hue):
#    cpputils.renderDepth(pointer(hsv), pointer(depthBox), c_int(ulCorner[0]), c_int(ulCorner[1]), c_int(hue))
#    return


def highlight(img, mask, chan='b', strength=100):
    # safety checks
    if len(img.shape) > 2:
        print 'mask nchannels', img.shape[2]
    elif not (mask.dtype == 'uint8'):
        print 'mask depth', mask.depth
    
    r = np.zeros(img.shape, dtype = "uint8")
    g = np.zeros(img.shape, dtype = "uint8")
    b = np.zeros(img.shape, dtype = "uint8")
    
    arr = [r, g, b]
    
    # shade in occluded segments with green
    cv2.split(img, arr)

    if chan == 'r':
        r[mask > 0] = r + strength
    elif chan == 'g':
        g[mask > 0] = g + strength
    else:
        b[mask > 0] = b + strength

    img = cv2.merge(arr)
    
    return

from config import NOMINAL_MIN_DEPTH, NOMINAL_MAX_DEPTH

def scaleImage(src, mask=None):
    tmp = src - src.min()
    
    if src.max() > src.min():
        dst = tmp * (255. / (src.max() - src.min()))
    else:
        dst = tmp
        
    return np.uint8(dst)



def scaleImageHard(src, mask=None):
    minv = NOMINAL_MIN_DEPTH
    maxv = NOMINAL_MAX_DEPTH

    #stuff = cv.MinMaxLoc(src, mask=mask)
    #(minv, maxv) = stuff

    tmp = src - minv
    
        #print 'ScaleImage:', (src.width, src.height, src.nChannels), (dst.width, dst.height, dst.nChannels)
    if maxv > minv:
        dst = tmp * (255. / (maxv - minv))
    else:
        dst = tmp
        
    return np.uint8(dst)


def listIntersection(lst1, lst2):
    return [x for x in lst1 if x in lst2]

def listSetDifference(lst1, lst2):
    return [x for x in lst1 if not x in lst2]
        

# Is Number of NonZERO Points in blob bigger then minObjSize?    
def isNegative(blob, imgDepth):
    vidArea = imgDepth.shape[0]*imgDepth.shape[1]
    minObjSize = vidArea * MIN_OBJSIZE_FRAC

    mask = region2mask(blob)
    maskedDep = imgDepth * mask
    #disp = maskedDep.copy() * (255. / 2500)
    ct = cv2.countNonZero(maskedDep)
    #print 'is negative?', ct, '<', minObjSize*.5    

    retval = ct < minObjSize*.5        
    return retval


def isNegative1(blob, imgDepth):
    mask = np.zeros((config.videoHeight, config.videoWidth), dtype="uint8")
    cv2.fillPoly(mask, [np.array(blob, dtype="int32")], 1)
    
    maskedDep = imgDepth * mask

    disp = maskedDep.copy() * (255. / 2500)
            
    #bsz = blobsize(blob, exact=True)
    bsz = cv2.countNonZero(mask)
    ct = cv2.countNonZero(maskedDep)
    
    retval =  ct < (.5*bsz)
    #print 'isNegative? ', bsz, ct, retval
    
    return retval
    

def region2mask(blob):
    mask = np.zeros((config.videoHeight, config.videoWidth), dtype="uint8")
    cv2.fillPoly(mask, [np.array(blob, dtype="int32")], 1)
    mask = np.bool8(mask)
    return mask
    

def regions2mask(blobs):
    mask = np.zeros((config.videoHeight, config.videoWidth), dtype="bool8")
    
    for b in blobs:
        mask1 = region2mask(b)
        mask = mask1 | mask
    
    return mask



def cvCopy(src, dst, mask=None):
    width1 = cv.GetImageROI(src)[2]
    height1 = cv.GetImageROI(src)[3]
    width2 = cv.GetImageROI(dst)[2]
    height2 = cv.GetImageROI(dst)[3]
    
    if not (src.depth == dst.depth and width1 == width2 and height1 == height2):
        print 'cvCopy argument error'
        print (width1, height1, src.depth), (width2, height2, dst.depth)
        raise RuntimeError

    cv.Copy(src, dst, mask)
        
    return

def cvConvertScale(src, dst, scale=None, offs=None):
    width1 = cv.GetImageROI(src)[2]
    height1 = cv.GetImageROI(src)[3]
    width2 = cv.GetImageROI(dst)[2]
    height2 = cv.GetImageROI(dst)[3]

    if not (width1 == width2 and height1 == height2 and src.nChannels == dst.nChannels):
        print 'cvConvertScale argument error'
        print (width1, height1, src.nChannels), (width2, height2, dst.nChannels)
        raise RuntimeError

    
    if scale and offs:
        cv.ConvertScale(src, dst, scale, offs)
    elif scale:
        cv.ConvertScale(src, dst, scale)
    elif offs:
        cv.ConvertScale(src, dst, offset = offs)
    else:
        cv.ConvertScale(src, dst)

def cvCmpS(src, scalar, dst, code):
    width1 = cv.GetImageROI(src)[2]
    height1 = cv.GetImageROI(src)[3]
    width2 = cv.GetImageROI(dst)[2]
    height2 = cv.GetImageROI(dst)[3]

    if not (width1 == width2 and height1 == height2):
        print 'cvCmpS src, dst size error'
        print (width1, height1), (width2, height2)

    cv.CmpS(src, scalar, dst, code)


def cvAnd(A, B, dst):
    width1 = cv.GetImageROI(A)[2]
    height1 = cv.GetImageROI(A)[3]
    width2 = cv.GetImageROI(B)[2]
    height2 = cv.GetImageROI(B)[3]
    width3 = cv.GetImageROI(dst)[2]
    height3 = cv.GetImageROI(dst)[3]

    if not (width1 == width3 and height1 == height3 and A.depth == dst.depth):
        print 'cvAnd argument error: size, type of src1, dst'
        print (width1, height1, A.depth), (width3, height3, dst.depth)
        raise RuntimeError

    cv.And(A,B,dst)
    
        
def displayDepth(name, dep, imshow=cv.ShowImage):
    width = cv.GetImageROI(dep)[2]
    height = cv.GetImageROI(dep)[3]
    disp = cv.CloneImage(dep)

    cv.ConvertScale(disp, disp, 10);
    imshow(name, disp)

    del disp


def subSampleImage(img, factor, debug = False):
    if factor != 2:
        raise "subSampleImage only knows how to quarter-sample"
    
    if len(img.shape) == 3:
        subSampled = img[::2, ::2, :]*1
    else:
        subSampled = img[::2, ::2]*1
    
    return subSampled
    

def tableFootprint(dep, mask, Twc):

    # project onto table
    blobs = traceContoursCV2(mask)
    b = blobs[0]
    #jj = [p[0] for p in b]
    #ii = [p[1] for p in b]
    ii,jj = mask.nonzero()
    dd = dep[ii, jj]
    #plt.scatter(jj, ii)
    
    wps = worldPoints(ii, jj, dd, Twc)

    plt.scatter(wps[0,:], wps[1,:])
    plt.show()
        
    b2 = worldPoints(np.asarray(ii), np.uint8(jj), dd, Twc)
    b3 = [(int(b2[0,i]), int(b2[1,i])) for i in range(b2.shape[1])]
    
    im = np.zeros((480, 640), 'uint8')
    drawBlob(im, b3)
    imshow('mask', np.uint8(mask)*255 )
    imshow('blob', im)
    waitKey()
    
    
    

def testTableFootprint():
    pass

#    smoothed = cv2.blur(img, (3, 3))
#    
#    if len(img.shape) == 2:
#        subSampled = np.zeros((img.shape[0] / 2, img.shape[1] / 2), dtype = img.dtype)
#    else:
#        subSampled = np.zeros((img.shape[0] / 2, img.shape[1] / 2, img.shape[2]), dtype = img.dtype)
#
#    for i in range(subSampled.shape[0]):
#        for j in range(subSampled.shape[1]):
#            if len(img.shape) == 3:
#                subSampled[i, j, :] = smoothed[factor*i, factor*j, :]
#            else:
#                subSampled[i, j] = smoothed[factor*i, factor*j]
                
                
    if (debug):
        cv2.imshow("Subsample", img)
        cv2.waitKey()
        cv2.imshow("Subsample", smoothed)
        cv2.waitKey()
        cv2.imshow("Subsample", subSampled)
        cv2.waitKey()

    #irange = range(0, smoothed.shape[0], factor)
    #jrange = range(0, smoothed.shape[1], factor)
    #subSampled = smoothed[irange, jrange, :]


    return subSampled


def raiseZeros(dep0, raiseTo, mask=None):
    dep = dep0.copy()
    
    if mask != None:
        nodepths = (dep == 0) & mask
    else:
        nodepths = dep == 0

    dep = dep + np.uint16(nodepths)*raiseTo    
    return dep


def lowerToZero(dep0, lowerFrom, mask=None):
    dep = dep0.copy()
    if mask != None:
        nodepths = (dep == lowerFrom) & mask
    else:
        nodepths = dep == lowerFrom
    dep = dep - np.uint16(nodepths)*lowerFrom
    return dep
    



def testSubSampleImage():
    img = imread('/home/jinna/kitchen_data/dataFake/mix2/image_100_rgb.png', -1)
    subSampleImage(img, 2, debug=True)

    img = imread('/home/jinna/kitchen_data/dataFake/mix2/image_100_dep.png', -1)
    subSampleImage(img, 2, debug=True)


def cropImage(img, rect, padding=0):
	x0 = max(0, rect[0]-padding)
	x1 = min(img.shape[1]-1, rect[0] + rect[2]+padding)
	y0 = max(0, rect[1]-padding)
	y1 = min(img.shape[0]-1, rect[1] + rect[3]+padding)
	rect = (x0, y0, x1-x0, y1-y0)

	if len(img.shape) == 3:
		cropped = img[rect[1]:(rect[1]+rect[3]), rect[0]:rect[0]+rect[2], :]
	else:
		cropped = img[rect[1]:(rect[1]+rect[3]), rect[0]:rect[0]+rect[2]]
	return cropped
        

if __name__ == '__main__':
    #testBuildAverageMap()
    #testBuildMinMap()
    testSubSampleImage()

    





