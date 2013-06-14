#! /usr/bin/python
import sys
import os

import numpy as np
import math
from cv2 import imread, imwrite, imshow, waitKey
import cv2



def getCoeffsFromPoints(dep, xx, yy):

    X2 = numpy.array(xx)
    Y2 = numpy.array(yy)
    Z2 = []
    ones2 = np.ones(len(xx))
    
    for i in range(len(xx)):
        Z2.append(dep[yy[i], xx[i]])
    Z2 = numpy.array(Z2)

    design = np.zeros((X2.shape[0], 3))
    design[:,0] = X2
    design[:,1] = Y2
    design[:,2] = ones2

    #coeffs_ = np.linalg.lstsq(design, Z)[0]
    coeffs = np.dot(np.linalg.pinv(design), Z2)
    Z3 = np.dot(design, coeffs)

    return np.array([coeffs[0], coeffs[1], -1, coeffs[2]])


def getCoeffs(dep, mask):
    rows = range(0,480)
    cols = range(0,640)
    [x,y] = np.meshgrid(cols, rows)

    X = x.reshape(-1)
    Y = y.reshape(-1)
    Z = np.int32(dep.reshape(-1))
    ones = np.ones(640*480)
    mI = mask.reshape(-1)

    X2 = X[mI]
    Y2 = Y[mI]
    Z2 = Z[mI]
    ones2 = ones[mI]

    design = np.zeros((X2.shape[0], 3))
    design[:,0] = X2
    design[:,1] = Y2
    design[:,2] = ones2


    #coeffs_ = np.linalg.lstsq(design, Z)[0]
    coeffs = np.dot(np.linalg.pinv(design), Z2)
    Z3 = np.dot(design, coeffs)

    return np.array([coeffs[0], coeffs[1], -1, coeffs[2]])



def getDistTable(coeffs, dep):
    magNormal = math.sqrt(coeffs[0]*coeffs[0] + coeffs[1]*coeffs[1] + coeffs[2]*coeffs[2])
    scoeffs = coeffs/magNormal

    rows = range(0,480)
    cols = range(0,640)
    os = np.ones((480,640), 'float32')

    [x,y] = np.meshgrid(cols, rows)
    #imshow('x', np.uint16(x)*100)
    #imshow('y', np.uint16(y)*100)
    #waitKey()

    term1 = x*scoeffs[0]
    term2 = y*scoeffs[1]
    term4 = os*scoeffs[3]
    #imshow('term1', np.int32(term1))
    #imshow('term2', np.int32(term2))
    #imshow('const term', np.int32(term4))

    z = np.int32(dep)
    term3 = z * scoeffs[2]
    #imshow('term3', term3)

    dists = np.abs(term1 + term2 + term3 + term4)
    print 'dists max and min', np.abs(dists).max(), np.abs(dists).min()
    
    return dists



if __name__ == '__main__':
    folder = sys.argv[1]
    
    dep = imread(os.path.join(folder, 'table','image_3_dep.png'), cv2.CV_LOAD_IMAGE_UNCHANGED)
    imshow('dep', np.uint8(dep/10))
    mask = imread(os.path.join(folder, 'mask.png'), cv2.CV_LOAD_IMAGE_UNCHANGED)
    mask = mask <= 0
    #imshow('mask', np.uint8(mask)*255)
    waitKey()

    coeffs = getCoeffs(dep,mask)
    print "%g %g %g %g" % tuple(coeffs)
    
    dists = getDistTable(coeffs, dep)        
    
    dists[dists > 50] = 0
    
    imshow('dists', np.uint8(dists)*5)
    
    tableMask = dists < 15;
    imshow('foreground', np.uint8(tableMask)*255)

    waitKey()


























