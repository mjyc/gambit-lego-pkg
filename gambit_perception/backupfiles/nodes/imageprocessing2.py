#! /usr/bin/python
from ctypes import *
from numpy.ctypeslib import ndpointer
import numpy as np
import cv2.cv as cv
import cv2 as cv2
import sys
import os
import numpy
from numpy import array, dot
import time
import math
import random
import traceback
import shutil
import pdb


# local imports
import util2 as util
from cv2 import imread, imshow, imwrite, waitKey,cv


### code ###
def segmentAndMask(img0, imgDepth0, staticMap0, mask0, thresh=10):
	debug = True
	
	staticMap = np.array(staticMap0)
	imgDepth = np.array(imgDepth0)
	img = np.array(img0)
	mask = np.array(mask0)
	#imshow('indepth', imgDepth*10)
	
	staticMap = np.int16(staticMap)
	imgDepth = np.int16(imgDepth)

	noDepthMask = imgDepth == 0;
	#imshow('no depth mask', np.uint8(noDepthMask)*255)
	
	diff = (imgDepth - staticMap)
	adiff = np.abs(diff)

	tableMask = adiff < thresh
	if debug:
		#imshow('adiff', adiff*15)
		print imgDepth.max(), staticMap.max()
		#imshow('table mask', np.uint8(tableMask)*255)
	
	# numpy setting is really slow
	maskout = (tableMask) | (mask <= 0)
	maskin = (tableMask <= 0) & (mask) 
	
	cvMask = util.array2cv(np.uint8(maskout)*255)
	cvImg = util.array2cv(img)
	cv.Set(cvImg, (0,0,0), cvMask)	
	img = util.cv2array(cvImg)
	#img[tableMask] = 0
	#img[mask <= 0] = 0
	
	cvDep = util.array2cv(imgDepth)
	depthBw = imgDepth>0
	depthBw = depthBw | noDepthMask
	depthBw = depthBw & maskin
	depthBw = np.uint8(depthBw)*255
	
	cv.Set(cvDep, 0, cvMask)
	imgDepth = util.cv2array(cvDep)
	imgDepth = np.uint16(imgDepth)	
	
	
	# Find contours. Only keep the large ones to reduce noise.
	param = 2
	for d in range(param):		
		depthBw = cv2.dilate(depthBw, None)
	for d in range(param):
		depthBw = cv2.erode(depthBw, None)
	
	#imshow('depthbw', depthBw)

	blobs = []
	blobs = traceContoursCV2(depthBw)
	blobs = [b for b in blobs if len(b) > 2]
	blobs = [b for b in blobs if util.blobsize(b, ignoreLessThan=150) > 150]

	foregroundMask = np.zeros(imgDepth.shape, 'uint8')
	mat = util.array2cv(foregroundMask)
	cv.FillPoly(mat, blobs, 255)	
	foregroundMask = util.cv2array(mat)	
	#imshow('foreground', foregroundMask)
	
	bgMask = util.array2cv(np.uint8(foregroundMask < 0)*255)

	cv.Set(cvImg, (0,0,0), bgMask)
	cv.Set(cvDep, 0, bgMask)
	img = util.cv2array(cvImg)
	imgDepth = util.cv2array(cvDep)
	imgDepth = np.uint16(imgDepth)



	if debug:
		#imshow('seg img', .5*np.float32(img0) + .5*np.float32(img0[foregroundMask]))
		img1 = img0.copy()
		img1[foregroundMask <= 0] = .5*img1[foregroundMask <= 0]
		#imshow('seg img', img1)
		#imshow('seg dep', imgDepth*10)
		#imshow('smoothed foreground mask', foregroundMask)

	return img, imgDepth, foregroundMask
	
	
# staticMap => tablemodel
# bigger threshold removes more points
def subtractTableSimple(img0, imgDepth0, staticMap0, mask0, thresh=10):
	staticMap = np.array(staticMap0)
	imgDepth = np.array(imgDepth0)
	img = np.array(img0)
	mask = np.array(mask0, 'bool8')
	
	staticMap = np.int16(staticMap)
	imgDepth = np.int16(imgDepth)
	#noDepthMask = imgDepth == 0; # mask for 0 (invalid) x,y positions
	#imshow('no depth mask', np.uint8(noDepthMask)*255)
	
	# mask for anything that is beyound tablemodel (allow some rooms by "thresh" value)
	tableMask = imgDepth >= (staticMap - thresh) 	# depths are at or farther than expected
	
#	diff = imgDepth - staticMap
#	imshow('diff', (diff-diff.min())*15)
#	print 'diff min, mean, max', diff.min(), diff.mean(), diff.max()
#	imshow('initial foreground mask', np.uint8(~tableMask)*255)
	
	fgMask = ~tableMask & mask
	
	return fgMask



def segmentAboveAndMask(img0, imgDepth0, staticMap0, mask0, thresh=10):
	debug = False
	
	staticMap = np.array(staticMap0)
	imgDepth = np.array(imgDepth0)
	img = np.array(img0)
	mask = np.array(mask0)
	#imshow('indepth', imgDepth*10)
	
	staticMap = np.int16(staticMap)
	imgDepth = np.int16(imgDepth)

	noDepthMask = imgDepth == 0;
	#imshow('no depth mask', np.uint8(noDepthMask)*255)
	

	tableMask = imgDepth >= (staticMap - thresh) 	# depths are at or farther than expected
	
	#diff = imgDepth - staticMap
	#imshow('diff', (diff-diff.min())*15)
	#print imgDepth.max(), staticMap.max()
	imshow('initial foreground mask', np.uint8(~tableMask)*255)
	
	# numpy setting is really slow
	maskout = (tableMask) | (mask <= 0)
	maskin = (tableMask <= 0) & (mask) 
	
	cvMask = util.array2cv(np.uint8(maskout)*255)
	cvImg = util.array2cv(img)
	cv.Set(cvImg, (0,0,0), cvMask)	
	img = util.cv2array(cvImg)
	#img[tableMask] = 0
	#img[mask <= 0] = 0
	
	cvDep = util.array2cv(imgDepth)
	depthBw = imgDepth>0
	depthBw = depthBw | noDepthMask
	depthBw = depthBw & maskin
	depthBw = np.uint8(depthBw)*255
	
	cv.Set(cvDep, 0, cvMask)
	imgDepth = util.cv2array(cvDep)
	imgDepth = np.uint16(imgDepth)	
	
	
	# Find contours. Only keep the large ones to reduce noise.
	param = 2
	for d in range(param):		
		depthBw = cv2.dilate(depthBw, None)
	for d in range(param):
		depthBw = cv2.erode(depthBw, None)
	
	#imshow('depthbw', depthBw)

	blobs = []
	blobs = traceContoursCV2(depthBw)
	blobs = [b for b in blobs if len(b) > 2]
	blobs = [b for b in blobs if util.blobsize(b, ignoreLessThan=150) > 150]

	foregroundMask = np.zeros(imgDepth.shape, 'uint8')
	mat = util.array2cv(foregroundMask)
	cv.FillPoly(mat, blobs, 255)	
	foregroundMask = util.cv2array(mat)	
	#imshow('foreground', foregroundMask)
	
	bgMask = util.array2cv(np.uint8(foregroundMask < 0)*255)

	cv.Set(cvImg, (0,0,0), bgMask)
	cv.Set(cvDep, 0, bgMask)
	img = util.cv2array(cvImg)
	imgDepth = util.cv2array(cvDep)
	imgDepth = np.uint16(imgDepth)

	if debug:
		#imshow('seg img', .5*np.float32(img0) + .5*np.float32(img0[foregroundMask]))
		img1 = img0.copy()
		img1[foregroundMask <= 0] = .5*img1[foregroundMask <= 0]
		#imshow('seg img', img1)
		#imshow('seg dep', imgDepth*10)
		#imshow('smoothed foreground mask', foregroundMask)

	foregroundMask = np.bool8(foregroundMask)
	#return img, imgDepth, foregroundMask
	return foregroundMask

	
	
def sub2ind(shape, subs):
    """ From the given shape, returns the index of the given subscript"""
    revshp = list(shape);
    revshp.reverse();
    mult = [1];
    for i in range(0, len(revshp)-1):
        mult.extend([mult[i]*revshp[i]]);
    mult.reverse();
    mult = numpy.array(mult).reshape(len(mult),1);
    
    idx = numpy.dot((subs) , (mult));
    return idx;




def accumulatePixelValues(folder):
	debug = True

	# Fake the depth subtraction
	staticMap = util.buildMinMap(os.path.join(folder, 'table'))
	mult = (256*256)/staticMap.max()
	imshow('map', staticMap*mult)

	mask = np.zeros(staticMap.shape, 'uint8')
	mask[35:389, 30:611] = 255


	# Get all images in hand directory
	fnames = os.listdir(folder)
	fnames = [f for f in fnames if f.find('image_') >= 0]
	n = len(fnames)/2


	# set up accumulator
	rgbValues = []

	for i in range(0, n):	
		print i
		img = imread(os.path.join(folder, 'image_'+str(i)+'_rgb.png'), -1)
		imgDepth = imread(os.path.join(folder, 'image_'+str(i)+'_dep.png'), -1)
		segImg, segImgDepth, fgMask = segmentAndMask(img, imgDepth, staticMap, mask)
		
				
		if fgMask.sum() > 0:
			# get list of pixel colors
			chan1 = segImg[:,:,0]		
			chan2 = segImg[:,:,1]
			chan3 = segImg[:,:,2]
		
			I = np.nonzero(fgMask > 0)
			print I[0].shape, I[1].shape
			n = I[0].shape[0]
			
			for i in range(n):
				j = I[0][i]
				k = I[1][i]
				rgb = (chan1[j,k], chan2[j,k], chan3[j,k])
				rgbValues.append(rgb)
	
	print rgbValues 
		
		
							
			
def subtractTableAndSave(folder, outFolderImg = None, outFolderDep = None, thresh=20):
	debug = True
	
	if not outFolderImg:
		outFolderImg = os.path.join(folder.strip('/') + '-data', 'imgNoTable')
	if not outFolderDep:
		outFolderDep = os.path.join(folder.strip('/') + '-data', 'imgDepthNoTable')

	# Build static map and mask
	staticMap = util.buildMinMap(os.path.join(folder, 'table'))
	mult = (256*256)/staticMap.max()
	#imshow('map', staticMap*mult)

	mask = None
	try:
		mask = imread(os.path.join(folder, 'mask.png'))
		mask = mask[:,:,1]
		mask = np.uint8(mask == 0)*255
	except:
		mask = np.zeros(staticMap.shape, 'uint8')
		mask[35:389, 30:611] = 255

	print mask
	print mask.dtype, mask.shape
	#imshow('mask', mask)


	# Get all images in hand directory
	fnames = os.listdir(folder)
	fnames = [f for f in fnames if f.find('image_') >= 0]
	nums = [int(fname.split('_')[1]) for fname in fnames]
	n = max(nums)
	start = min(nums)


	for i in range(start, n):	
		print i
		img = imread(os.path.join(folder, 'image_'+str(i)+'_rgb.png'), -1)
		imgDepth = imread(os.path.join(folder, 'image_'+str(i)+'_dep.png'), -1)
		
		segImg, segImgDepth, fgMask = segmentAndMask(img, imgDepth, staticMap, mask, thresh)
		#imshow('segmented image', segImg)
		#imshow('segmented depth', segImgDepth*8)
		waitKey(1)
				
		imwrite(os.path.join(outFolderImg, 'image_'+str(i)+'_rgb.png'), segImg)
		imwrite(os.path.join(outFolderDep, 'image_'+str(i)+'_dep.png'), segImgDepth)





def cropImage(img, dep, mask):
	bw = mask.copy()
	bw = cv2.dilate(bw, None)
	bw = cv2.dilate(bw, None)
	bw = cv2.erode(bw, None)
	bw = cv2.erode(bw, None)

	cvimg = util.array2cv(np.uint8(bw)*255)	
	blobs = traceContoursCV2(np.uint8(bw))		
	blobs = [b for b in blobs if len(b) > 2]
	blobs = [b for b in blobs if util.blobsize(b, ignoreLessThan=600) > 600]
	
	if blobs:
		#for b in blobs: drawBlob(imgS, b, color=(255,0,0))
		#imshow('bw', np.uint8(bw)*255)
	
		blobs.sort(key=util.blobsize)
		blob = blobs[0]		
		roi = util.boundingRect(blob)
		
		padding = 30
		print roi
		x0 = max(0, roi[0]-padding)
		x1 = min(img.shape[1]-1, roi[0] + roi[2]+padding)
		y0 = max(0, roi[1]-padding)
		y1 = min(img.shape[0]-1, roi[1] + roi[3]+padding)
		
		cropped = img[y0:y1, x0:x1]
		#imshow('cropped', cropped)
	
		depCropped = dep[y0:y1, x0:x1]
		#imshow('depcropped', depCropped*10)
		
		maskCropped = bw[y0:y1, x0:x1]*255

		return cropped, depCropped, maskCropped

	else:
		return None, None, None




def subtractTableCropAndSave(folder, outFolderImg = None, outFolderDep = None, thresh=20):
	debug = True
	
	if not outFolderImg:
		outFolderImg = os.path.join(folder.strip('/') + '-data', 'imgNoTable')
	if not outFolderDep:
		outFolderDep = os.path.join(folder.strip('/') + '-data', 'imgDepthNoTable')

	# Build static map and mask
	staticMap = util.buildMinMap(os.path.join(folder, 'table'))
	mult = (256*256)/staticMap.max()
	#imshow('map', staticMap*mult)

	mask = None
	try:
		mask = imread(os.path.join(folder, 'mask.png'))
		mask = mask[:,:,1]
		mask = np.uint8(mask == 0)*255
	except:
		mask = np.zeros(staticMap.shape, 'uint8')
		mask[35:389, 30:611] = 255

	print mask
	print mask.dtype, mask.shape
	#imshow('mask', mask)


	# Get all images in hand directory
	fnames = os.listdir(folder)
	fnames = [f for f in fnames if f.find('image_') >= 0]
	nums = [int(fname.split('_')[1]) for fname in fnames]
	n = max(nums)
	start = min(nums)


	for i in range(start, n):	
		print i
		img = imread(os.path.join(folder, 'image_'+str(i)+'_rgb.png'), -1)
		imgDepth = imread(os.path.join(folder, 'image_'+str(i)+'_dep.png'), -1)

		segImg, segImgDepth, fgMask = segmentAndMask(img, imgDepth, staticMap, mask, thresh)
		cropImg, cropDepth, cropMask = cropImage(img, imgDepth, fgMask)
				
		waitKey(10)
		if not (cropImg == None):
			basename = "image_%05i_rgb_cropped.png" % i
			print os.path.join(outFolderImg, basename)
			imwrite(os.path.join(outFolderImg, basename), cropImg)
				
			basename = "image_%05i_dep_cropped.png" % i
			print os.path.join(outFolderDep, basename)
			imwrite(os.path.join(outFolderDep, basename), cropDepth)

			basename = "image_%05i_mask_cropped.png" % i
			print os.path.join(outFolderImg, basename)
			imwrite(os.path.join(outFolderImg, basename), cropMask)

		





def buildMask(folder):
	fnames = os.listdir(folder)
	num = 100
	imgPath = '%s/image_%i_rgb.png' % (folder, num)
	print imgPath
	
	depPath = '%s/image_%i_dep.png' % (folder, num)
	print depPath
	
	img = imread(imgPath)
	imgDepth = imread(depPath, -1)
	img2 = img.copy()
	img2[imgDepth <= 0, :] = 0
	
	#imshow('Click mask', img2)
	imshow('Click mask', imgDepth*15)

	pointQueue = []	
	def handleMouseClick(event, x, y, flags, parameters):
		if event == cv.CV_EVENT_LBUTTONDOWN:
			pointQueue.append((x,y))
			print pointQueue
		

	cv2.namedWindow("Click mask", 1)
	cv.SetMouseCallback("Click mask", handleMouseClick, None);

	key = 0	
	while (key % 256) != 32:			
		key = waitKey(10)
		
	print 'final point queue:', pointQueue				


	mask = np.ones((img.shape[0], img.shape[1]), dtype='uint8')*255
	cv2.fillPoly(mask, [np.array(pointQueue, dtype="int32")], (0,0,0))
	imshow('mask', mask)
	
	img2[mask > 0] = 0;
	imshow('masked img', img2)
	
	dep2 = imgDepth.copy()
	dep2[mask > 0] = 0;
	imshow('masked dep', dep2*10)
	
	waitKey(0)
	
	outpath = os.path.join(folder, 'mask.png')
	imwrite(outpath, mask)	
	print 'wrote', outpath


	
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

	
def testTraceContoursCV2():
	pass







if __name__ == '__main__':
	folder = sys.argv[1]
	
	staticMap = util.buildMinMap(os.path.join(folder, 'table'))
	mult = (256*256)/staticMap.max()
	#imshow('map', staticMap*mult)

	mask = None
	try:
		mask = imread(os.path.join(folder, 'mask.png'))
		mask = mask[:,:,1]
		mask = np.uint8(mask == 0)*255
	except:
		mask = np.zeros(staticMap.shape, 'uint8')
		mask[35:389, 30:611] = 255

	print mask
	print mask.dtype, mask.shape
	#imshow('mask', mask)


	# Get all images in hand directory
	fnames = os.listdir(folder)
	fnames = [f for f in fnames if f.find('image_') >= 0]
	nums = [int(fname.split('_')[1]) for fname in fnames]
	n = max(nums)
	start = min(nums)


	for i in range(start, n):	
		print i
		img = imread(os.path.join(folder, 'image_'+str(i)+'_rgb.png'), -1)
		imgDepth = imread(os.path.join(folder, 'image_'+str(i)+'_dep.png'), -1)
		
		imshow('inmask', mask)
		
		segImg, segImgDepth, fgMask = segmentAndMask(img, imgDepth, staticMap, mask, 20)
		imshow('segmented image', segImg)
		imshow('segmented depth', segImgDepth*8)
		waitKey(10)



def subtractTable(im, dep, tablemodel, mask, thresh=None):
    if thresh:
        return segmentAboveAndMask(im, dep, tablemodel, mask,thresh)
    else:
        return segmentAboveAndMask(im, dep, tablemodel, mask)
    



