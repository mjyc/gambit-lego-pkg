import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cv2.cv as cv
import cv2
from cv2 import imread, imwrite, imshow, waitKey

import util2 as util
from util3 import downsample

#kinect_focal_length = 570.34
kinect_focal_length = 525.0

def toPointXYZ(x,y,d,height,width):
   # center_x = (width-1) / 2.0
   # center_y = (height-1) / 2.0
   # center_x = (640-1) / 2.0
   # center_y = (480-1) / 2.0
   center_x = (640/2-1) / 2.0
   center_y = (480/2-1) / 2.0
   f =  kinect_focal_length
   pointCloud = []
   ind = 0
   v = y
   u = x
   depth = d * 0.001
   if depth == 0:
      return (0,0,0)
   px = (u - center_x) * depth / f
   py = (v - center_y) * depth / f
   pz = depth

   return (px,py,pz)

def toPointCloud(img, dep):
   print dep.shape

   height,width = dep.shape
   # center_x = (width-1) / 2.0
   # center_y = (height-1) / 2.0
   # center_x = (640-1) / 2.0
   # center_y = (480-1) / 2.0
   center_x = (640/2-1) / 2.0
   center_y = (480/2-1) / 2.0
   f =  kinect_focal_length
   pointCloud = []
   ind = 0
   for v in range(0,height):
      for u in range(0,width):
         depth = dep[v,u] * 0.001
         if depth == 0:
            continue
         px = (u - center_x) / f * depth
         py = (v - center_y) / f * depth
         pz = depth
         pointCloud.append([px,py,pz,img[v,u,2],img[v,u,1],img[v,u,0]])
         ind += 1

   return pointCloud

def scatterPointCloud(axis,cloud,factor=1):
   color = [tuple(pt) for pt in (cloud[0:len(cloud):factor,3:6]/256.0)]
   axis.scatter(cloud[0:len(cloud):factor,0], \
                   cloud[0:len(cloud):factor,1], \
                   cloud[0:len(cloud):factor,2], \
                   c=color)

def transformCloud(cloud,Tcw):
   augCloud = np.hstack((np.array(cloud)[:,0:3],np.ones((len(cloud),1))))
   newCloud = np.hstack((np.dot(Tcw,augCloud.T).T[:,0:3], np.array(cloud)[:,3:6]))
   return newCloud

if __name__ == "__main__":
   Tcw = np.genfromtxt("params/Tcw.txt")
   fullimg = imread("img.png",-1)
   fulldep = imread("dep.png",-1)
   #imshow("fullimg", fullimg)
   #waitKey(0)
   #fulldep = imread('dep.png',-1)
   (img,dep) = downsample(fullimg,fulldep,factor=2)
   #img = fullimg
   #dep = fulldep
   print img.shape

   cloud = toPointCloud(img,dep)
   newCloud = transformCloud(cloud,Tcw)

   fig = plt.figure()
   axis = fig.add_subplot(111, projection='3d')
   scatterPointCloud(axis,newCloud,20)
   #axis.scatter(0.35857489, -0.02256214, -0.24087212, c="r")
   #axis.scatter(0.23810774,  0.16704236,  0.18944692, c="r")
   plt.show()
