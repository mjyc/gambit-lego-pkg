#!/usr/bin/env python

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cv2.cv as cv
import cv2
from cv2 import imshow, imwrite, waitKey, destroyWindow

import roslib; roslib.load_manifest("gambit_perception")
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2.cv as cv
import cv2
import message_filters
import tf

from interactivetools import interactiveMask, interactiveLines, interactiveDisplayArea, interactiveBaseTable
from util3 import downsample
import util2

if __name__ == "__main__":

    br = CvBridge()
    listener = tf.TransformListener()

    def callback(imgmsg,depmsg):
        print imgmsg.header
        print depmsg.header
        image = br.imgmsg_to_cv(imgmsg, "bgr8")
        depth = br.imgmsg_to_cv(depmsg, "32FC1")
        fullimg = np.asarray(image[:,:], dtype="uint8")
        fulldep = np.uint16( np.asarray(depth[:,:], dtype="float32") * 1000 ) # m -> mm
        imwrite("img.png", fullimg)
        imwrite("dep.png", fulldep)
        (img,dep) = downsample(fullimg,fulldep,factor=2)

        now = rospy.Time(0)
        (trans,rot) = listener.lookupTransform("/camera_depth_optical_frame","/arm0",now)
        Tcw = listener.fromTranslationRotation(trans, rot)

        np.savetxt("Tcw.txt",Tcw)
        print Tcw

        #ii,jj = np.nonzero(depth)
        #dd = depth[ii, jj]

        #wps = util2.worldPoints(ii,jj,dd,Tcw);
        #print len(wps)
        #print wps[0].shape, wps[1].shape
        #scatter.scatter(wps[0], wps[1])
        #plt.show()

        #np.savetxt( "dep.txt", fulldep, "%d" );

        #np.savetxt( "img.txt", fullimg, fmt="%.4e" );
        #np.savetxt( "dep.txt", fulldep, "%d" );

        # mask = interactiveMask((img,dep))
        # plines = interactiveLines((img,dep), mask)
        # displayArea = interactiveDisplayArea((img,dep), mask)
        # flatTable = interactiveBaseTable((img,dep), (320, 240), 20)
        
        rospy.signal_shutdown("")

    rospy.init_node("test_node")
    img_sub = message_filters.Subscriber("/camera/rgb/image_rect_color_sync", sensor_msgs.msg.Image)
    dep_sub = message_filters.Subscriber("/camera/depth_registered/image_rect_sync", sensor_msgs.msg.Image)

    queue_size = 5;
    ts = message_filters.TimeSynchronizer([img_sub,dep_sub],queue_size)
    ts.registerCallback(callback)

    print "[WARN] Don't forget to launch \"convert_pointcloud_imgdep.launch\"."
    rospy.spin()
