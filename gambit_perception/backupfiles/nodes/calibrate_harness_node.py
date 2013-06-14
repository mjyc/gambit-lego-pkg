#!/usr/bin/env python

import os
import numpy as np

import roslib; roslib.load_manifest("gambit_perception")
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2.cv as cv
import cv2
import message_filters

from interactivetools import interactiveMask, interactiveLines, interactiveDisplayArea, interactiveBaseTable
from util3 import downsample

if __name__ == "__main__":

    br = CvBridge()
    
    iBaseTable = interactiveBaseTable((320, 240), 20)

    def callback(imgmsg,depmsg):
        image = br.imgmsg_to_cv(imgmsg, "bgr8")
        depth = br.imgmsg_to_cv(depmsg, "32FC1")
        fullimg = np.asarray(image[:,:], dtype="uint8")
        fulldep = np.uint16( np.asarray(depth[:,:], dtype="float32") * 1000 ) # m -> mm
        (img,dep) = downsample(fullimg,fulldep,factor=2)

        if iBaseTable.ind < iBaseTable.nTableFrames:
            iBaseTable.setDep(dep)
        else:
            iBaseTable.computeMinMap()
            rospy.signal_shutdown("")

        mask = interactiveMask((img,dep))
        plines = interactiveLines((img,dep), mask)
        displayArea = interactiveDisplayArea((img,dep), mask)
        flatTable = interactiveBaseTable((img,dep), (320, 240), 20)
    
    
    rospy.init_node("calibrate_harness")
    img_sub = message_filters.Subscriber("/camera/rgb/image_rect_color_sync", sensor_msgs.msg.Image)
    dep_sub = message_filters.Subscriber("/camera/depth_registered/image_rect_sync", sensor_msgs.msg.Image)

    queue_size = 5;
    ts = message_filters.TimeSynchronizer([img_sub,dep_sub],queue_size)
    ts.registerCallback(callback)

    print "[WARN] Don't forget to launch \"convert_pointcloud_imgdep.launch\"."
    rospy.spin()
