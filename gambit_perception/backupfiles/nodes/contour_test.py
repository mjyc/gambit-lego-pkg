import cv2.cv as cv
import cv2; from cv2 import imshow, imread, imwrite, waitKey
import numpy as np

import util2
import handtracking

if __name__=="__main__":
    img = imread("img.png",-1)
    dep = imread("dep.png",-1)
    mask = imread("mask.png",-1)

    # imshow("mask",mask*255)
    # waitKey()
   
    contours, trash = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnt = contours[0]
    rect = cv2.minAreaRect(cnt)
    box = cv2.cv.BoxPoints(rect)
    print rect
    print np.int16(box)

    pts = np.int16(box)
    print pts
    for j in range(len(pts)):
        cv2.line( img, tuple(pts[j]), tuple(pts[(j+1)%4]), (255,255,255))

    # Point2f rect_points[4]; minRect[i].points( rect_points );
    #    for( int j = 0; j < 4; j++ )
    #       line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );


    # box = cv2.cv.BoxPoints(rect)
    # box = np.int8(box)
    # cv2.drawContours(img,[box],0,(0,0,255),2)
    imshow("box",img)
    # waitKey()


    waitKey()
