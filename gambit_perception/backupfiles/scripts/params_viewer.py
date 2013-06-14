import cv2.cv as cv
import cv2; from cv2 import imshow, imread, imwrite, waitKey
import numpy as np

if __name__=="__main__":
    mask = imread('params/mask.png', -1)
    baseTable = imread('params/baseTable.png', -1)

    imshow("mask", mask*10)
    imshow("baseTable", baseTable*10)
    waitKey()
