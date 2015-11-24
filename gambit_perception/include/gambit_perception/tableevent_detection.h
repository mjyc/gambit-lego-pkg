// Ported&Inspired from Jinna's oasis_svn_repo code
#ifndef DETECTION_H
#define DETECTION_H

#include <string>
#include <boost/make_shared.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gambit_perception/cv_tableimgproc.h>
#include <gambit_perception/handtracking.h>


// Global variables
static const float MIN_OBJSIZE_FRAC = 0.003;
static const int OBJ_REMOVAL_THRESH = 10;


// Funcitons

bool isObjectRemovedROI(cv::Mat objDepROI, cv::Mat curDepROI, \
                        int removalAreaThresh, \
                        int obj_removal_thresh = OBJ_REMOVAL_THRESH) {
    cv::Mat underMask = curDepROI > (objDepROI + obj_removal_thresh);
    underMask.convertTo(underMask,CV_8UC1);
    //cv::imshow("underMask",underMask*50);

    int areaUnder = cv::countNonZero(underMask);
    return areaUnder > removalAreaThresh;
}

bool isObjectRemoved(cv::Mat dep, cv::Mat mask, cv::Mat objDep, cv::Rect roi,
       float min_objsize_frac = MIN_OBJSIZE_FRAC,
       int obj_removal_thresh = OBJ_REMOVAL_THRESH) {
    cv::Mat croppedMask = cv::Mat(mask, roi).clone();;
    cv::Mat expectedROI = cv::Mat(objDep, roi);
    cv::Mat actualROI; cv::Mat(dep, roi).copyTo(actualROI,croppedMask);

    // Use half of the object pixels as a threshold
    int removalAreaThresh = .5*min_objsize_frac*dep.size().area();
    return isObjectRemovedROI(expectedROI, actualROI, \
                       removalAreaThresh, obj_removal_thresh);
}

std::vector< std::vector<cv::Point> >
findObjectContours(cv::Mat dep, cv::Mat tableModel, cv::Mat mask,
               int table_noise_threshold = TABLE_NOISE_THRESHOLD, \
               float min_objsize_frac = MIN_OBJSIZE_FRAC) {

    cv::Mat fgMask = subtractTableSimple(dep, tableModel, table_noise_threshold);
    // cv::imshow("fgMask", fgMask);
    fgMask &= ~superSetHandMask(dep, tableModel, mask); // MASK!
    //cv::imshow("superSetHandMask", ~superSetHandMask(dep, tableModel, mask));
    //cv::imshow("fgMask2", fgMask);

    float vidArea = mask.size().area();
    std::vector< std::vector<cv::Point> > contours = traceContours(fgMask  & mask);
    //cv::imshow("contours", contours2mask(contours,mask.size()));
    std::vector< std::vector<cv::Point> > objContours;
    BOOST_FOREACH(std::vector<cv::Point> contour, contours) {
        cv::RotatedRect rotRect = cv::minAreaRect(cv::Mat(contour));
        // Size filtering
        if (contour.size() > 2 && \
                rotRect.size.area() >= min_objsize_frac * vidArea) {
            objContours.push_back(contour);
        }
    }
    //cv::imshow("objects",contours2mask(objContours,mask.size()));
    return objContours;
}


#endif // DETECTION_H
