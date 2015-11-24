// Ported&Inspired from Jinna's oasis_svn_repo code
#ifndef HANDTRACKING_H
#define HANDTRACKING_H

#include <vector>
//#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gambit_perception/cv_utils.h>
#include <gambit_perception/cv_tableimgproc.h>
#include <gambit_perception/handstate.h>


// Global variables
static const float MIN_HANDSIZE_FRAC = 0.002;

static const int HUE_TARGET = 0;
static const int HUE_THRESH = 20;
static const int VAL_THRESH = 50;
static const int SAT_THRESH_LOW = 50;
static const int SAT_THRESH_HIGH = 150;


//// Classes
//class HandState {
//protected:
//    cv::Mat handMask_;
//    std::vector<cv::Point> contour_;

//public:
//    typedef boost::shared_ptr<HandState> Ptr;

//    // TODO: test below code
////    HandState(cv::Mat handMask):
////        handMask_(handMask),
////        contour_(traceContours(handMask)[0])
////    {}
//    HandState(std::vector<cv::Point> handContour, cv::Size size)
//        : handMask_(contour2mask(handContour, size)),
//          contour_(handContour)
//    {}

//    cv::Mat getHandMask() {
//        return handMask_;
//    }
//};


// Functions

// Super-set of hand masks
// NOTE: NOT-MASKED output image
cv::Mat superSetHandMask(cv::Mat dep, cv::Mat tableModel, cv::Mat mask, \
                      int table_noise_threshold = TABLE_NOISE_THRESHOLD, \
                      float min_handsize_frac = MIN_HANDSIZE_FRAC) {
    cv::Mat depthMask = subtractTableSimple(dep,tableModel,table_noise_threshold);
    // cv::imshow("fgMask",depthMask);
    return crossedContoursMask(depthMask, mask, min_handsize_frac);
}

void getColorMasks(cv::Mat img, cv::Mat& maskLow, cv::Mat& maskHigh, \
                   int hue_target = HUE_TARGET, \
                   int hue_thresh = HUE_THRESH, \
                   int val_thresh = VAL_THRESH, \
                   int sat_thresh_low = SAT_THRESH_LOW, \
                   int sat_thresh_high = SAT_THRESH_HIGH) {

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat hue(hsv.size(), hsv.depth());
    cv::Mat sat(hsv.size(), hsv.depth());
    cv::Mat val(hsv.size(), hsv.depth());

    cv::Mat out[] = {hue, sat, val};
    int fromTo[] = { 0,0, 1,1, 2,2 };
    mixChannels( &hsv, 1, out, 3, fromTo, 3 );

    cv::Mat valMask = val > val_thresh;
    cv::Mat satMaskLow = sat > sat_thresh_low;
    cv::Mat satMaskHigh = sat > sat_thresh_high;

    cv::Mat hueDiff1 = cv::abs(hue - hue_target);
    cv::Mat hueDiff2 = cv::abs((hue - 180) - hue_target);
    cv::Mat hueDiff; cv::min(hueDiff1, hueDiff2, hueDiff);

    cv::Mat hueMask = hueDiff < hue_thresh;
    maskLow = valMask & satMaskLow & hueMask;
    maskHigh = valMask & satMaskHigh & hueMask;
}

// WARN: empties the "dstHandRegions" vector first.
// NOTE: MASKED output image
void getHandMaskHSV(cv::Mat img, cv::Mat dep, cv::Mat tableModel, cv::Mat mask, \
                    std::vector< std::vector<cv::Point> >& dstHandRegions, \
                    cv::Mat& dstHandMask, \
                    int table_noise_thresh = TABLE_NOISE_THRESHOLD, \
                    float min_handsize_frac = MIN_HANDSIZE_FRAC) {
    if (mask.type() != CV_8UC1) {
        std::cerr << "gethandMaskHSV: mask.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_8UC1 << ", got " << mask.type() << std::endl;
        exit(-1);
    }

    int vidArea = mask.size().area();
    cv::Mat colorMaskLow, colorMaskHigh;
    getColorMasks(img, colorMaskLow, colorMaskHigh);
    cv::Mat depthMask = (tableModel - dep) > table_noise_thresh;
    depthMask.convertTo(depthMask,CV_8UC1);
    depthMask = crossedContoursMask(depthMask, mask) & mask; // MASK!
    //cv::imshow("after crossed", depthMask);

    // Initial filtering with colorMaskLow
    std::vector< std::vector<cv::Point> > contours = traceContours(depthMask & colorMaskLow);
    //cv::imshow("contours", contours2mask(contours,mask.size()));
    dstHandRegions.empty();
    dstHandMask = cv::Mat(mask.size(), CV_8UC1);
    BOOST_FOREACH(std::vector<cv::Point> contour, contours) {
        cv::RotatedRect rect = cv::minAreaRect(cv::Mat(contour));
        // Filtering conditions
        if (contour.size() > 10 && \
                rect.size.area() >= min_handsize_frac * vidArea) {
            dstHandRegions.push_back(contour);
            cv::Mat regionsMask = contour2mask(contour, mask.size());
            // Using colorMaskHigh as selector here
            if (cv::countNonZero(regionsMask & colorMaskHigh) > 0) {
                dstHandMask = dstHandMask | regionsMask;
            }
        }
    }
    //cv::imshow("handMask", dstHandMask);
}

// IDEA: improve handState assignment?
// IDEA: instead of creating HandState object all the time, create pointer to handStates and update them?
void findHands(cv::Mat img, cv::Mat dep, cv::Mat tableModel, cv::Mat mask, \
               std::vector<HandState>& hands, \
               int table_noise_threshold = TABLE_NOISE_THRESHOLD, \
               float min_handsize_frac = MIN_HANDSIZE_FRAC) {
    cv::Mat superHandMask = superSetHandMask(dep, tableModel, mask, table_noise_threshold);
    //cv::imshow("superHandMask", superHandMask);
    //std::cout << "cv::countNonZero(superHandMask) = " << cv::countNonZero(superHandMask) << std::endl;
    if (cv::countNonZero(superHandMask) <= 0)
        return;

    cv::Mat handMask;
    std::vector< std::vector<cv::Point> > handContours;
    getHandMaskHSV(img,dep,tableModel,mask, handContours, handMask, min_handsize_frac);

    // Simplest handState assignment
    hands.empty();
    unsigned int maxNumHand = 2;
    for (size_t i = 0; i < handContours.size(); ++i) {
        if (i == maxNumHand)
            break;
        std::vector<cv::Point> handContour = handContours[i];
        hands.push_back(HandState(handContour,handMask.size()));
    }
}

#endif // HANDTRACKING_H
