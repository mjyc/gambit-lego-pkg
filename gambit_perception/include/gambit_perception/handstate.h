#ifndef HANDSTATE_H
#define HANDSTATE_H

// Classes
class HandState {
protected:
    cv::Mat handMask_;
    std::vector<cv::Point> contour_;

public:
    typedef boost::shared_ptr<HandState> Ptr;

    // TODO: test below code
//    HandState(cv::Mat handMask):
//        handMask_(handMask),
//        contour_(traceContours(handMask)[0])
//    {}
    HandState(std::vector<cv::Point> handContour, cv::Size size)
        : handMask_(contour2mask(handContour, size)),
          contour_(handContour)
    {}

    cv::Mat getHandMask() {
        return handMask_;
    }
};

#endif // HANDSTATE_H
