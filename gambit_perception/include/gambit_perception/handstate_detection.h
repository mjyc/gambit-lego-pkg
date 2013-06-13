#ifndef HAND_DETECTOR_CPP
#define HAND_DETECTOR_CPP

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gambit_perception/handtracking.h>

class HandStateDetector {
private:
    int table_noise_threshold_;
    float min_handsize_frac_;

    cv::Mat img_;
    cv::Mat dep_;
    cv::Mat mask_;
    cv::Mat tableModel_;

    std::vector<HandState> handStates_;

public:
    HandStateDetector(cv::Mat mask, cv::Mat tableModel, int table_noise_threshold, float min_handsize_frac)
        : mask_(mask),
          tableModel_(tableModel),
          table_noise_threshold_(table_noise_threshold),
          min_handsize_frac_(min_handsize_frac)
    {}

    void setImage(cv::Mat img, cv::Mat dep) {
        img_ = img;
        dep_ = dep;
        // Here I can do better things.
        findHands(img_,dep_,tableModel_,mask_,handStates_,table_noise_threshold_,min_handsize_frac_);
    }
};

#endif // HAND_DETECTOR_CPP
