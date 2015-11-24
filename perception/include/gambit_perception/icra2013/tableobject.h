#ifndef TABLEOBJECT_H
#define TABLEOBJECT_H

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <gambit_perception/cv_utils.h>
#include <gambit_perception/tableevent_detection.h>


// Global variables
static int NUM_OBJECTS = 0;


class TableObject {
public:
    enum loc_state_t {
        LEFT = 0,
        RIGHT = 1,
        OFFTABLE = 2,
        UNKNOWN = -1
    };

protected:
    TableParams params_;


    std::string name_;
    bool offTable_;
    loc_state_t locState_;


    std::vector<cv::Point> contour_;
    cv::Rect rect_;
    cv::RotatedRect rotRect_;

    cv::Mat img_;              // full images
    cv::Mat dep_;
    cv::Mat croppedImg_;       // rect cropped images
    cv::Mat croppedDep_;
    cv::Mat croppedMask_;      // rect mask
    cv::Mat croppedTightImg_;  // contour cropped images
    cv::Mat croppedTightDep_;

    void setImages(cv::Mat img, cv::Mat dep, \
                   std::vector<cv::Point> contour) {
        contour_ = contour;
        rect_ = cv::boundingRect(cv::Mat(contour_));
        rotRect_ = cv::minAreaRect(cv::Mat(contour_));

        img_ = img.clone();
        dep_ = dep.clone();

        croppedImg_ = cv::Mat(img, rect_).clone();
        croppedDep_ = cv::Mat(dep, rect_).clone();
        cv::Mat mask = contour2mask(contour_,dep.size());
        croppedMask_ = cv::Mat(mask, rect_).clone();

        croppedDep_.copyTo(croppedTightDep_,croppedMask_);
        croppedImg_.copyTo(croppedTightImg_,croppedMask_);
    }

public:
    typedef boost::shared_ptr<TableObject> Ptr;

    static std::string getNextObjectName() {
        return "obj" + boost::lexical_cast<std::string>(NUM_OBJECTS++);
    }

    TableObject(std::string name, loc_state_t locState, \
                cv::Mat img, cv::Mat dep, \
                std::vector<cv::Point> contour, \
                TableParams params)
        : name_(name),
          params_(params)
    {
        setLocState(locState);
        setImages(img,dep,contour);
    }

    virtual ~TableObject() {}

    // Update variables related to images
    void updateImages(cv::Mat img, cv::Mat dep, std::vector<cv::Point> contour) {
        setImages(img,dep,contour);
    }

    // Check if the object is removed
    bool isRemoved(cv::Mat dep) {
        cv::Mat expectedROI = croppedDep_;
        cv::Mat actualROI; cv::Mat(dep,rect_).copyTo(actualROI,croppedMask_);

        // Use half of the object pixels as a threshold
        int removalAreaThresh = .5*params_.min_objsize_frac*dep.size().area();
        return isObjectRemovedROI(expectedROI,actualROI,removalAreaThresh,params_.obj_removal_thresh);
    }

    // Convert object part to PointCloud
    void toCloud(float fx, float fy, float centerX, float centerY,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudMsg) {
        toPointCloud(croppedTightImg_,croppedTightDep_,fx,fy,centerX-rect_.x,centerY-rect_.y,cloudMsg);
    }

    void toLocStateCloud(float fx, float fy, float centerX, float centerY,
                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudMsg) {

        if (locState_ == LEFT || locState_ == RIGHT) {
            cv::Mat croppedColoredMask(croppedTightImg_.rows, croppedTightImg_.cols, croppedTightImg_.type());

            if (locState_ == LEFT) {  // LEFT
                for(int i=0; i<croppedTightImg_.rows; i++)
                    for(int j=0; j<croppedTightImg_.cols; j++) {
                        croppedColoredMask.at<cv::Vec3b>(i,j)[0] = 255;
                        croppedColoredMask.at<cv::Vec3b>(i,j)[1] = 0;
                        croppedColoredMask.at<cv::Vec3b>(i,j)[2] = 0;
                    }
            } else {                  // RIGHT
                for(int i=0; i<croppedTightImg_.rows; i++)
                    for(int j=0; j<croppedTightImg_.cols; j++) {
                        croppedColoredMask.at<cv::Vec3b>(i,j)[0] = 0;
                        croppedColoredMask.at<cv::Vec3b>(i,j)[1] = 0;
                        croppedColoredMask.at<cv::Vec3b>(i,j)[2] = 255;
                    }
            }

            toPointCloud(croppedColoredMask,croppedTightDep_,fx,fy,centerX-rect_.x,centerY-rect_.y,cloudMsg);
        }
    }

    // Get tight four bounding points (based on rotRect_)
    void boundingPoints(float fx, float fy, float centerX, float centerY, \
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out) {
        // get points
        cv::Point2f points[4];
        rotRect_.points(points);
        // compute 4-bounding points
        BOOST_FOREACH(cv::Point pt, points) {
            // quick hack to avoid invalid depth
            int colInd = pt.x;
            uint16_t depth = dep_.at<uint16_t>(pt.y,colInd);
            while (depth == 0) {
                depth = dep_.at<uint16_t>(pt.y,colInd);
                colInd++;
            }

            out->points.push_back(toPointXYZRGB(pt.x,pt.y,depth, \
                                                255,255,255,\
                                                fx, fy, centerX, centerY));
        }
    }


    // Getters and Setters

    std::string getName() {
        return name_;
    }

    bool getOffTable() {
        return offTable_;
    }

    void setOffTable(bool offTable) {
        offTable_ = offTable;
        if (offTable_ == true)
            locState_ = OFFTABLE;
    }

    loc_state_t getLocState() {
        return locState_;
    }

    void setLocState(loc_state_t locState) {
        locState_ = locState;
        if (locState_ == OFFTABLE)
            offTable_ = true;
        else
            offTable_ = false;
    }

    cv::Mat getDep() {
        return dep_;
    }

    cv::Rect getROI() {
        return rect_;
    }
};

#endif // TABLEOBJECT_H





