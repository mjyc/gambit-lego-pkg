#ifndef TABLEOBJECT_H
#define TABLEOBJECT_H

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
//#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/centroid.h>
#include <tf/transform_listener.h>

#include <gambit_perception/cv_utils.h>
#include <gambit_perception/tableevent_detection.h>
#include <gambit_perception/tf_utils.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


// TODO:
// 1. Better color detection
// 2. Clearly finish up Coordinate Conversions
// 3. How to organize Lego and NonLego block stuff
// 4. Relation update part is such a mass... please fix that..



// snap color to rgb
void sanpColor2RGB(const PointCloud::Ptr& cloudPtr,
                   float& rout, float& gout, float& bout) {
    double r = 0.0; double g = 0.0; double b = 0.0;
    for(size_t i=0; i<cloudPtr->points.size(); i++) {
        if (cloudPtr->points[i].z == cloudPtr->points[i].z) {
            r += cloudPtr->points[i].r;
            g += cloudPtr->points[i].g;
            b += cloudPtr->points[i].b;
        }
    }

    rout = gout = bout = 0.0;
    double maxval = std::max(std::max(r,g),b);
    if (maxval == r)
        rout = 1.0;
    else if (maxval == g)
        gout = 1.0;
    else
        bout = 1.0;
}


// Global variables
static int NUM_OBJECTS = 0;


class TableObject {
public: // Forward declaration required
    enum loc_state_t {
        ONTABLE = 0,
        OFFTABLE = 2,
        UNKNOWN = -1
    };

protected:
    TableParams params_;
    image_geometry::PinholeCameraModel* model_; // dont' modify
    tf::TransformListener listener_;

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

    Eigen::Vector4f centroid_;
    Eigen::Vector4f centroid2_;
    Eigen::Vector4f color_;

    virtual void setImages(cv::Mat img, cv::Mat dep, \
                           std::vector<cv::Point> contour);

public:
    typedef boost::shared_ptr<TableObject> Ptr;

    double r,g,b;  // random color identifier

    static std::string getNextObjectName() {
        return "obj" + boost::lexical_cast<std::string>(NUM_OBJECTS++);
    }

    TableObject(std::string name, loc_state_t locState, \
                cv::Mat img, cv::Mat dep, \
                std::vector<cv::Point> contour, \
                TableParams params,
                image_geometry::PinholeCameraModel* model)
        : name_(name),
          params_(params),
          model_(model),
          r(((double)rand())/RAND_MAX),
          g(((double)rand())/RAND_MAX),
          b(((double)rand())/RAND_MAX)
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

    // Convert object part to PointCloud with external model parameters
    void toCloud(const PointCloud::Ptr& cloudMsg) {
        toPointCloud(croppedTightImg_,croppedTightDep_, \
                     model_->fx(),model_->fy(), \
                     model_->cx()-rect_.x,model_->cy()-rect_.y, \
                     cloudMsg);
    }

    // Get tight four bounding points (based on rotRect_) with external model parameters
    void boundingPoints(const PointCloud::Ptr& out) {
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
                                                model_->fx(),model_->fy(), \
                                                model_->cx(),model_->cy()));
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

    Eigen::Vector4f getCentroid() {
        return centroid_;
    }

    Eigen::Vector4f getCentroid2() {
        return centroid2_;
    }

    Eigen::Vector4f getColor() {
        return color_;
    }
};

void TableObject::setImages(cv::Mat img, cv::Mat dep, \
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

    PointCloud::Ptr cloudPtr(new PointCloud);
    toCloud(cloudPtr);
    pcl::compute3DCentroid(*cloudPtr,centroid_);
    pcl::compute3DCentroid(*cloudPtr,centroid2_);

    sanpColor2RGB(cloudPtr,color_[0],color_[1],color_[2]);

    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(0.151, 0.244, 0.780));
    transform.setRotation(tf::Quaternion(-0.625, 0.747, -0.221, 0.052));

//    try{
//        listener_.lookupTransform("/arm0","/camera_rgb_optical_frame",ros::Time(0),transform);
//    }
//    catch (tf::TransformException ex){
//        ROS_ERROR("%s",ex.what());
//    }
    Eigen::Matrix4f AffineTransform;
    tfToEigen(transform,AffineTransform);
    centroid_ = AffineTransform * centroid_;
    transformPointCloud(*cloudPtr, *cloudPtr, transform);
}




class LegoTableObject : public TableObject {
public: // Forward declaration required

    enum pred_colo { // involve 1 literal
        RED = 0,
        GREEN = 1,
        BLUE = 2,
        YELLOW = 3
    };

    enum pred_loc_x { LEFT = 0, RIGHT = 1 };
    enum pred_loc_y { BOTTOM = 0, TOP = 1 };
    enum pred_loc_z { BELOW = 0, ABOVE = 1 };

//    typedef boost::tuple<pred_loc_x, pred_loc_y, pred_loc_z> Tuple;

    typedef boost::shared_ptr<LegoTableObject> Ptr;

protected:

    std::map<std::string,LegoTableObject::Ptr> relObj;
    std::map< std::string, pred_loc_x > relSymX;
    std::map< std::string, pred_loc_y > relSymY;
    std::map< std::string, pred_loc_z > relSymZ;

public:

    LegoTableObject(std::string name, loc_state_t locState, \
                    cv::Mat img, cv::Mat dep, \
                    std::vector<cv::Point> contour, \
                    TableParams params,
                    image_geometry::PinholeCameraModel* model)
        : TableObject(name,locState,img,dep,contour,params, model)
    {
        setLocState(locState);
        setImages(img,dep,contour);
    }

    void getRelativeProperty(LegoTableObject::Ptr other) {

        relObj[other->getName()] = other;

        Eigen::Vector4f ocentroid = other->getCentroid();
        if ((centroid_[0] - ocentroid[0]) < 0)
            relSymX[other->getName()] = LegoTableObject::LEFT;
        else
            relSymX[other->getName()] = LegoTableObject::RIGHT;

        if ((centroid_[1] - ocentroid[1]) < 0)
            relSymY[other->getName()] = LegoTableObject::BOTTOM;
        else
            relSymY[other->getName()] = LegoTableObject::TOP;

        if ((centroid_[2] - ocentroid[2]) < 0)
            relSymZ[other->getName()] = LegoTableObject::BELOW;
        else
            relSymZ[other->getName()] = LegoTableObject::ABOVE;

        if (relSymX[other->getName()]) {
            std::cout << this->getName() << " is RIGHT OF " << other->getName() << std::endl;
        } else
            std::cout << this->getName() << " is LEFTOF " << other->getName() << std::endl;

        if (relSymY[other->getName()]) {
            std::cout << this->getName() << " is TOP OF " << other->getName() << std::endl;
        } else
            std::cout << this->getName() << " is BOTTOM OF " << other->getName() << std::endl;

        if (relSymZ[other->getName()]) {
            std::cout << this->getName() << " is ABOVE OF " << other->getName() << std::endl;
        } else
            std::cout << this->getName() << " is BELOW OF " << other->getName() << std::endl;

    }



//    }

};

#endif // TABLEOBJECT_H





