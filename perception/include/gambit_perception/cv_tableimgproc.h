// Ported&Inspired from Jinna's oasis_svn_repo code
#ifndef CV_TABLEIMGPROC_H
#define CV_TABLEIMGPROC_H

#include <ctime>
#include <string>
#include <list>
#include <vector>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gambit_perception/cv_utils.h>


// Global variables
static const int TABLE_NOISE_THRESHOLD = 5;


struct TableParams {
    int table_noise_threshold;
    float min_objsize_frac;
    int obj_removal_thresh;
};


// Functions
cv::Mat subtractTableSimple(cv::Mat dep, cv::Mat tableModel, \
                            int table_noise_thresh = TABLE_NOISE_THRESHOLD) {
    cv::Mat depthMask = (tableModel - dep) > table_noise_thresh;
    depthMask.convertTo(depthMask,CV_8UC1);
    return depthMask;
}

// @param dep
// @param roi
// @return tableModel
void augmentTableModel(cv::Mat dep, cv::Rect roi, cv::Mat& tableModel) {
    //cv::imshow("before aug",tableModel*10);

    //unsigned int big = 65535;
    cv::Mat depROI = cv::Mat(dep,roi);
    cv::Mat tableROI = cv::Mat(tableModel,roi);
    //cv::imshow("depROI",dep);
    tableROI = cv::min(tableROI, depROI);
    //cv::imshow("tableROI",tableROI);

    //cv::imshow("after aug",tableModel*10);
}

void cutTableModel(cv::Mat baseTable, cv::Rect roi, cv::Mat&tableModel) {
    //cv::imshow("before cut",tableModel*10);

    unsigned int big = 65535;
    cv::Mat baseROI = cv::Mat(baseTable,roi);
    cv::Mat tableROI = cv::Mat(tableModel,roi);
    patchValue16UC1(baseROI,0,big);
    patchValue16UC1(tableROI,0,big);
    tableROI = cv::max(tableROI, baseROI);
    patchValue16UC1(tableModel,big,0);

    //cv::imshow("after cut",tableModel*10);
}


// Classes
class InteractiveMask {
private:
    static const unsigned int POINT_BUFFER_SIZE = 4;

    std::list<cv::Point> pointBuffer_;
    cv::Mat dep_;
    cv::Mat img_;
    cv::Mat mask_;
    std::string params_path_;

    // For OpenCV GUI
    static InteractiveMask *instance_;
    void setInstance() {
        //std::cout << "[InteractiveMask] instance set" << std::endl;
        instance_ = this;
    }

public:
    typedef boost::shared_ptr<InteractiveMask> Ptr;
    InteractiveMask(std::string params_path)
        : params_path_(params_path)
    {}

    bool getMask(cv::Mat img, cv::Mat dep) {
        img_ = img;
        dep_ = dep;
        cv::imshow("Click 4 points", img_);
        cv::imshow("depth map", dep_*10);
        cvSetMouseCallback("Click 4 points", &InteractiveMask::mouseClickCallbackWrapper, 0);
        setInstance();

        std::cout << "[imask] Click 4 corners for the workspace mask" << std::endl;
        std::cout << "[imask] Make sure 2nd and 3rd points create a line that crosses with hands" << std::endl;
        std::cout << "[imask] Press SPACE to quit" << std::endl;
        bool retval = false;
        int key = cv::waitKey(1) % 256;
        while ((unsigned char) key != ' ') {
            if (pointBuffer_.size() == POINT_BUFFER_SIZE && key == (int) 's') {
                cv::imwrite(params_path_ + "mask.png", mask_);
                std::cout << "[mask] Saving current mask to file: " + params_path_ + "mask.png" << std::endl;
                retval = true;
                break;
            }
            key = cv::waitKey(1) % 256;
        }

        cv::destroyWindow("Click 4 points");
        cv::destroyWindow("depth map");
        return retval;
    }

    void mouseClickCallback(int event, int x, int y, int flags, void* param) {
        if (event == CV_EVENT_LBUTTONDOWN) {
            cv::Point pt(x,y);

            pointBuffer_.push_back(pt);
            while (pointBuffer_.size() > POINT_BUFFER_SIZE)
                pointBuffer_.pop_front();

            if (pointBuffer_.size() == POINT_BUFFER_SIZE) {
                std::vector<cv::Point> pts(pointBuffer_.size());
                std::copy(pointBuffer_.begin(), pointBuffer_.end(), pts.begin());
                std::vector< std::vector<cv::Point> > ppt; ppt.push_back(pts);

                cv::Mat mask = cv::Mat::zeros(dep_.rows, dep_.cols, CV_8UC1);

                cv::fillPoly(mask, ppt, cv::Scalar(255,255,255));
                cv::imshow("mask", mask);

                // for debugging
                cv::Mat maskedBgrImg;
                cv::Mat maskedDepthImg;

                img_.copyTo(maskedBgrImg, mask);
                dep_.copyTo(maskedDepthImg, mask);

                cv::imshow("Click 4 points", maskedBgrImg);
                cv::imshow("depth map", maskedDepthImg*10);
                mask_ = mask;
            }
        }
    }

    static void mouseClickCallbackWrapper(int event, int x, int y, int flags, void* param) {
        instance_->mouseClickCallback(event, x, y, flags, param);
    }
};
InteractiveMask *InteractiveMask::instance_ = NULL;

class BaseTableLearner {
private:
    static const size_t DEPTH_BUFFER_SIZE = 60;

    std::vector<cv::Mat> depthBuffer_;
    cv::Mat dep_;
    cv::Mat img_;
    std::string params_path_;

public:
    typedef boost::shared_ptr<BaseTableLearner> Ptr;
    BaseTableLearner(std::string params_path)
        : params_path_(params_path)
    {}

    // @returns: true if finished computing, false if not finished computing
    bool computeBaseTable(cv::Mat img, cv::Mat dep) {
        img_ = img;
        dep_ = dep;

        bool retval = false;
        depthBuffer_.push_back(dep_.clone() ); // MUST CLONE
        if (depthBuffer_.size() == DEPTH_BUFFER_SIZE) {
            std::cout << "[baseTable] Computed baseTable model." << std::endl;
            std::cout << "[baseTable] Press 's' to save or press other to skip." << std::endl;

            cv::Mat baseTable = buildMinMapFrames(depthBuffer_);
            cv::imshow("baseTable", baseTable*10);

            char key = (char) cv::waitKey() % 256;
            if (key == 's') {
                cv::imwrite(params_path_ + "baseTable.png", baseTable);
                std::cout << "[baseTable] Saving current baseTable to file: " + params_path_ + "baseTable.png" << std::endl;
            }
            cv::destroyWindow("baseTable");
            retval = true;

        } else {
            cv::Mat baseTable = buildMinMapFrames(depthBuffer_);
            cv::imshow("baseTable", baseTable*10);
        }
        return retval;
    }
};


class InteractiveArea {
private:
    static const unsigned int POINT_BUFFER_SIZE = 4;
    static const unsigned int NUM_AREA = 2;

    std::string params_path_;

    std::vector< std::vector<cv::Point> > pointBuffers_;
    cv::Mat dep_;
    cv::Mat img_;
    int areaID_;
    cv::RNG rng_;

    // For OpenCV GUI
    static InteractiveArea *instance_;
    void setInstance() {
        //std::cout << "[InteractiveArea] instance set" << std::endl;
        instance_ = this;
    }

public:
    typedef boost::shared_ptr<InteractiveArea> Ptr;
    InteractiveArea(std::string params_path)
        : params_path_(params_path),
          areaID_(0),
          rng_(time(0))
    {}

    bool getMask(cv::Mat img, cv::Mat dep) {
        img_ = img;
        dep_ = dep;

        // Set OpenCV
        cv::imshow("Click 4 points", img_);
        cv::imshow("depth map", dep_*10);
        cvSetMouseCallback("Click 4 points", &InteractiveArea::mouseClickCallbackWrapper, 0);
        setInstance();

        // Allocate buffer for area1
        std::vector<cv::Point> pointBuffer;
        pointBuffers_.push_back(pointBuffer);

        std::cout << "[imask] Click 4 corners for the area " << areaID_ << std::endl;
        std::cout << "[imask] Press 's' to save or SPACE to quit" << std::endl;

        bool retval = false;
        int key = cv::waitKey(1) % 256;
        while ((unsigned char) key != ' ') {
            if (areaID_ == (int) NUM_AREA && pointBuffers_[areaID_-1].size() == POINT_BUFFER_SIZE && (unsigned char) key == 's') {

                std::ofstream outfile;
                outfile.open ((params_path_ + "area.txt").c_str());
                outfile << dep.rows << " " << dep.cols << std::endl;
                BOOST_FOREACH(std::vector< cv::Point > area, pointBuffers_) {
                    BOOST_FOREACH(cv::Point pt, area) {
                        outfile << pt.x << " " << pt.y << " ";
                    }
                    outfile << std::endl;
                }

                std::cout << "[mask] Saving current mask to file: " + params_path_ + "area.txt" << std::endl;
                retval = true;
                break;
            }
            key = cv::waitKey(1) % 256;
        }

        cv::destroyWindow("Click 4 points");
        cv::destroyWindow("depth map");
        return retval;
    }

    void mouseClickCallback(int event, int x, int y, int flags, void* param) {
        if (event == CV_EVENT_LBUTTONDOWN) {
            //std::cout << x << ", " << y << std::endl;
            cv::Point pt(x,y);

            // User clicked 4 points
            pointBuffers_[areaID_].push_back(pt);
            if (areaID_ < NUM_AREA && (int) pointBuffers_[areaID_].size() == POINT_BUFFER_SIZE) {

                cv::Mat img = img_;
                cv::Mat dep = dep_;

                cv::Scalar color = cv::Scalar(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
                int centerX, centerY;
                centerX = centerY = 0;
                for( int j = 0; j < 4; j++ ) {
                    cv::line( img, pointBuffers_[areaID_][j], pointBuffers_[areaID_][(j+1)%4], color, 2, 8 );
                    cv::line( dep, pointBuffers_[areaID_][j], pointBuffers_[areaID_][(j+1)%4], color, 2, 8 );
                    centerX += pointBuffers_[areaID_][j].x;
                    centerY += pointBuffers_[areaID_][j].y;
                }
                cv::Point center((int) 1.0*centerX/4.0, (int) 1.0*centerY/4.0);
                cv::putText(img,"area"+boost::lexical_cast<std::string>(areaID_),\
                            center,cv::FONT_HERSHEY_SIMPLEX,1,color,2,8);

                cv::imshow("Click 4 points", img);
                cv::imshow("depth map", dep*10);

                areaID_ += 1;
                if (areaID_ < NUM_AREA) {
                    std::vector<cv::Point> pointBuffer;
                    pointBuffers_.push_back(pointBuffer);
                    std::cout << "[imask] Click 4 corners for the area " << areaID_ << std::endl;
                    std::cout << "[imask] Press 's' to save or SPACE to quit" << std::endl;
                } else {
                    std::cout << "[imask] No more points to click" << std::endl;
                    std::cout << "[imask] Press 's' to save or SPACE to quit" << std::endl;
                }
            }
        }
    }

    static void mouseClickCallbackWrapper(int event, int x, int y, int flags, void* param) {
        instance_->mouseClickCallback(event, x, y, flags, param);
    }

    static void readAreaFile(std::string filename,
                             std::vector< std::vector<cv::Point> >& areaData, \
                             cv::Size& imgSize) {
        // Check existence of file
        if (!boost::filesystem::exists(filename) || !boost::filesystem::is_regular_file(filename)) {
            std::cerr << filename + " was not found."<< std::endl;
            exit(-1);
        }

        // Read data in
        std::ifstream file ( filename.c_str() );
        std::string line;
        for (int i = 0; i < NUM_AREA+1; ++i) {
            getline ( file, line, '\n' );
            std::stringstream iss(line);

            if (i == 0) { // first line is for the size of img
                iss >> imgSize.height;
                iss >> imgSize.width;
            } else {
                std::vector<cv::Point> buffer;
                areaData.push_back(buffer);
                for (int j = 0; j < POINT_BUFFER_SIZE; ++j) {
                    double x,y;
                    iss >> x; iss >> y;
                    areaData[i-1].push_back(cv::Point(x,y));
                }
            }
        }
    }

    static void getAreaMasksFromFile(std::string filename, std::vector<cv::Mat>& areaMasks) {
        cv::Size temp(640,480);
        std::vector< std::vector<cv::Point> > areaData;
        cv::Size imgSize;
        InteractiveArea::readAreaFile(filename,areaData,imgSize);

        for (int i = 0; i < areaData.size(); ++i) {
            areaMasks.push_back(contour2mask(areaData[i],imgSize));
            cv::imshow("areaMasks",areaMasks[i]);
            cv::waitKey();
        }
    }
};
InteractiveArea *InteractiveArea::instance_ = NULL;


class CaptureImages {
private:
    static const unsigned int POINT_BUFFER_SIZE = 4;
    static const unsigned int NUM_AREA = 2;

    std::string params_path_;
    cv::Mat dep_;
    cv::Mat img_;

    pcl::PCDWriter w_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMsg_;

    // For OpenCV GUI
    static CaptureImages *instance_;
    void setInstance() {
        instance_ = this;
    }

public:
    typedef boost::shared_ptr<CaptureImages> Ptr;
    CaptureImages(std::string params_path)
        : params_path_(params_path)
    {
        std::cout << "[capture] Click to capture images and pcd" << std::endl;
        std::cout << "[capture] SPACE to quit" << std::endl;

        cv::namedWindow("Click to capture", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("depth map", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("Last captured rgb", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("Last captured depth", CV_WINDOW_AUTOSIZE);
        cvSetMouseCallback("Click to capture", &CaptureImages::mouseClickCallbackWrapper, 0);
        setInstance();
    }

    ~CaptureImages() {
        cv::destroyWindow("Click to capture");
        cv::destroyWindow("depth map");
        cv::destroyWindow("Last captured rgb");
        cv::destroyWindow("Last captured depth");
    }

    bool update(cv::Mat img, cv::Mat dep,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudMsg) {
        img_ = img;
        dep_ = dep;
        cloudMsg_ = cloudMsg;

        // Set OpenCV
        cv::imshow("Click to capture", img_);
        cv::imshow("depth map", dep_*10);

        int key = cv::waitKey(1) % 256;
        if (key != ' ') {
            return false;
        } else
            return true;
    }

    void mouseClickCallback(int event, int x, int y, int flags, void* param) {
        if (event == CV_EVENT_LBUTTONDOWN) {
            cv::Mat img = img_;
            cv::Mat dep = dep_;

            // [NOTE] ctime way to do timestap
//            time_t rawtime;
//            struct tm * timeinfo;
//            char imgfilename[80];
//            char depfilename[80];
//            char pcdfilename[80];
//            time ( &rawtime );
//            timeinfo = localtime ( &rawtime );

//            strftime (imgfilename,80,"img_%m%d%H%M%S.png",timeinfo); puts(imgfilename);
//            strftime (depfilename,80,"dep_%m%d%H%M%S.png",timeinfo); puts(depfilename);
//            strftime (pcdfilename,80,"pcd_%m%d%H%M%S.pcd",timeinfo); puts(pcdfilename);

            // [NOTE] boost way to do timestap
            std::stringstream ss;
            ss << boost::posix_time::to_iso_string (boost::posix_time::second_clock::local_time ());

            // opencv write
//            cv::imwrite(params_path_ + imgfilename, img);
//            cv::imwrite(params_path_ + depfilename, dep);
            cv::imwrite(params_path_ + "img_" + ss.str() + ".png", img);
            cv::imwrite(params_path_ + "dep_" + ss.str() + ".png", dep);

            // pcl write
            // [NOTE] hardcoded to pcl::PointXYZRGB type for now
//            w_.writeASCII<pcl::PointXYZRGB> (params_path_ + pcdfilename, *cloudMsg_);
            w_.writeASCII<pcl::PointXYZRGB> (params_path_ + "pcd_" + ss.str() + ".pcd", *cloudMsg_);

            cv::imshow("Last captured rgb", img);
            cv::imshow("Last captured depth", dep*10);
        }
    }

    static void mouseClickCallbackWrapper(int event, int x, int y, int flags, void* param) {
        instance_->mouseClickCallback(event, x, y, flags, param);
    }

};
CaptureImages *CaptureImages::instance_ = NULL;


#endif // CV_TABLEIMGPROC_H
