// Ported&Inspired from Jinna's oasis_svn_repo code
#ifndef CV_UTILS_PY
#define CV_UTILS_PY

#include <limits>
#include <vector>
#include <iostream>

#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>

#include <gambit_perception/depth_traits.h>


// typedefs
typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

// Convert pointCloud format (float{nan}/m) to depthMap format (uint16{0}/mm)
void toOpenCVDepthMap(cv::Mat& pointCloud, cv::Mat& depthMap) {
    if (pointCloud.type() != CV_32FC1) {
        std::cerr << "Expected data of type " << CV_32FC1 << ", got " << pointCloud.type() << std::endl;
        return;
    }
    // internal conversion of NaN to 0
    pointCloud.convertTo(depthMap,CV_16UC1,1000);
}

// TODO: NOT TESTED YET - TEST IT
// Convert pointCloud format (float{nan}/m) to depthMap format (uint16{0}/mm)
//void fromOpenCVDepthMap(cv::Mat& depthMap, cv::Mat& pointCloud) {
//    cv::Mat zeroMask;
//    cv::compare(depthMap, 0, zeroMask, cv::CMP_LE);
//    pointCloud.convertTo(depthMap,CV_32FC1,0.001);
//    depthMap.setTo(std::numeric_limits<float>::quiet_NaN(), zeroMask);
//}

pcl::PointXYZRGB
toPointXYZRGB(float colInd, float rowInd, uint16_t depth, uint8_t r, uint8_t g, uint8_t b, \
              float fx, float fy, float centerX, float centerY)
{
    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = gambit_perception::DepthTraits<uint16_t>::toMeters( uint16_t(1) );
    float constant_x = unit_scaling / fx;
    float constant_y = unit_scaling / fy;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    pcl::PointXYZRGB pt;
    if (!gambit_perception::DepthTraits<uint16_t>::valid(depth)) {
        pt.x = pt.y = pt.z = bad_point;
    } else {
        // Fill in XYZ
        pt.x = (colInd - centerX) * depth * constant_x;
        pt.y = (rowInd - centerY) * depth * constant_y;
        pt.z = gambit_perception::DepthTraits<uint16_t>::toMeters(depth);
    }

    // Fill in color
    RGBValue color;
    color.Red   = r;
    color.Green = g;
    color.Blue  = b;
    color.Alpha = 0;
    pt.rgb = color.float_value;

    return pt;
}

void toPointCloud(cv::Mat& img, cv::Mat& dep, float fx, float fy, float centerX, float centerY,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudMsg,
                  int red_offset=2, int green_offset=1, int blue_offset=0, int color_step=3)
{
    // Set PointCloud fields
    cloudMsg->height = dep.rows;
    cloudMsg->width  = dep.cols;
    cloudMsg->is_dense = false;
    cloudMsg->points.resize (cloudMsg->height * cloudMsg->width);

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = gambit_perception::DepthTraits<uint16_t>::toMeters( uint16_t(1) );
    float constant_x = unit_scaling / fx;
    float constant_y = unit_scaling / fy;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    uint16_t* depth_row = dep.ptr<uint16_t>(0);
    int row_step = dep.step / sizeof(uint16_t);
    uint8_t* rgb = img.ptr<uint8_t>(0);
    int rgb_skip = img.step - img.cols * color_step;
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloudMsg->begin();

    for (int v = 0; v < (int)cloudMsg->height; ++v, depth_row += row_step, rgb += rgb_skip)
    {
        for (int u = 0; u < (int)cloudMsg->width; ++u, rgb += color_step)
        {
            pcl::PointXYZRGB& pt = *pt_iter++;
            uint16_t depth = depth_row[u];

            // Check for invalid measurements
            if (!gambit_perception::DepthTraits<uint16_t>::valid(depth))
            {
                pt.x = pt.y = pt.z = bad_point;
            }
            else
            {
                // Fill in XYZ
                pt.x = (u - centerX) * depth * constant_x;
                pt.y = (v - centerY) * depth * constant_y;
                pt.z = gambit_perception::DepthTraits<uint16_t>::toMeters(depth);
            }

            // Fill in color
            RGBValue color;
            color.Red   = rgb[red_offset];
            color.Green = rgb[green_offset];
            color.Blue  = rgb[blue_offset];
            color.Alpha = 0;
            pt.rgb = color.float_value;
        }
    }
}

void patchValue16UC1(cv::Mat input, unsigned int fromVal, unsigned int toVal) {
    if (input.type() != CV_16UC1) {
        std::cerr << "patchValue16UC1: input.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_16UC1 << ", got " << input.type() << std::endl;
        exit(-1);
    }

    cv::Mat mask = (input == fromVal);
    input.setTo(toVal,mask);
}

std::vector< std::vector<cv::Point> > traceContours(cv::Mat img) {
    if (img.type() != CV_8UC1) {
        std::cerr << "traceContours: img.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_8UC1 << ", got " << img.type() << std::endl;
        exit(-1);
    }
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(img,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

cv::Mat contours2mask(std::vector< std::vector<cv::Point> > contours, cv::Size size) {
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    cv::fillPoly(mask, contours, cv::Scalar(255));
    return mask;
}

cv::Mat contour2mask(std::vector<cv::Point> contour, cv::Size size) {
    std::vector< std::vector<cv::Point> > contours(1, contour);
    return contours2mask(contours,size);
}

// WARNING: Uses bitwise logical operators
bool crossesMask(cv::Mat contourMask, cv::Mat mask) {
    if (contourMask.type() != CV_8UC1) {
        std::cerr << "crossesMask: contourMask.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_8UC1 << ", got " << contourMask.type() << std::endl;
        exit(-1);
    }
    if (mask.type() != CV_8UC1) {
        std::cerr << "crossesMask: mask.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_8UC1 << ", got " << mask.type() << std::endl;
        exit(-1);
    }

    int area0 = cv::countNonZero(contourMask);
    int area1 = cv::countNonZero(mask & contourMask);
    if (area1 < area0 && area1 > 0)
        return true;
    else
        return false;
}

bool crossesMask(std::vector<cv::Point> contour, cv::Mat mask) {
    cv::Mat contourMask = contour2mask(contour,mask.size());
    return crossesMask(contourMask, mask);
}

// @params dep: an input depth image
// @params mask: the mask we want to find crossed contours with
// @params min_handsize_frac: a contour size threshold
// @return : contour vector which has contours that crosses the input maskdj
std::vector< std::vector<cv::Point> > \
crossedContours(cv::Mat dep, cv::Mat mask, float min_handsize_frac=0.002) {
    if (mask.type() != CV_8UC1) {
        std::cerr << "crossedContours: mask.type()" << std::endl;
        std::cerr << "Expected data of type " << CV_8UC1 << ", got " << mask.type() << std::endl;
        exit(-1);
    }

    double vidArea = mask.size().area();
    std::vector< std::vector<cv::Point> > contours = traceContours(dep);
    std::vector< std::vector<cv::Point> > filteredContours;
    BOOST_FOREACH(std::vector<cv::Point> contour , contours ) {
        cv::RotatedRect rect = cv::minAreaRect(cv::Mat(contour));
        if (contour.size() > 2 && \
                rect.size.area() >= min_handsize_frac * vidArea && // for RotatedRect
                crossesMask(contour, mask)) {
            filteredContours.push_back(contour);
        }
    }

    return filteredContours;
}

cv::Mat crossedContoursMask(cv::Mat dep, cv::Mat mask, float min_handsize_frac=0.002) {
    cv::Mat contoursMask = contours2mask( \
                crossedContours(dep,mask,min_handsize_frac), mask.size());
    return contoursMask;
}

// ASSUMPTION: Input frames are 16UC1 type
cv::Mat buildMinMapFrames(std::vector<cv::Mat> frames) {
    if (frames[0].type() != CV_16UC1) {
        std::cerr << "Expected data of type " << CV_16UC1 << ", got " << frames[0].type() << std::endl;
        exit(-1);
    }

    unsigned int big = 65535; // biggest number uint16 can get
    cv::Mat minMap = cv::Mat::ones(frames[0].size(), frames[0].type()) * big;
    BOOST_FOREACH( cv::Mat frame, frames ) {
        // Set 0 to big
        cv::Mat mask;
        cv::compare(frame, 0, mask, cv::CMP_LE);
        frame.setTo(big,mask);
        //compute minmap
        cv::min(minMap,frame,minMap);
    }
    cv::Mat mask;
    cv::compare(minMap, big, mask, cv::CMP_EQ);
    minMap.setTo(0,mask);

    return minMap;
}

#endif // CV_UTILS_PY
