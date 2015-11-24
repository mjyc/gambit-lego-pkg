#ifndef TF_UTILS_H
#define TF_UTILS_H

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <tf/tf.h>


std::vector< std::vector<double> >
loadMatrixFromFile(std::string filename) {
    // Check existence of file
    using namespace boost::filesystem;
    if (exists(filename))
        if (!is_regular_file(filename)) {
            std::cerr << filename + " was not found."<< std::endl;
            exit(-1);
        }

    // Read data in
    std::ifstream file ( filename.c_str() );
    std::vector< std::vector<double> > data;
    std::string line;
    while ( file.good() )
    {
        std::vector<double> lineData;
        getline ( file, line, '\n' );
        if (line == "")
            continue;
        std::stringstream iss(line);
        double value;
        while (iss >> value) {
            lineData.push_back(value);
        }
        data.push_back(lineData);
    }

    return data;
}

Eigen::MatrixXf loadEigenMatrixFromFile(std::string filename) {
    std::vector< std::vector<double> > data = loadMatrixFromFile(filename);

    // Convert to MatrixXf
    Eigen::MatrixXf m(data.size(),data[0].size());
    for (int r = 0; r < data.size(); ++r) {
        for (int c = 0; c < data[r].size(); ++c) {
            m(r,c) = data[r][c];
        }
    }
    return m;
}

inline void tfToEigen(const tf::Transform &tx, Eigen::Matrix4f &out) {
    tf::Quaternion rot = tx.getRotation();
    Eigen::Quaternionf rot_eig(rot.w(), rot.x(), rot.y(), rot.z());
    tf::Vector3 origin = tx.getOrigin();
    Eigen::Vector3f origin_eig(origin.x(), origin.y(), origin.z());
    Eigen::Vector3f scale(1,1,1);
    Eigen::Affine3f tr;
    tr.fromPositionOrientationScale(origin_eig, rot_eig, scale);
    out = tr.matrix();
}

inline void eigenToTf(const Eigen::Matrix4f &t, tf::Transform& out) {
    Eigen::Matrix3f rotation = t.block<3,3>(0, 0);
    Eigen::Vector3f translation = t.block<3,1>(0, 3);

    tf::Matrix3x3 basis;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            basis[r][c] = rotation(r,c);
        }
    }

    tf::Vector3 orig;
    for (int i = 0; i < 3; ++i) {
        orig[i] = translation(i);
    }

    out.setOrigin(orig);
    out.setBasis(basis);
}

inline void printRotation(const tf::Matrix3x3 &rotation) {
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation[0][0], rotation[0][1], rotation[0][2]);
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation[1][0], rotation[1][1], rotation[1][2]);
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation[2][0], rotation[2][1], rotation[2][2]);
}

inline void printTranslation(const tf::Vector3 &translation) {
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation[0], translation[1], translation[2]);
}

// WARN: "PointCloud" means pcl::PointCloud<pcl::PointXYZRGB>
inline void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, \
                                pcl::PointCloud<pcl::PointXYZRGB> &cloud_out, \
                                tf::StampedTransform &tx) {
    Eigen::Matrix4f t;
    tfToEigen(tx, t);
    pcl::transformPointCloud(cloud_in, cloud_out, t);
}

#endif // TF_UTILS_H
