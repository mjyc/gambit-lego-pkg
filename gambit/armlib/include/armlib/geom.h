#ifndef __armlib_geom_h_
#define __armlib_geom_h_

#include <armlib/trajectory.h>

namespace armlib {
namespace geom {

/**
 * @brief Converts roll, pitch, and yaw angles to a rotation matrix.
 *
 * Copied from pywam.util3d; need to check to see which order the angles are being applied;
 * may be better to use fixed euler angles...
 *
 * @param rmat the rotation matrix to be filled (a 9-element, 3x3 row major array)
 * @param roll the roll angle
 * @param pitch the pitch angle
 * @param yaw the yaw angle
 */
void rot_matrix_from_rpy(double *rmat, double roll, double pitch, double yaw);

/**
 * @brief Returns the distance between two n-dimensional points
 * 
 * @param a first point
 * @param b second point
 * @return the distance between the points
 */
double js_distance(js_vect &a, js_vect &b);

/**
 * @brief Returns the closest point.
 * 
 * Given a vector of candidate points and a reference point, returns the candidate point that
 * is closest to the reference.
 *
 * @param candidates a vector of candidate points
 * @param the reference point
 * @return the closest point from the candidate list
 */
js_vect closest_point(std::vector<js_vect> &candidates, js_vect &reference);

double angle_difference(double a1, double a2);

}
}

#endif // __armlib_geom_h_
