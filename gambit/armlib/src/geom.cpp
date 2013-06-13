#include <armlib/geom.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <float.h>
#include <vector>

namespace armlib {
namespace geom {

void rot_matrix_from_rpy(double *rmat, double roll, double pitch, double yaw) {
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    rmat[0] = cr*cp;
    rmat[1] = -sr*cy + cr*sp*sy;
    rmat[2] = sr*sy + cr*sp*cy;

    rmat[3] = sr*cp;
    rmat[4] = cr*cy + sr*sp*sy;
    rmat[5] = -cr*sy + sr*sp*cy;

    rmat[6] = -sp;
    rmat[7] = cp*sy;
    rmat[8] = cp*cy;

}

double js_distance(js_vect &a, js_vect &b) {
    assert(a.size() == b.size());
    double sum = 0.0;
    for(unsigned int i=0; i<a.size(); i++)
        sum += pow(a[i] - b[i], 2.0);
    return sqrt(sum);
}

js_vect closest_point(std::vector<js_vect> &candidates, js_vect &reference) {
    assert(candidates.size() > 0);
    double min_dist = DBL_MAX;
    js_vect *closest = NULL;
    for(unsigned int i=0; i<candidates.size(); i++) {
        double d = js_distance(reference, candidates[i]);
        if(d < min_dist) {
            min_dist = d;
            closest = &candidates[i];
        }
    }
    return *closest;
}

double angle_difference(double a1, double a2) {
    double a1m = atan2(sin(a1), cos(a1));
    double a2m = atan2(sin(a2), cos(a2));
    return a1m - a2m;
}

}
}
