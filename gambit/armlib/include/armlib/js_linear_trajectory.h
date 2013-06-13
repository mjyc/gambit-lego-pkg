#ifndef __armlib_js_linear_trajectory_h_
#define __armlib_js_linear_trajectory_h_

#include <armlib/trajectory.h>
#include <Eigen/Core>

namespace armlib {

/**
 * @brief A linear trajectory between two points in joint space, with a trapezoidal velocity
 *  profile.
 *
 * Computed from a start point and an end point.
 */
class JSLinearTrajectory : public Trajectory {
    public:
    /**
     * @brief Constructs a new linear trajectory between two points in joint space with a
     *  trapezoidal velocity profile.
     *
     * @param start the start point of the trajectory
     * @param end the end point of the trajectory
     * @param joint_weights a vector of weights to be used in the computation of velocities.
     *  If all weights are 1.0, no joint will ever move faster than vmax.  A weight specified
     *  as 2.0 will result in that joint never exceeding vmax/2.0.  This allows large joints
     *  to have large weights to ensure that they move relatively slowly while still allowing
     *  smaller joints to have small weights so that they can move quickly.
     * @param vmax the velocity of the constant-velocity segment of the trapezoidal velocity
     *  profile, provided that amax is sufficient to allow this velocity to be reached.
     * @param amax the acceleration for the speed-up and slow-down segments of the trapezoidal
     *  velocity profile.
     * @param timestep the resolution at which points of the trajectory will be stored
     *  internally.  The default value is generally acceptable.
     */
    JSLinearTrajectory(js_vect &start, js_vect &end, js_vect &joint_weights,
        double vmax, double amax, double timestep=0.05);
    virtual double get_length();
    virtual void get_end(js_vect &joints);
    virtual void get_start(js_vect &joints);
    virtual void evaluate(double t, js_vect &joints);
    virtual unsigned int get_num_dofs();
    virtual std::string get_name();
    void print();

    protected:
    unsigned int _n_dofs;
    unsigned int _n_points;
    double _timestep;
    std::vector<Eigen::VectorXf> _points;
};

}

#endif // __armlib_js_linear_trajectory_h_

