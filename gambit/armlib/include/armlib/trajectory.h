#ifndef __armlib_trajectory_h_
#define __armlib_trajectory_h_

#include <vector>
#include <string>

namespace armlib {

/**
 * Vector of jointspace coordinates
 */
typedef std::vector<float> js_vect;

/**
 * @brief Abstract class defining a pre-computed motion of the arm.
 */
class Trajectory {
    public:

    /**
     * Gets the amount of time required to execute the trajectory, in seconds.
     *
     * @return the length of the trajectory, in seconds
     */
    virtual double get_length() = 0;

    /**
     * Gets the endpoint of the trajectory.
     *
     * @param joints reference to a vector that will be set to the trajectory endpoint
     */
    virtual void get_end(js_vect &joints) = 0;

    /**
     * Gets the start point of the trajectory.
     *
     * @param joints reference to a vector that will be set to the trajectory start point
     */
    virtual void get_start(js_vect &joints) = 0;

    /**
     * Evaluates the trajectory at a point in time.
     *
     * @param t time offset from the beginning of the trajectory that is being evaluated
     * @param joints reference to a vector that will be set to the position in which the arm
     *  should be at time t
     */
    virtual void evaluate(double t, js_vect &joints) = 0;

    /**
     * Gets the number of degrees of freedom in the trajectory.
     * 
     * @return the number of DOFs in the trajectory
     */
    virtual unsigned int get_num_dofs() = 0;

    /**
     * Gets the name of this trajectory.
     *
     * @return a string describing this trajectory
     */
    virtual std::string get_name() = 0;
};

}

#endif // __armlib_trajectory_h_
