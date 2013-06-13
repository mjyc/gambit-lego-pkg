#ifndef __armlib_recorded_trajectory_h_
#define __armlib_recorded_trajectory_h_

#include <armlib/trajectory.h>
#include <Eigen/Core>
#include <string>

namespace armlib {

class RecordedTrajectory : public Trajectory {
    public:
    RecordedTrajectory(const char *filename);
    
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
    std::string _filename;
};

}

#endif // __armlib_js_linear_trajectory_h_

