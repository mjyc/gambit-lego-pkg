/**
 * @file
 * @author Brian D. Mayton <bmayton@bdm.cc>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2010, Intel Labs Seattle
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - Neither the name of Intel Labs Seattle nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * Classes for representing robotic degrees of freedom.
 */

#ifndef __gambit_driver_dof_h_
#define __gambit_driver_dof_h_

#include <vector>

namespace gambit_driver {
    class DOF;
}
#include <gambit_driver/actuator.h>
#include <gambit_msgs/DOFInfo.h>
#include <gambit_msgs/DOFProperty.h>

namespace gambit_driver {
using namespace gambit_msgs;

/**
 * @brief An abstract representation of a degree of freedom.
 * 
 * The DOF class is an abstract representation of a robotic degree of freedom.  In this
 * driver architecture, the user interacts with the arm at the DOF level, which translates
 * internally to actuator commands.
 */
class DOF {
    public:

    /**
     * Default constructor.
     */
    DOF(std::string dof_name);
   
    /**
     * Adds an actuator to this DOF.  Generally, each DOF has one actuator, with a weight
     * of 1.0.  In cases where a DOF is a linear combination of actuators, multiple 
     * actuators can be added.  In this case, the first actuator added should be the
     * "primary" actuator, i.e., the actuator whose torque should be enabled/disabled when
     * the DOF is enabled/disabled.
     *
     * @param a pointer to the actuator object to add
     * @param weight the coefficient of the actuator in the conversion from actuator
     *  position to DOF position
     */
    void add_actuator(Actuator *a, double weight);

    /**
     * Sets the joint limits for this DOF.
     *
     * @param angle_min the low joint angle limit
     * @param angle_max the high joint angle limit
     */
    void set_limits(double angle_min, double angle_max);

    /**
     * Sets a calibration offset for the DOF's zero point.
     *
     * @param joint_offset the calibration offset, in radians
     */
    void set_offset(double joint_offset);

    /**
     * Sets the global index of this DOF (not used internally, but the index is included
     * in the ROS messages, so nodes know how to address this DOF in the scope of the
     * driver.
     * 
     * @param dof_index the global index of this DOF
     */
    inline void set_index(unsigned int dof_index) { index = dof_index; }

    inline unsigned int get_index() { return index; }

    inline std::string get_name() { return name; }

    /**
     * Gets the actual position of this DOF (from encoder values of its actuators, 
     * regardless of the commanded DOF position)
     * 
     * @return the actual DOF joint angle
     */
    double get_position();

    double get_absolute_position(bool apply_offset=true);

    double get_velocity();
    
    double get_torque();

    /**
     * Should be called after the actuators have read their encoders to update internal
     * DOF state (e.g. from the control loop)
     */
    void update();

    /**
     * Called to reset the commanded position to the actual position and reset any
     * internal state when activating a previously inactive DOF
     */
    virtual void reset() { }

    /**
     * Sets whether this DOF is active (i.e. actively controlling to a position)
     *
     * @param en whether the DOF is enabled
     */
    void set_enabled(bool en);

    inline bool get_enabled() { return enabled; }

    /**
     * Gets the last commanded position of this DOF (if this DOF is disabled, then this
     * will be the actual position of the DOF.)
     * @return the commanded position for this DOF
     */
    virtual double get_cmd_position();

    virtual void set_cmd_position(double new_position, bool apply_offset=true);
    virtual void set_cmd_position(double new_position, ros::Time timestamp,
        bool apply_offset=true);

    /**
     * Gets a ROS DOFInfo message for this DOF.
     * 
     * @return a boost::shared_ptr to a DOFInfo message containing the current state of this
     *  DOF.
     */
    void get_DOFInfo(DOFInfo &info);

    void get_properties(std::vector<DOFProperty> &properties);

    bool set_property(DOFProperty &p);

    void set_continuous_rotation(bool value);

    protected:
    std::vector<Actuator*> actuators;
    std::vector<double> weights;

    std::string name;
    unsigned int index;
    double cmd_position;        // commanded position
    bool enabled;               // whether or not this DOF is enabled (applying torque)
    double limit_min;           // minimum joint angle limit
    double limit_max;           // maximum joint angle limit
    double offset;

    bool continuous_rotation;
};

}

#endif // __gambit_driver_dof_h_

