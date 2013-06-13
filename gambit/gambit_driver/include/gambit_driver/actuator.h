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
 * Classes for representing arm actuators.
 */


#ifndef __gambit_driver_actuator_h_
#define __gambit_driver_actuator_h_

#include <vector>

namespace gambit_driver {
    class Actuator;
}
#include <gambit_driver/DOF.h>
#include <gambit_msgs/DOFProperty.h>

namespace gambit_driver {
using namespace gambit_msgs;

/**
 * @brief An abstract representation of a robotic actuator.
 *
 * The Actuator class defines an abstract representation of a robotic actuator.  This class
 * must be extended to provide the implementation for actually controlling the physical
 * actuator.
 */
class Actuator {
    public:

    /**
     * Gets the actual position of the actuator from the last encoder reading, regardless
     * of the commanded position of the actuator.
     *
     * @return the encoder position, in radians
     */
    virtual double get_position() = 0;

    /**
     * Gets the actual velocity of the actuator.
     *
     * @return the velocity of the actuator, in radians/second
     */
    virtual double get_velocity() = 0;

    /**
     * Gets the torque currently exerted by the actuator.
     *
     * @return current actuator torque in Newton-meters.
     */
    virtual double get_torque() = 0;

    /**
     * Checks for the physical presence and connectivity of this actuator.
     *
     * @return whether or not the actuator is responding
     */
    virtual bool ping() = 0;

    /**
     * Sets whether or not the actuator is allowed to apply torques to actively control to
     * a position.
     *
     * @param enabled whether the actuator is enabled
     */
    virtual void set_torque_enabled(bool enabled) = 0;

    /**
     * Gets whether the primary DOF for this actuator is enabled.
     */
    bool get_DOF_enabled();

    /**
     * Reads the state of the actuator (i.e. triggers communication on the bus) and updates
     * the internal state variables of this class.  Should be called periodically from the
     * control loop.
     */
    virtual void read_status() = 0;

    virtual void send_control() = 0;

    virtual void reset();

    /**
     * Adds a DOF that influences the position of this actuator.  In the simple case, each
     * actuator controls only one DOF which has a weight of 1.0.  In instances where this
     * actuator affects the position of multiple DOFs, this function should be called to
     * add each DOF.
     *
     * @param d a pointer to the DOF object to add
     * @param weight the coefficient of the DOF in the computation of the position of this
     *  actuator from the DOF angles
     */
     void add_DOF(DOF *d, double weight);

    /**
     * Gets the commanded position of this actuator (based on the commanded positions of the
     * DOFs that comprise it)
     *
     * @return the commanded position, in radians
     */
    double get_cmd_position();

    virtual bool set_property(DOFProperty &p);
    virtual void get_properties(std::vector<DOFProperty> &properties);

    virtual void load_calibration_parameters() { }

    inline void set_name(std::string &name) { primary_name = name; }

    protected:
    std::string primary_name;
    std::vector<DOF*> DOFs;
    std::vector<double> weights;

};

}

#endif // __gambit_driver_actuator_h_

