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
 * An implementation of Actuator for Copley AccelNet drives.
 */

#ifndef __gambit_driver_actuator_copley_h_
#define __gambit_driver_actuator_copley_h_

#include <gambit_driver/actuator.h>

#include <copley/copleyCmds.h>
#include <copley/copleySockCanSdo.h>
#include <copley/copleyDictionary.h>

namespace gambit_driver {

class ActuatorCopley : public Actuator {
    
    public:

    ActuatorCopley(unsigned int node_id, unsigned int counts_per_rev);
    ~ActuatorCopley();

    inline void set_torque_constant(double value) { torque_constant = value; }

    double get_position();
    double get_velocity();
    double get_torque();

    bool ping();
    void set_torque_enabled(bool enabled);
    void send_control() {};
    
    void home(bool direction);
    bool is_homed();
    void sync();

    virtual void get_properties(std::vector<DOFProperty> &properties);
    virtual bool set_property(DOFProperty &p);

    void load_calibration_parameters();
    
    /* Realtime/control loop functions */
    void stop_interpolated_move();
    int start_interpolated_move(double rate);
    int set_interpolated_target(int target);
    inline bool get_trajectory_active() { return trajectory_active; }
    void read_status();
    void send_updated_parameters();

    unsigned int counts_per_rad;
    
    protected:
    unsigned int id;
    double position;
    bool trajectory_active;
    double torque_constant;

    int16_t peak_current_limit;
    int16_t continuous_current_limit;
    int16_t peak_current_time;
    bool parameters_updated;

};

}


#endif // __gambit_driver_actuator_copley_h_

