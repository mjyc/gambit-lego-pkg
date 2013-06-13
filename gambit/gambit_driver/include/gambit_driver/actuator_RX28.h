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
 * An implementation of Actuator for Dynamixel RX-28 servomotors.
 */


#ifndef __gambit_driver_actuator_RX28_h_
#define __gambit_driver_actuator_RX28_h_

#include <gambit_driver/actuator.h>
#include <bus/interface.h>
#include <dynamixel/actuator.h>
#include <dynamixel/packet.h>

namespace gambit_driver {

/**
 * @brief An implementation of Actuator for Dynamixel RX-28 servomotors.
 */
class ActuatorRX28 : public Actuator {
    public:

    /**
     * Constructor for a new RX-28 actuator.
     *
     * @param iface the communication interface bus to be used for communications with this
     *  actuator
     * @param node_id the address/ID of the Dynamixel on the RS-485 bus
     */
    ActuatorRX28(bus::Interface *iface, unsigned int node_id);

    /**
     * Default destructor.
     */
    ~ActuatorRX28();

    double get_position();
    double get_velocity();
    double get_torque();

    bool ping();
    void set_torque_enabled(bool enabled);
    void read_status();
    void send_control();

    virtual bool set_property(DOFProperty &p);
    virtual void get_properties(std::vector<DOFProperty> &properties);

    protected:
    // Torque in newton-meters corresponding to 1023 dynamixel torque units
    static const double RX28_MAX_TORQUE = 2.77528195;
    static const double RX28_SPEED_CONVERSION = 0.111 * 2.0 * M_PI / 60.0;

    dynamixel::RX28 *rx28;

    ros::Time last_error_warning;
    uint8_t last_error_state;
};

}

#endif // __gambit_driver_actuator_RX28_h_

