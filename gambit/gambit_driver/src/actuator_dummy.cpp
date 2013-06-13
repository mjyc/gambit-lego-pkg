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
 * A "dummy" actuator that merely stores a position, velocity, and torque, without
 * communicating with an actual actuator.
 */

#include <gambit_driver/actuator_dummy.h>

namespace gambit_driver {

ActuatorDummy::ActuatorDummy() {
    d_position = 0.0;
    d_velocity = 0.0;
    d_torque = 0.0;
}

double ActuatorDummy::get_position() {
    return d_position;
}

double ActuatorDummy::get_velocity() {
    return d_velocity;
}

double ActuatorDummy::get_torque() {
    return d_torque;
}

bool ActuatorDummy::ping() {
    return true;
}

void ActuatorDummy::set_torque_enabled(bool enabled) {
    /* unimplemented */
}

void ActuatorDummy::read_status() {
    /* nothing to do */
}

void ActuatorDummy::send_control() {
    d_position = get_cmd_position();
}

void ActuatorDummy::set_status(double position, double velocity, double torque) {
    d_position = position;
    d_velocity = velocity;
    d_torque = torque;
}

}


