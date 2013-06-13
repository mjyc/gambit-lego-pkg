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
 * A subclass of DOF that uses splined interpolation for smoother motion.
 */

#include <gambit_driver/smoothed_DOF.h>

namespace gambit_driver {

SmoothedDOF::SmoothedDOF(std::string dof_name, unsigned int lag_time):
    DOF(dof_name)
{
    realtime = false;
    smoother = new smoothing::Smoother();
    smoother->setLagTime(lag_time * 1000);
    ROS_FATAL("I'm setting the lag time to %d!!!!", lag_time);
    pthread_mutex_init(&smoother_lock, NULL);
}

void SmoothedDOF::set_realtime(bool value) {
    realtime = value;
    smoother->setRealtime(value);
}

double SmoothedDOF::get_cmd_position() {
    if(!(enabled && smoother->isValid()))
        return get_position();
    double position;
    lock();
    smoother->getSmoothedPVA(&position);
    unlock();
    return position;
}

void SmoothedDOF::set_cmd_position(double new_position, bool apply_offset) {
    set_cmd_position(new_position, ros::Time::now(), apply_offset);
}

void SmoothedDOF::set_cmd_position(double new_position, ros::Time timestamp,
    bool apply_offset) {
    
    if(!(enabled && smoother->isValid()))
        return;
    if(continuous_rotation) {
        double cur_position = get_absolute_position();
        double a1 = atan2(sin(cur_position), cos(cur_position));
        double a2 = atan2(sin(new_position), cos(new_position));
        double da = a2 - a1;
        if(da < -M_PI) da += M_PI * 2;
        if(da > M_PI) da -= M_PI * 2;
        new_position = cur_position + da;
    }
    
    uint64_t timestamp_usec = ((uint64_t)timestamp.sec * 1000000000UL + 
        (uint64_t)timestamp.nsec) / 1000;

    pthread_mutex_lock(&smoother_lock);
    smoother->setCoarseTarg(timestamp_usec, new_position - (apply_offset? offset : 0));
    pthread_mutex_unlock(&smoother_lock);
}

void SmoothedDOF::reset() {
    double position = get_absolute_position(false);
    lock();
    smoother->reset(position);
    unlock();
}

}

