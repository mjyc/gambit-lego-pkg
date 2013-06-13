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

#ifndef __gambit_driver_smoothed_dof_h_
#define __gambit_driver_smoothed_dof_h_

#include <ros/ros.h>

#include <gambit_driver/DOF.h>
#include <smoothing/Smoother.h>

#include <stdint.h>
#include <pthread.h>


namespace gambit_driver {

class SmoothedDOF : public DOF {
    
    public:

    SmoothedDOF(std::string dof_name, unsigned int lag_time);
    double get_cmd_position();
    void set_cmd_position(double new_position, bool apply_offset=true);
    void set_cmd_position(double new_position, ros::Time timestamp, bool apply_offset=true);
    void reset();
    void set_realtime(bool value);

    inline void lock() { if(!realtime) pthread_mutex_lock(&smoother_lock); }
    inline void unlock() { if(!realtime) pthread_mutex_unlock(&smoother_lock); } 

    protected:

    smoothing::Smoother *smoother;
    pthread_mutex_t smoother_lock;
    bool realtime;
};

}


#endif // __gambit_driver_smoothed_dof_h_

