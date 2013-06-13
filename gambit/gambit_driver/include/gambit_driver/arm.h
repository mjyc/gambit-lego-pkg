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
 * High-level definiton of the newarm 3-DOF arm.
 */

 #ifndef __gambit_driver_arm_h_
 #define __gambit_driver_arm_h_

#include <ros/ros.h>

#include <gambit_driver/actuator_group.h>
#include <gambit_driver/actuator_copley.h>
#include <gambit_driver/smoothed_DOF.h>

#ifdef REALTIME
#include <native/task.h>
#include <native/timer.h>
#else
#include <pthread.h>
#endif

#include <stdint.h>

namespace gambit_driver {

class Arm : public ActuatorGroup {

    public:
    Arm();

    void initialize();
    bool check_connectivity();
    void set_control_rate(double rate);
    void control();

    protected:
    void sync_drives();

#ifdef REALTIME
    static void internal_control(void *arg);
#else
    static void *internal_control(void *arg);
#endif    

    /* control thread */
    double control_thread_rate;
    bool control_started;
#ifdef REALTIME
    RT_TASK rt_internal_thread;
#else
    pthread_t internal_thread;
#endif

    uint64_t last_sync_time;

    /* control thread timing statistics */
    void update_timing_stats(); // called inside control loop

    void reset_timing_stats(); // called outside control loop
    void print_timing_stats(); // called outside control loop

    static uint64_t get_time_usec();

    uint64_t timing_last_exec;
    int timing_max_latency;
    int timing_min_latency;
    double timing_avg_latency;

    ros::Time last_status_print_time;

};

}

 #endif // __gambit_driver_arm_h_

