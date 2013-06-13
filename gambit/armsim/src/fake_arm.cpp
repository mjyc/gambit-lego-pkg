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
 * Actuator group for fake arm DOFs.
 */

#include <armsim/fake_arm.h>

namespace gambit_driver {

FakeArm::FakeArm() {
    ActuatorDummy *m1 = new ActuatorDummy();
    ActuatorDummy *m2 = new ActuatorDummy();
    ActuatorDummy *m3 = new ActuatorDummy();

    DOF *j1 = new DOF("waist");
    j1->set_limits(-M_PI, M_PI);
    j1->add_actuator((Actuator*)m1, 1.0);
    j1->set_continuous_rotation(true);
    m1->add_DOF(j1, 1.0);

    DOF *j2 = new DOF("shoulder");
    j2->set_limits(-M_PI, M_PI);
    j2->add_actuator((Actuator*)m2, 1.0);
    j2->set_continuous_rotation(true);
    m2->add_DOF(j2, 1.0);

    DOF *j3 = new DOF("elbow");
    j3->set_limits(-M_PI, M_PI);
    j3->add_actuator((Actuator*)m3, 1.0);
    j3->set_continuous_rotation(true);
    m3->add_DOF(j3, 1.0);

    actuators.push_back(m1);
    actuators.push_back(m2);
    actuators.push_back(m3);

    DOFs.push_back(j1);
    DOFs.push_back(j2);
    DOFs.push_back(j3);
}

bool FakeArm::check_connectivity() {
    return true;
}

void FakeArm::control() {
    read_status();
    send_control();
}

}


