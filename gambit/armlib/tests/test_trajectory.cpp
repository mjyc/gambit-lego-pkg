#include <armlib/js_linear_trajectory.h>
#include <stdio.h>
#include <math.h>

int main(int argc, char **argv) {
    armlib::js_vect start;
    armlib::js_vect end;
    armlib::js_vect weights;

    start.push_back(-2.0);
    start.push_back(1.0);
    end.push_back(1.0);
    end.push_back(-1.0);

    weights.push_back(1.0);
    weights.push_back(1.0);

    armlib::JSLinearTrajectory t1(start, end, weights, 1.0, 90. * M_PI / 180.0);
    t1.print();

    armlib::JSLinearTrajectory t2(start, end, weights, 2.0, 45. * M_PI / 180.0);
    t2.print();

    for(float f=0.0; f<=t2.get_length(); f+=0.01) {
        armlib::js_vect p;
        t2.evaluate(f, p);
        printf("evaluated at %05.02f: %07.02f %07.02f\n", f, p[0] * 180.0 / M_PI, 
            p[1] * 180.0 / M_PI);
    }
    return 0;
}

