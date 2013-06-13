#include <stdio.h>
#include <stdlib.h>

#include <armlib/recorded_trajectory.h>

int main(int argc, char **argv) {
    if(argc < 2) {
        printf("usage: %s <filename>\n", argv[0]);
        exit(1);
    }

    armlib::RecordedTrajectory t(argv[1]);
    t.print();

    printf("%f seconds\n", t.get_length());

    return 0;
}

