import numpy as np
import os
import time

if __name__ == '__main__':
    offsetx = 0.2
    offsety = -0.186

    xgridSize = 4 
    ygridSize = 4
    width = 0.0465
    #for i in range(xgridSize):
    #    for j in range(ygridSize):
    for i in [0,2]:
	for j in [0,2]:
            x = offsetx + i * width
            y = offsety + j * width
            cmdstr = 'rosservice call /manip_object_simple "{yawangle: 3.14159265359, from_x: %f, from_y: %f, from_z: 0.125, type: 7, offview: false}"' % (
                x, y)
	    raw_input("Press any key to execute: " + cmdstr)
            os.system(cmdstr)
            # time.sleep(0.5)
