import numpy as np
import os
import time

if __name__ == '__main__':

    xgridSize = 4 
    ygridSize = 4
    #width = 0.0465
    width = 0.051

    offsetx = 0.2
    #offsety = -0.186
    offsety = -4.0 * width

    #for i in range(xgridSize):
    #    for j in range(ygridSize):

    xs = [6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 2, 0, 0, 1, 1, 1]
    ys = [3, 5, 4, 2, 3, 4, 5, 6, 6, 5, 4, 3, 2, 4, 4, 4, 5, 4, 3, 2]
    cs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2]
    
     #    for i in [0,1,2]:
    	# for j in [0,1,2]:
    
    for i in range(len(xs)):
        x = xs[i]
        y = ys[i]
        print cs[i]
        x = offsetx + x * width
        y = offsety + y * width
        # z == 125 : with supporter
        cmdstr = 'rosservice call /manip_object_simple "{yawangle: 3.14159265359, from_x: %f, from_y: %f, from_z: 0.105, type: 7, offview: false}"' % (x, y)
        raw_input("Press any key to execute: " + cmdstr)
        os.system(cmdstr)
