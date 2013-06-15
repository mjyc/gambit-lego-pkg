[2012.08.27 mjyc]
HOW TO:
1. Find initial transformation using - uses robotarm not predefined marker positions.
  TERMINAL1
    roslaunch launch/calibrate_new_kinect.launch
  TERMINAL2
    openni_online

2. Fine tune the found transformation with visualizer (repeat couple times to get good transformation)

3. rviz visualization for monitoring calibration process
  rosrun rviz rviz -d rvizconf/gambit_kinect_arm_calibration.vcg


***** For all REMOTE MACHINES, set the MASTER_URI correctly.
  export ROS_MASTER_URI=http://chess-armbox.dyn.cs.washington.edu:11311
  export ROS_IP=128.208.3.253
  (128.208.3.253 = chess-laptop uri one day)

***** The gambit URDF file that gambit_driver calls from gambit_driver_mjyc in chess-armbox is modeled after "gray" gambit finger tip.  Using blue tip will be little bit short.


NOTE
* Still, did NOT test known position calibration
* CANNOT make transformation example work out. Please read my notes in "calibrate_new_kinect.launch" file.
* SO, I changed gambit model urdf file in gambit_driver folder!
    roscd gambit_driver
    emacs urdf/gambit.urdf
and look for <!-- for debug --> tags




[2012.07.22 mjyc]
IDEAS
* Various ideas for increasing accuracy of control
  ** using 3D detected points *directly*, for example, detect points 2D camera and estimate where that points are in 3D space - then use them to match with physical points, e.g., known points in the world. Basically this methods skips use of "ideal corner points". It looks for transformation between phycial corner points and detected points in 2D.
  => Use their SolvePnP function. This should be pretty straight forward.

* Reason why Willow Garage's code is not working
  ** "Turtle arm" has different gripper orientation then that of "Gambit"
  ** I need to make sure "order of virtual cloud" I'm providing is correct one

POSSIBLE WORKS
  1. Make sure I have physical arm that can perform IK. (sanity check)
    * For example, I should be able to go to various (known) positions perfectly.
    * I need to have some understanding what is going on with the robot.
  2. Try out transformation based on
    * Known position / Ideal detected points
    * Then try, known position / Detected points directly
    (maybe use more than 4 corners in future)
  3. Experiment if detection and reaching works okay


[2012.07.21 mjyc]
* Bounding box for the gambit base:
  a. Seems like 8.4cm? or 8.3cm? (the radius of the circle part)
    - tried paraview, and opened arm0.stl
    - opened arm0.stl file and click apply
    - click diplay panel on lower left, then click annotation "show cube axes"
    - also can click "edit" to see what are the limits
    - It says "0.0845344"(y-axis, longer part) or "0.085"(x-axis, shorter part)
  b. The center of /arm0 is actually 0.006m higher from black board where the gambit is mounted.

* 6x7 27mm checkboard
  a. 13mm (left) 14mm (right) - width edge length
  b. 4.15 (top) 5.2mm (bottom) - height edge length



[2012.05.01 Mike Chung]
Terminal1:
  roslaunch armsim arm_sim.launch
Terminal2:
  roslaunch openni_launch openni.launch
Terminal3:
  rosrun rviz rviz
Terminal4:
  roscd kinect_translation_calibration
  ./bin/gambit_kinect_calibration input:=/camera/depth_registered/points

After position the virtual axis on landmark axis, hit Ctrl+C
You can now visualize the Gambit in rviz (using Robot Model)

If you want to change the position of landmark axis, you can modify:
  src/gambit_kinect_calibration.cpp
from line 147.




