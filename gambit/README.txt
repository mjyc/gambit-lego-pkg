[2013.06.01]  Explanation of Files in "chess-armbox"

+ robot account
  roscd gambit_driver
  roslaunch launch/arm_mjyc.launch (launches modified file)

+ mjyc account
  ~/ros/stacks/gambit_driver_mjyc :- has modified "model" that is being used from gambit_driver package.
  ~/ros/stacks/gambit_manip :- ?
  ~/ros/stacks/audio_common :- ?

  * I need to still rosnodes from "robot" account for some permission, ros setting issues.

+ from remote machine
  export ROS_MASTER_URI=http://chess-armbox.dyn.cs.washington.edu:11311
  export ROS_IP=128.208.3.253



[2012.06.01]  2012 Summer Gambit Initialization
Terminal1:
  roslaunch armsim arm_sim.launch
Terminal2:
  roslaunch openni_launch openn
Terminal3:
  rosrun rviz rviz
Terminal4:
  roscd kinect_translation_calibration
  /bin/gambit_kinect_calibration input:=/camera/depth_registered/points

After position the virtual axis on landmark axis, hit Ctrl+C
You can now visualize the Gambit in rviz (using Robot Model)

If you want to change the position of landmark axis, you can modify:
  src/gambit_kinect_calibration.cpp
from line 147.




