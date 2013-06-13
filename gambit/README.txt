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




