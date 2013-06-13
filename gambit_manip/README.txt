[2012.07.26 Mike Chung]
NEW structure - basic_manip2
  1. two new services
  2. StatePattern applied

WARNING
  * MoveObject currently only can control the yaw angles of the gripper. Controlling others may cause ik to find bad solutions. (big swing moves)


[2012.06.03 Mike Chung]
HOW TO SET "launch/gambit_kinect_calibration.launch" LAUNCH FILE
  1. run "gambit_kinect_calibration" rountine
  2. use
      rosrun tf tf_echo /camera_rgb_optical_frame /table_plane
	  rosrun tf tf_echo /table_plane /arm0
    to check calibrated static transforms.
  3. write translation and rotation outputs to "launch/gambit_kinect_calibration.launch".
      WARNING, make sure to swap roll and yaw positions when you write outputs down to the launch file - static_transform_publisher and tf_echo uses different orderings of rotation values


[2012.05.13 Mike Chung]
TODOs
* refactor src/basic_manip_node 
  ** a lot of redundant codes - factor them out.


NOTE
* no vision part was ported from chess_manip.
* following error occurs when I ran the code from Ubuntu 11.04 Natty

    /usr/include/boost/thread/pthread/recursive_mutex.hpp:62: boost::recursive_mutex::~recursive_mutex(): Assertion `!pthread_mutex_destroy(&m)' failed.

  It occurs everytime I created Arm object!!! I still don't understand why!
  But the code (Arm object) was running in Ubuntu 11.10. No idea why again...


HOW TO RUN CURRENTLY WORKING CODE
1. (in terminal1)
    roscore

2. (in terminal2)
    rosrun rviz rviz

3. (in terminal3)
    roslaunch armsim arm_sim.launch
    *NOTE: now you can use scripts in armlib package to control the (simulated) robot arm.

4. (in terminal4)
    rosrun gambit_manip basic_manip_node
    *NOTE: use rostopic and rosservice to control the robot arm.
