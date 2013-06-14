

[2013.03.07] NEW ORGANIZATION

  * hand_state: class for the hand state
  * handtracking: functions for doing handtracking-related maskings

  * img_dep_sync_node: functions and classes

  * tableevent.h: class for tableevent, e.g., *static* events
  * tableevent_detection: functions for detecting *static* events, e.g., scene changes
  * tableobject.h: object class - keeps its own / relative properties
  * tablestate.h: state of table

  * tf_utils.h:



[2013.03.02] NOTES ABOUT FOR REVIVING ICRA2013 WORKS

How to run ICRA2013 activity recognition code from scratch

  1) Create parameters
    * run /bin/learn_basetable
    TIP: if the recorded video do not have enough frames (<60), you can restart or loop the recorded video to provide enough data to the program.
    TIP: image is being published at 30fps, so receiver programs does not need to worry about that, and they should read images as fast as they can.
    * run /bin/interactive_mask
    * run /bin/interactive_area
    TIP: double check if they are saved in correct directory. Sometimes error outputs from main program is cryptic due to missing parameter files! which we don't want to happen often!

  2) start scripts
  Terminal1. roscore

  Terminal2. rosparam set /use_sim_time true; openni_offline;

  Terminal3. rosrun tf static_transform_publisher 0.160959 0.287365 0.770709 -0.0236179 0.54977 -0.145079 0.822282 /arm0 /camera_link 100 
    [NOTE] assumes camera position and gambit arm position used durring summer of 2012.

  Terminal4. roslaunch launch/arm_sim.launch
    [NOTE] if the playback video gets looped then arm0 will NOT BE rendered correctly.  It has to do with timestamps being jumped back to past and tf programs gets massed up because of that.

  Terminal5. ./bin/event_main_test
    [NOTE] can set parameters like
        rosparam set /min_objsize_frac 0.003

  Terminal6. rosbag_play -l lego_house_1.bag




OLD FILES ARE IN BACKUP FOLDER & OTHER NOTES
  * nodelet experiments are now all in backups/
  * "convert_pointcloud_imgdep_nodelet.cpp" is for PYTHON PROGRAMS.




HEADER FILES
  * detection: event detections
  * hand_state: not current being used / not implemented
  * hand_state_detector: detect hand related states
  * handtracking: actually have defined handobj here as well

  * img_dep_sync_node: being actively used

  * tableevent.h
  * tableobject.h
  * tableobject_icra2013.h
  * tablestate.h: not being used
  * tablestate_icra2013.h

  * tablestate_visualizer.h: not being used / not implemented
  * tf_utils.h

  note: currently hand_state / tableevent (exeption) / tableobjct / tablestate are just place holders.






[2012.08.27]
* All nodelets in src_test DOES NOT work!
* They are being called in very low fps!
