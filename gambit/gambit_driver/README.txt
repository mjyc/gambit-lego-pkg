


[2013.02.23 Mike Chung]
* robot class modified - some functions became virtual to be able to be modified from lower classes.
* CMakeLists.txt changed to reflect new fuerte requirements.



[2012.04.03 Mike Chung]
* copied this package from "ros-uw/gambit" stack in "robot@chess-laptop.dyn.cs.washington.edu".
* a high-level gambit controller
* compiles fine on Ubuntu 11.10.
* WARNING: not every part has been tested!

CHANGELOG
* removed models/openrave/, models/skteches/ folder
* updated gambit.urdf to point at *.stl files in models/ folder. Previous gambit.urdf file did not worked on my setup.
* removed hybrid.urdf, wrist.urdf, gambit_orig.urdf from urdf/ folder.

UNEXPLORED / TODO LISTs
  * "CMakeLists.txt" has different options for build (drivers for a simulator, realtime, etc... robot)
  * organize urdf/, models/, and launch/ folder.
