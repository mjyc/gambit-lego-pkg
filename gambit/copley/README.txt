

[2012.04.03 Mike Chung]
* copied an entire "copley" package from "ros-uw/gambit" stack in "robot@chess-laptop.dyn.cs.washington.edu".
* compiles without problems on Ubuntu 11.10.
* seems like a program that communicates with the robot hardwares? not sure.
  ** clue1: no executable - see CMakeLists.txt
  ** clue2: wrapper style package features - see manifest.xml, <export> tag is usually used for wrapper packages
* WARNING: "./bin/coplayTest" program ends with an error. Not sure if that is okay.
