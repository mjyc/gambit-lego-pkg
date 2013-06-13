

[2012.04.03 Mike Chung]
* copied this package from "ros-uw/gambit" stack in "robot@chess-laptop.dyn.cs.washington.edu".
* seems like another a wrapper, or library only type package for communication purposes. (like a coplay package)
  ** clue1: only compiles libraries, executables that are compiled are basic utilities (maybe for debugging?) - see "CMakeLists.txt"
  ** clue2: manifest.xml has <export> tags, again which are often used for wrapper type packages.
* compiles fine on Ubuntu 11.10.
* WARNING: "./bin/test_packet" fails! with "Errors: OVERLOAD OVERHEAT". Don't know what this means.
