#!/bin/sh

rosservice call /arm/move_object -- 0.3 -0.2 0.11 0.5 0.0 0.11
rosservice call /arm/move_object -- 0.3 -0.2 0.11 0.5 0.0 0.16
rosservice call /arm/move_object -- 0.3 -0.2 0.11 0.5 0.0 0.20

