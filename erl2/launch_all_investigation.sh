#!/bin/bash

gnome-terminal --tab --title="sherlock_pkg" -- bash -c "roslaunch sherlock_moveit demo_gazebo.launch 2</dev/null"
gnome-terminal --tab --title="services" -- bash -c "sleep 10; roslaunch erl2 services.launch 2</dev/null"
gnome-terminal --tab --title="ROSPlan" -- bash -c "sleep 10; roslaunch erl2 rosplan.launch 2</dev/null"
