#!/bin/bash
source ~/robot_design_lab_ws/devel/setup.bash
# test move base make plan service
# rosrun rdl_turtlebot3_navigation call_move_base_srv_make_plan.sh
# test rdl make plan service
rosservice call /robot_design_lab/make_plan "start:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
goal:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    position:
      x: 1.7
      y: 1.7
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
tolerance: 0.0" 

