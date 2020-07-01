# rdl_rgb_obstacle_detection

A simple RGB based obstacle detection using opencv

## installation

    sudo apt install ros-melodic-rqt-reconfigure ros-melodic-cv-bridge ros-melodic-rosbridge-server


## running

  roslaunch rdl_rgb_obstacle_detection image_processing.launch

## troubleshoot

Empty dynamic reconfigure when doing `rosrun rqt_reconfigure rqt_reconfigure`

from: https://github.com/ros-visualization/rqt_reconfigure/issues/4

do:

    rosrun rqt_gui rqt_gui --clear-config

and then go plugin -> configuration -> Dynamic reconfigure

close rqt_gui and try again: `rosrun rqt_reconfigure rqt_reconfigure`
