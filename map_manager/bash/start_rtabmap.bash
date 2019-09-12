#!/bin/bash

function cpp_func {

    /home/ehlers/catkin_ws/src/cmr_os/map_manager/src/rtabmap_starter.cpp

}
while read line; do
    if [ "$line" == "Bash Command: Start RTAB-MAP mapping indoor" ]; then
        roslaunch map_manager rtabmap_mapping_indoor.launch
    fi

    if [ "$line" == "Bash Command: Start RTAB-MAP localization indoor" ]; then
        roslaunch map_manager rtabmap_localization_indoor.launch
    fi

    if [ "$line" == "Bash Command: Start RTAB-MAP mapping outdoor" ]; then
        roslaunch map_manager rtabmap_mapping_outdoor.launch
    fi

    if [ "$line" == "Bash Command: Start RTAB-MAP localization outdoor" ]; then
        roslaunch map_manager rtabmap_localization_outdoor.launch
    fi
done < <(cpp_func)
