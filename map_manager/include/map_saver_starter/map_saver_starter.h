/* *****************************************************************
 *
 * map_manager
 *
 * Copyright (c) 2019
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   map_saver_starter.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Starts RTAB-Map with different configurations for the different maps segmented by doorways. Indoor config is for small distances, outdoor config is for large distances.
 * Is able to start mapping, localization and navigation (with move_base) in both configurations.
 */

#ifndef MAPSAVER_STARTER_H
#define MAPSAVER_STARTER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "stdio.h"
#include "iostream"
#include <unistd.h>

/**
 * @brief The RtabmapStarter class: Starts RTAB-Map with different configurations for the different maps segmented by doorways.
 */
class MapSaverStarter
{
public:

    /**
     * Constructor for map_saver_starter node with initializing subscriber callbacks.
     * @brief RtabmapStarter::RtabmapStarter
     * @param node_handle (ros::NodeHandle)
     */
    MapSaverStarter(ros::NodeHandle &node_handle);

    // node handle
    ros::NodeHandle *node_;

    //ros_communication
    ros::Subscriber map_saver_starter_subscriber_;


    // ########### CALLBACKS #################
    /**
     * Starts map_saver when receiving message
     * @brief MapSaverStarter::subscriberCallback
     * @param msg (const std_msgs::String)
     */
    void subscriberCallback(const std_msgs::String &msg);
};

#endif // RTABMAP_STARTER_H
