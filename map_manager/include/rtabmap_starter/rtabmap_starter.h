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
 * @file   rtabmap_starter.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Starts RTAB-Map with different configurations for the different maps segmented by doorways. Indoor config is for small distances, outdoor config is for large distances.
 * Is able to start mapping, localization and navigation (with move_base) in both configurations.
 */

#ifndef RTABMAP_STARTER_H
#define RTABMAP_STARTER_H

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
class RtabmapStarter
{
public:

    /**
     * Constructor for rtabmap_starter node with initializing subscriber callbacks.
     * @brief RtabmapStarter::RtabmapStarter
     * @param node_handle (ros::NodeHandle)
     */
    RtabmapStarter(ros::NodeHandle &node_handle);

    // node handle
    ros::NodeHandle *node_;

    //ros_communication
    ros::Subscriber mapping_indoor_subscriber_;
    ros::Subscriber localization_indoor_subscriber_;
    ros::Subscriber mapping_outdoor_subscriber_;
    ros::Subscriber localization_outdoor_subscriber_;
    ros::Subscriber rtabmap_only_subscriber_;
    ros::Subscriber navigation_indoor_subscriber_;
    ros::Subscriber navigation_outdoor_subscriber_;
    ros::Subscriber rtabmap_ros_subscriber_;

    // ########### CALLBACKS #################
    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackMappingIndoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackMappingIndoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackLocalizationIndoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackLocalizationIndoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackMappingOutdoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackMappingOutdoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackLocalizationOutdoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackLocalizationOutdoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberStartRtabmapOnly
     * @param msg (const std_msgs::Bool)
     */
    void subscriberStartRtabmapOnly(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackNavigationIndoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackNavigationIndoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map when receiving message
     * @brief RtabmapStarter::subscriberCallbackNavigationOutdoor
     * @param msg (const std_msgs::Bool)
     */
    void subscriberCallbackNavigationOutdoor(const std_msgs::Bool &msg);

    /**
     * Starts RTAB-Map ROS when receiving message for saving maps to .pgm and .yaml
     * @brief subscriberCallbackRtabmapROS
     * @param msg (const std_msgs::String &msg)
     */
    void subscriberCallbackRtabmapROS(const std_msgs::String &msg);
};

#endif // RTABMAP_STARTER_H
