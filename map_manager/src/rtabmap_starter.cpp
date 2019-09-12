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
 * @file   rtabmap_starter.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Starts RTAB-Map with different configurations for the different maps segmented by doorways. Indoor config is for small distances, outdoor config is for large distances.
 * Is able to start mapping, localization and navigation (with move_base) in both configurations.
 */

#include "rtabmap_starter/rtabmap_starter.h"

//########## CONSTRUCTOR ######################################################################################
RtabmapStarter::RtabmapStarter(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
    // === SUBSCRIBERS ===
    mapping_indoor_subscriber_ = node_->subscribe("/sm_mapping/start_mapping_indoor", 10, &RtabmapStarter::subscriberCallbackMappingIndoor, this);
    localization_indoor_subscriber_ = node_->subscribe("/sm_mapping/start_localization_indoor", 10, &RtabmapStarter::subscriberCallbackLocalizationIndoor, this);
    mapping_outdoor_subscriber_ = node_->subscribe("/sm_mapping/start_mapping_outdoor", 10, &RtabmapStarter::subscriberCallbackMappingOutdoor, this);
    localization_outdoor_subscriber_ = node_->subscribe("/sm_mapping/start_localization_outdoor", 10, &RtabmapStarter::subscriberCallbackLocalizationOutdoor, this);
    rtabmap_only_subscriber_ = node_->subscribe("/sm_mapping/start_rtabmap_only", 10, &RtabmapStarter::subscriberStartRtabmapOnly, this);
    navigation_indoor_subscriber_ = node_->subscribe("/sm_mapping/start_navigation_indoor", 10, &RtabmapStarter::subscriberCallbackNavigationIndoor, this);
    navigation_outdoor_subscriber_ = node_->subscribe("/sm_mapping/start_navigation_outdoor", 10, &RtabmapStarter::subscriberCallbackNavigationOutdoor, this);
    rtabmap_ros_subscriber_ = node_->subscribe("/sm_mapping/start_rtabmap_ros", 10, &RtabmapStarter::subscriberCallbackRtabmapROS, this);
}

//########## CALLBACK: SUBSCRIBER ######################################################################################
void RtabmapStarter::subscriberCallbackMappingIndoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap mapping indoor");
    ros::Duration(0.5).sleep();
    system("roslaunch map_manager rtabmap_mapping_indoor.launch");  //start
}


void RtabmapStarter::subscriberCallbackLocalizationIndoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap localization indoor");
    ros::Duration(0.5).sleep();
    system("roslaunch map_manager rtabmap_localization_indoor.launch");  //start
}


void RtabmapStarter::subscriberCallbackMappingOutdoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap mapping outdoor");
    ros::Duration(0.5).sleep();
    system("roslaunch map_manager rtabmap_mapping_outdoor.launch");  //start
}


void RtabmapStarter::subscriberCallbackLocalizationOutdoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap localization outdoor");
    ros::Duration(0.5).sleep();
    system("roslaunch map_manager rtabmap_localization_outdoor.launch");  //start
}


void RtabmapStarter::subscriberStartRtabmapOnly(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap only");
    ros::Duration(0.5).sleep();
    system("rosrun rtabmap_ros rtabmap");  //start
}


void RtabmapStarter::subscriberCallbackNavigationIndoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap navigation indoor");
    ros::Duration(0.5).sleep();
    system("roslaunch cmr_navigation rtabmap_navigation_indoor.launch");  //start
}


void RtabmapStarter::subscriberCallbackNavigationOutdoor(const std_msgs::Bool &msg)
{
    ROS_INFO("Start rtabmap navigation outdoor");
    ros::Duration(0.5).sleep();
    system("roslaunch cmr_navigation rtabmap_navigation_outdoor.launch");  //start
}

void RtabmapStarter::subscriberCallbackRtabmapROS(const std_msgs::String &msg){
    ROS_INFO("Start rtabmap_ros");
    ros::Duration(0.5).sleep();
    std::string start_rtabmap_ros = "rosrun rtabmap_ros rtabmap _database_path:=" + msg.data;
    const char * start_rtabmap_ros_char = start_rtabmap_ros.c_str();
    system(start_rtabmap_ros_char);
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtabmap_starter");

    ros::NodeHandle node_handle;
    RtabmapStarter rtabmap_starter(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
