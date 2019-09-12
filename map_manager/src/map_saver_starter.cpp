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
 * @file   map_saver_starter.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Starts map_saver for saving map tp .pgm and .yaml
 * Is able to start mapping, localization and navigation (with move_base) in both configurations.
 */

#include "map_saver_starter/map_saver_starter.h"

//########## CONSTRUCTOR ######################################################################################
MapSaverStarter::MapSaverStarter(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
    // === SUBSCRIBERS ===
    map_saver_starter_subscriber_ = node_->subscribe("/map_manager/start_map_saver", 10, &MapSaverStarter::subscriberCallback, this);
}

//########### SUBSCRIBER CALLBACK ##############################################################################
void MapSaverStarter::subscriberCallback(const std_msgs::String &msg){
    ROS_INFO("Start map_saver");
    ros::Duration(0.5).sleep();
    std::string start_map_saver = "rosrun map_server map_saver map:=proj_map -f "+ msg.data;
    const char * start_map_saver_char = start_map_saver.c_str();
    system(start_map_saver_char);
}


//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_saver_starter");

    ros::NodeHandle node_handle;
    MapSaverStarter map_saver_starter(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
