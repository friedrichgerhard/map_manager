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
 * @file   save_map.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   02.08.2019
 *
 * @brief  ROS-Node to save all the rtabmap maps top .pgm and yaml format
 */

#ifndef SAVEMAP_H
#define SAVEMAP_H

#include "ros/ros.h"
#include "ros/param.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "text_input/text_input.h"
#include "dbdriver/dbdriver.h"
#include "linkpoint/linkpoint.h"

class SaveMap
{
public:
    SaveMap(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    ros::Timer my_timer_;
    ros::Publisher rtabmap_ros_pub_;
    ros::Publisher start_map_saver_pub_;
    bool saving_done_;
    std::string working_dir_ = "/home/ehlers/map_manager"; //default working directory for saving rtabmap databases
    std::string lp_db_url_;

    //Objects
    DBDriver linkpoint_db_;
    TextInput text_io_;

    /**
     * Timer callback for saving the maps in .pgm and .yaml format
     * @brief MapSaver::timerCallback
     * @param evt (const ros::TimerEvent)
     */
    void timerCallback(const ros::TimerEvent &evt);


};

#endif // SAVEMAP_H
