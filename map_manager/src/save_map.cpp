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
 * @file   save_map.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   02.08.2019
 *
 * @brief  ROS-Node to save all the rtabmap maps to.pgm and yaml format
 */

#include "save_map/save_map.h"

//########## CONSTRUCTOR ###############################################################################################
SaveMap::SaveMap(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // === PARAMETERS ===


    // === SUBSCRIBERS ===


    // === PUBLISHERS ===
    rtabmap_ros_pub_ = node_->advertise<std_msgs::String>("/sm_mapping/start_rtabmap_ros", 20);
    start_map_saver_pub_ = node_->advertise<std_msgs::String>("/map_manager/start_map_saver", 20);

    // === SERVICE CLIENTS ===

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.1), &SaveMap::timerCallback, this);

    saving_done_ = false;
}

void SaveMap::timerCallback(const ros::TimerEvent &evt){
    if(!saving_done_){

        //initialize
        working_dir_ = text_io_.get_working_dir(working_dir_);
        lp_db_url_ = working_dir_ + "/linkpoints.db";

        std::vector<int> map_ids = linkpoint_db_.get_map_ids(lp_db_url_);
        text_io_.print_ids(map_ids);

        for (int i = 0; i < map_ids.size(); ++i) {
            std::string map_name = linkpoint_db_.get_map_property_text(lp_db_url_, map_ids.at(i), "NAME");
            int map_id = linkpoint_db_.get_map_property_int(lp_db_url_, map_ids.at(i), "MAP_ID");

            std::string map_saving_path = working_dir_ + "/data/" + std::to_string(map_id) + "_" + map_name;
            std::string database_path = working_dir_ + "/data/" + std::to_string(map_id) + "_" + map_name + ".db";
            std::cout << "map_saving_path: " << map_saving_path << "\n";

            std_msgs::String string_msg;
            string_msg.data = database_path;
            rtabmap_ros_pub_.publish(string_msg);
            ros::Duration(5.0).sleep(); // wait until rtabmap has started

            std_msgs::String start_msg;
            start_msg.data = map_saving_path;
            start_map_saver_pub_.publish(start_msg);
            ros::Duration(3.0).sleep(); // wait until map_saver started

            
            //std::string system_call = "rosrun map_server map_saver map:=proj_map -f "+map_saving_path;
            //std::cout << system_call;
            //const char * system_call_char = system_call.c_str();
            //system(system_call_char);

            std::cout << "rosservice call /publish_map 1 1 0" << "\n";
            system("rosservice call /publish_map 1 1 0");
            ROS_INFO("Map saved");
            ros::Duration(2.0).sleep();

            //std::cout << "PLEASE RUN:\nrosservice call /publish_map 1 1 0\n\nThen confirm with any letter and press ENTER.\n";

            system("rosnode kill /rtabmap");
            ros::Duration(2.0).sleep(); // wait until rtabmap got killed

        }
    ROS_INFO("DONE, saved all maps!");
    }
    saving_done_ = true;
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_map");

    ros::NodeHandle node_handle;
    SaveMap save_map(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
