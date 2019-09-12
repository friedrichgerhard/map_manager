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
 * @file   text_input.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class for text input and output and creating folder and directories based on text entries. Important for human-machine-interface.
 */

#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

#include "stdio.h"
#include "string.h"
#include <iostream>
#include <boost/filesystem.hpp>
#include <limits>
#include <dbdriver/dbdriver.h>

/**
 * @brief The TextInput class: Class for text input and output and creating folder and directories based on text entries.
 */
class TextInput
{
public:

    /**
     * Gets the working directory where the linkpoint.db and averything else is saved.
     * @brief TextInput::get_working_dir
     * @param default_url (std::string)
     * @return input_url (std::string)
     */
    std::string get_working_dir(std::string default_url);

    /**
     * Receives the signal to start mapping.
     * @brief TextInput::get_mapping_start
     * @return input (std::string)
     */
    std::string get_mapping_start();

    /**
     * Receives signal for end mapping.
     * @brief TextInput::get_mapping_end
     * @return input (std::string)
     */
    std::string get_mapping_end();

    /**
     * Gets index for the next map.
     * @brief TextInput::get_start_map_index
     * @return index (int)
     */
    int get_start_map_index();

    /**
     * Get index of the next linkpoint.
     * @brief TextInput::get_start_linkpoint_index
     * @return
     */
    int get_start_linkpoint_index();

    /**
     * Gets the name for the next map.
     * @brief TextInput::get_map_name
     * @return name (std::string)
     */
    std::string get_map_name();

    /**
     * Creates the directory for saving the rtabmap databases and pointclouds.
     * @brief TextInput::create_data_dir
     * @param working_dir (std::string)
     */
    void create_data_dir(std::string working_dir);

    /**
     * Creates the directory for saving pointclouds.
     * @brief TextInput::create_pointcloud_dir
     * @param working_dir (std::string)
     */
    void create_pointcloud_dir(std::string working_dir);

    /**
     * Gets info whether the map is indoor (config small ranges) or outdoor (config high ranges)
     * @brief TextInput::get_indoor_info
     * @return bool (true = indoor, false = outdoor)
     */
    bool get_indoor_info();

    /**
     * Get information about restart mapping.
     * @brief TextInput::get_restart_mapping
     * @return input (std::string)
     */
    std::string get_restart_mapping();

    /**
     * Asks if to save a linkpoint on a previously created map.
     * @brief TextInput::ask_for_linkpoints
     * @return input (std::string)
     */
    std::string ask_for_linkpoints();

    /**
     * Receive signal if an linkpoint is reached.
     * @brief TextInput::ask_for_linkpoint_reached
     * @return input (std::string)
     */
    std::string ask_for_linkpoint_reached();

    /**
     * Asks for the difficulty to reach the linkpoint (e.g. passing a doorway).
     * @brief TextInput::get_difficulty
     * @return input (std::string)
     */
    int get_difficulty();

    /**
     * Checks, if the linkpoint already exists on another map.
     * @brief TextInput::check_lp_existance
     * @return input (std::string)
     */
    std::string check_lp_existance();

    /**
     * Get the id of the linkpoint.
     * @brief TextInput::get_existing_lp_id
     * @return input (int)
     */
    int get_existing_lp_id();

    /**
     * Gets id of the map.
     * @brief TextInput::get_existing_map_id
     * @return input (int)
     */
    int get_existing_map_id();

    /**
     * Gets ID of the new map. Shows also already existing ids.
     * @brief get_new_map_id
     * @return
     */
    int get_new_map_id(std::string url);

    /**
     * Gets ID of the new linkpoint. Shows also already existing ids.
     * @brief get_new_map_id
     * @return
     */
    int get_new_linkpoint_id(std::string url);

    /**
     * Gets ID of the new map. Shows also already existing ids.
     * @brief get_new_map_id
     * @return
     */
    int get_new_linkpoint_candidate_id(std::string url);

    /**
     * Get tag of the linkpoint.
     * @brief TextInput::get_lp_tag
     * @return input (std::string)
     */
    std::string get_lp_tag();

    /**
     * Derletes the RTAB-Map database at the given path.
     * @brief TextInput::delete_rtabmap_db
     * @param database_path (std::string)
     */
    void delete_rtabmap_db(std::string database_path);

    /**
     * Asks for the first step (recors new map or add linkpoint).
     * @brief TextInput::get_first_step
     * @return input (std::string)
     */
    std::string get_first_step();

    /**
     * Gets maps index for only setting linkpoints.
     * @brief TextInput::get_map_index_lp_mode
     * @return index (int)
     */
    int get_map_index_lp_mode();

    /**
     * Gets index for the current map.
     * @brief TextInput::get_current_map_index
     * @return index (int)
     */
    int get_current_map_index();

    /**
     * @brief TextInput::get_initial_pose_lp
     * @return linkpoint_id (int)
     */
    int get_initial_pose_lp();

    /**
     * @brief get_lp_candidate_id
     * @return lp_candidate_id (int)
     */
    int get_lp_candidate_id();

    /**
     * Main menu for mapping with state machine.
     * @brief TextInput::main_menu
     * @return input (std::string)
     */
    std::string main_menu();

    /**
     * Prints out the ids.
     * @brief TextInput::print_ids
     * @param ids (std::vector<int>)
     */
    void print_ids(std::vector<int> ids);

    //objects
    DBDriver linkpoint_db_;
};

#endif // TEXT_INPUT_H
