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
 * @file   text_input.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class for text input and output and creating folder and directories based on text entries. Important for human-machine-interface.
 */

#include "text_input/text_input.h"

std::string TextInput::get_working_dir(std::string default_url)
{
    std::string input_url;
    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Enter the URL for the working directory where all data are saved. \n"
                 "If you want to use the default URL (" << default_url <<"), enter '-'\n\n";
    std::cin >> input_url;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";

    //check whether the given directory exists and if not, create directory
    if(input_url == "-"){
        boost::filesystem::path dir = boost::filesystem::absolute(default_url);
        if(!boost::filesystem::exists(dir)){
            boost::filesystem::create_directories(dir);
        }
        return default_url;
    }
    else {
        boost::filesystem::path dir = boost::filesystem::absolute(input_url);
        if(!boost::filesystem::exists(dir)){
            boost::filesystem::create_directories(dir);
        }
        return input_url;
    }
}

void TextInput::create_data_dir(std::string working_dir)
{
    std::string data_dir = working_dir + "/data";
    boost::filesystem::path dir = boost::filesystem::absolute(data_dir);
    if(!boost::filesystem::exists(dir)){
        boost::filesystem::create_directories(dir);
        std::cout << "Created direction: " << data_dir << "\n";
    }
}

void TextInput::create_pointcloud_dir(std::string working_dir)
{
    std::string pointclouds_dir = working_dir + "/data/pointclouds";
    boost::filesystem::path dir = boost::filesystem::absolute(pointclouds_dir);
    if(!boost::filesystem::exists(dir)){
        boost::filesystem::create_directories(dir);
        std::cout << "Created direction: " << pointclouds_dir << "\n";
    }
}


void TextInput::delete_rtabmap_db(std::string database_path)
{
    boost::filesystem::path dir = boost::filesystem::absolute(database_path);
    if(boost::filesystem::exists(dir)){
        boost::filesystem::remove(dir);
        std::cout << "\nRemoved file: " << database_path << "\n";
    }
    else {
        std::cout << "\nError removing file: " << database_path << "\n";
    }
}


std::string TextInput::get_first_step()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Enter 'M' to record a new map.\n"
                  << "Enter 'L' to add a linkpoint on an existing map and then abort.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "M" && input != "L"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::get_mapping_start()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Bring the robot in position for Mapping.\n"
                  << "Enter 'S' to start mapping. \n"
                  << "Enter 'M' to go to the main menu.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "S" && input != "M"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::get_mapping_end()
{
    std::string input;
    bool valid;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << '\n' << "Now map the room by moving the robot.\n"
             << "When you finished mapping, enter 'E' to end the mapping session with rtabmap and save the map.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "E"){
            valid = false;
            std::cout << "Input wrong. Please try again.";
        }
        else {
            valid = true;
            return input;
        }
    }
}


int TextInput::get_start_map_index()
{
    int index;
    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Enter the index for the next MAP. \n"
              << "If you start completely new it is '0'. \n"
              << "If there was a previous session enter [index of the last map] + 1.\n";

    std::cin >> index;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return index;
}


int TextInput::get_current_map_index()
{
    int index;
    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Enter the index for the current MAP. \n";

    std::cin >> index;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return index;
}


int TextInput::get_map_index_lp_mode()
{
    int index;
    std::cout << "\n-----------------------------------------------\n"
              << "You are in linkpoint only mode now.\n"
              << "Here you can add as may linkpoints on one map as you want, then the program ends.\n"
              << "Enter the id of the map you want to add linkpoints for.\n";
    //TODO: let the program check the linkpoints.db automatically for the last index
    std::cin >> index;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return index;
}


int TextInput::get_start_linkpoint_index()
{
    int index;
    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Enter the index for the next LINKPOINT. \n"
              << "If you start completely new it is '0'. \n"
              << "If there was a previous session enter [index of the last linkpoint] + 1.\n";
    //TODO: let the program check the linkpoints.db automatically for the last index
    std::cin >> index;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return index;
}


std::string TextInput::get_map_name()
{
    std::string name;
    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Enter the name for the next map.\n "
                 "The map will be stored in the working directory in '/data' under the name [map_index]_[map_name].\n";
    std::cin >> name;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return name;
}


bool TextInput::get_indoor_info()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Is this an indoor map? \n"
                     "Enter 'I' for indoor map;\n"
                     "Enter 'O' for outdoor map.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "I" && input != "O"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.\n";
        }
        else {
            valid = true;
            if(input == "I"){
                return true;
            }
            else {
                return false;
            }
        }
    }
}


std::string TextInput::get_restart_mapping()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Enter 'S' to to save the map.\n"
                  << "Enter 'N' to record the map again.\n"
                  << "Enter '-' to abort mapping.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "S" && input != "N" && input != "-"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.\n";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::ask_for_linkpoints()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Enter 'L' to to save a linkpoint on the prevoiusly created map.\n"
                  << "Enter 'M' to record a new map.\n"
                  << "Enter '-' to abort.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "L" && input != "M" && input != "-"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.\n";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::ask_for_linkpoint_reached()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Enter 'S' if you have reached the Linkpoint and want to save it.\n"
                  << "Enter 'M' to record a new map.\n"
                  << "Enter '-' to abort.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "S" && input != "M" && input != "-"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong. Please try again.\n";
        }
        else {
            valid = true;
            return input;
        }
    }
}


int TextInput::get_difficulty()
{
    int input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n";
        std::cout << "Enter the difficulty [0-10] for the linkpoint (integer).\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input < 0 && input > 10){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong or out of range. Please try again.\n";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::check_lp_existance()
{
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n"
                     "Does this linkpoint already exist on another map?.\n"
                     "Enter 'N' if the Linkpoint is new.\n"
                     "Enter 'tag' to provide the tag of the linkpoint.\n"
                     "Enter 'id' to provide the id of the linkpoint\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "N" && input != "tag" && input != "id"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong or out of range. Please try again.";
        }
        else {
            valid = true;
            return input;
        }
    }
}


std::string TextInput::get_lp_tag()
{
    std::string input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the tag of the linkpoint.\n";
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}


int TextInput::get_existing_lp_id()
{
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the id of the linkpoint.\n";
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}


int TextInput::get_existing_map_id()
{
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the id of the map.\n";
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}

int TextInput::get_new_map_id(std::string url)
{
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the id of the new map.\n"
                 "Already existing map ids are:\n";

    TextInput::print_ids(linkpoint_db_.get_map_ids(url));
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}

int TextInput::get_new_linkpoint_id(std::string url)
{
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the id of the new linkpoint.\n"
                 "Already existing linkpoint ids are:\n";

    TextInput::print_ids(linkpoint_db_.get_linkpoint_ids(url));
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}

int TextInput::get_new_linkpoint_candidate_id(std::string url)
{
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Enter the id of the new linkpoint candidate.\n"
                 "Already existing linkpoint candidate ids are:\n";

    TextInput::print_ids(linkpoint_db_.get_lp_candidate_ids(url));
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}

int TextInput::get_initial_pose_lp(){
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Set the INITIAL POSE as a linkpoint (if necessary):\n"
                 "If you want to set an initial pose, enter the LINKPOINT ID.\n"
                 "If you don't want to set an initial pose enter -1!\n";
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}

int TextInput::get_lp_candidate_id(){
    int input;
    std::cout << "\n-----------------------------------------------\n"
                 "Is this linkpoint in LOOP CLOSURE with a linkpoint candidate?:\n"
                 "If YES, set the ID if the linkpoint candidate.\n"
                 "If NO, enter -1!\n";
    std::cin >> input;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "\n-----------------------------------------------\n";
    return input;
}


std::string TextInput::main_menu(){
    std::string input;
    bool valid = false;
    while(!valid){
        std::cout << "\n-----------------------------------------------\n"
                     "### MAIN MENU ###\n"
                     "Enter 'N' if you want to start a NEW MAP.\n"
                     "Enter 'O' if you want to add information on an OLD MAP.\n"
                     "Enter 'DM' to DELETE MAPS in the databse.\n"
                     "Enter 'DL' to DELETE LINKPOINTS in the database.\n"
                     "Enter '-' to ABORT.\n";
        std::cin >> input;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "\n-----------------------------------------------\n";

        if(input != "N" && input != "O" && input != "DM" && input != "DL" && input != "-"){
            valid = false;
            std::cout << "Input was: " << input << "\n";
            std::cout << "Input wrong or out of range. Please try again.\n";
        }
        else {
            valid = true;
            return input;
        }
    }
}

void TextInput::print_ids(std::vector<int> ids){
    for (int i = 0; i < ids.size(); i++) {
        std::cout << ids.at(i) << "\n";
    }
}
