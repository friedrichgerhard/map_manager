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
 * @file   linkpoint.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class to set, store and retrieve information of linkpoints or linkpoint-candidates.
 */

#include "linkpoint/linkpoint.h"


// ############# METHODS TO SET PROPERTIES ##################

void LinkPoint::set_id(int id){id_ = id;}

void LinkPoint::set_tag(std::string tag){tag_ = tag;}

void LinkPoint::set_map_id(int map_id, int index){
    if(index == 0){
        map_id_0_ = map_id;
    }
    else if (index == 1) {
        map_id_1_ = map_id;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

void LinkPoint::set_pos_x(float pos_x, int index){
    if(index == 0){
        pos_x_0_ = pos_x;
    }
    else if (index == 1) {
        pos_x_1_ = pos_x;
    }
    else if (index == 2) {
        pos_x_world_ = pos_x;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

void LinkPoint::set_pos_y(float pos_y, int index){
    if(index == 0){
        pos_y_0_ = pos_y;
    }
    else if (index == 1) {
        pos_y_1_ = pos_y;
    }
    else if (index == 2) {
        pos_y_world_ = pos_y;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

void LinkPoint::set_theta(float theta, int index){
    if(index == 0){
        theta_0_ = theta;
    }
    else if (index == 1) {
        theta_1_ = theta;
    }
    else if (index == 2) {
        theta_world_ = theta;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

void LinkPoint::set_theta_from_z_w(float z, float w, int index){
    tf::Quaternion q(0,0,z,w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    LinkPoint::set_theta(yaw, index);
}

void LinkPoint::set_direction(int direction){direction_ = direction;}

void LinkPoint::set_difficulty(int difficulty){difficulty_ = difficulty;}

void LinkPoint::set_environment(int environment){environment_ = environment;}


// ############# METHODS TO GET PROPERTIES ##################
int LinkPoint::get_id(){return id_;}

std::string LinkPoint::get_tag(){return tag_;}

int LinkPoint::get_map_id(int index){
    if(index == 0){
        return map_id_0_;
    }
    else if (index == 1) {
        return map_id_1_;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

float LinkPoint::get_pos_x(int index){
    if(index == 0){
        return pos_x_0_;
    }
    else if (index == 1) {
        return pos_x_1_;
    }
    else if (index == 2) {
        return pos_x_world_;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

float LinkPoint::get_pos_y(int index){
    if(index == 0){
        return pos_y_0_;
    }
    else if (index == 1) {
        return pos_y_1_;
    }
    else if (index == 2) {
        return pos_y_world_;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

float LinkPoint::get_theta(int index){
    if(index == 0){
        return theta_0_;
    }
    else if (index == 1) {
        return theta_1_;
    }
    else if (index == 2) {
        return theta_world_;
    }
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";
    }
}

float LinkPoint::get_q_z_from_theta(int index){
    tf::Quaternion q;
    if(index == 0){
        q.setRPY(0, 0, theta_0_);}
    else if (index == 1) {
        q.setRPY(0, 0, theta_1_);}
    else if (index == 2) {
        q.setRPY(0, 0, theta_world_);}
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";}

    return q.z();
}

float LinkPoint::get_q_w_from_theta(int index){
    tf::Quaternion q;
    if(index == 0){
        q.setRPY(0, 0, theta_0_);}
    else if (index == 1) {
        q.setRPY(0, 0, theta_1_);}
    else if (index == 2) {
        q.setRPY(0, 0, theta_world_);}
    else {
        std::cout << "Index ["<<index<<"] out of bounds. Only [0,1] valid.";}

    return q.w();
}

int LinkPoint::get_direction(){return direction_;}

int LinkPoint::get_difficulty(){return difficulty_;}

int LinkPoint::get_environment(){return environment_;}

void LinkPoint::print_linkpoint_properties(){
    fprintf(stdout, "Linkpoint properties:\n"
                    "id: %i \n"
                    "tag: %s \n\n"

                    "map_id_0 = %i\n"
                    "pos_x_0 = %f\n"
                    "pos_y_0 = %f\n"
                    "theta_0 = %f\n\n"

                    "map_id_1 = %i\n"
                    "pos_x_1 = %f\n"
                    "pos_y_1 = %f\n"
                    "theta_1 = %f\n\n"

                    "pos_x_world = %f\n"
                    "pos_y_world = %f\n"
                    "theta_world = %f\n\n"

                    "Difficulty = %i\n"
                    "Environment = %i\n",

                    id_, tag_,
                    map_id_0_, pos_x_0_, pos_y_0_, theta_0_,
                    map_id_1_, pos_x_1_, pos_y_1_, theta_1_,
                    pos_x_world_, pos_y_world_, theta_world_,
                    difficulty_, environment_);
}
