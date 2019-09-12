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
 * @file   linkpoint.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class to set, store and retrieve information of linkpoints or linkpoint-candidates.
 */

#ifndef LINKPOINT_H
#define LINKPOINT_H

#include "stdio.h"
#include "sqlite3.h"
#include "iostream"
#include "tf/tf.h"

/**
 * @brief The LinkPoint class: Class to set, store and retrieve information of linkpoints or linkpoint-candidates.
 */
class LinkPoint
{
public:

    //Parameters
    int id_;
    std::string tag_ = "";
    int map_id_0_ = -1; // set to -1 to check whether map id was set or not
    float pos_x_0_;
    float pos_y_0_;
    float theta_0_;
    int map_id_1_ = -1;
    float pos_x_1_;
    float pos_y_1_;
    float theta_1_;
    float pos_x_world_;
    float pos_y_world_;
    float theta_world_;
    int direction_;
    int difficulty_ = -1;
    int environment_ = -1;

    // ######## METHODS ###############

    /**
     * @brief LinkPoint::set_id
     * @param id (int)
     */
    void set_id(int id);

    /**
     * @brief LinkPoint::set_tag
     * @param tag (std::string)
     */
    void set_tag(std::string tag);

    /**
     * @brief LinkPoint::set_map_id
     * @param map_id (int)
     * @param index (int) index where to store the information (slot 0 or 1)
     */
    void set_map_id(int map_id, int index);

    /**
     * @brief LinkPoint::set_pos_x
     * @param pos_x (float)
     * @param index (int) index where to store the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     */
    void set_pos_x(float pos_x, int index);

    /**
     * @brief LinkPoint::set_pos_y
     * @param pos_y (float)
     * @param index (int) index where to store the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     */
    void set_pos_y(float pos_y, int index);

    /**
     * @brief LinkPoint::set_theta
     * @param theta (float)
     * @param index (int) index where to store the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     */
    void set_theta(float theta, int index);

    /**
     * @brief LinkPoint::set_theta_from_z_w
     * @param z (float) from Quaternion
     * @param w (float) from Quaternion
     * @param index (int) index where to store the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     */
    void set_theta_from_z_w(float z, float w, int index);

    /**
     * @brief LinkPoint::set_direction
     * @param direction (int) direction in which the robot was oriented while saving the linkpoint. Relevant for theta.
     */
    void set_direction(int direction);

    /**
     * @brief LinkPoint::set_difficulty
     * @param difficulty (int)
     */
    void set_difficulty(int difficulty);

    /**
     * @brief LinkPoint::set_environment
     * @param environment (int) (0 = outdoor, 1 = indoor)
     */
    void set_environment(int environment);


    //Get properties for convenience
    /**
     * @brief LinkPoint::get_id
     * @return id_ (int)
     */
    int get_id();

    /**
     * @brief LinkPoint::get_tag
     * @return  tag_ (std::string)
     */
    std::string get_tag();

    /**
     * @brief LinkPoint::get_map_id
     * @param index (int) dtermines whether to get the map id of slot 0 or slot 1
     * @return map_id (int)
     */
    int get_map_id(int index);

    /**
     * @brief LinkPoint::get_pos_x
     * @param index (int) index where to receive the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     * @return  pos_x (float)
     */
    float get_pos_x(int index);

    /**
     * @brief LinkPoint::get_pos_y
     * @param index (int) index where to receive the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     * @return (float) pos_y
     */
    float get_pos_y(int index);

    /**
     * @brief LinkPoint::get_theta
     * @param index (int) index where to receive the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     * @return theta (float)
     */
    float get_theta(int index);

    /**
     * @brief LinkPoint::get_q_z_from_theta
     * @param index (int) index where to receive the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     * @return q_z (float) z entry of Quaternion based on theta
     */
    float get_q_z_from_theta(int index);

    /**
     * @brief LinkPoint::get_q_w_from_theta
     * @param index (int) index where to receive the information (0 => belongs to map_id_0, 1 => belongs to map_id_1, 2 => belongs to global position)
     * @return q_w (float) w entry of Quaternion based on theta
     */
    float get_q_w_from_theta(int index);

    /**
     * @brief LinkPoint::get_direction
     * @return direction_ (int)
     */
    int get_direction();

    /**
     * @brief LinkPoint::get_difficulty
     * @return difficulty_ (int)
     */
    int get_difficulty();

    /**
     * @brief LinkPoint::get_environment
     * @return environment_ (int)
     */
    int get_environment();

    /**
     * @brief LinkPoint::print_linkpoint_properties
     */
    void print_linkpoint_properties();

};

#endif // LINKPOINT_H
