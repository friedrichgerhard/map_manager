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
 * @file   dbdriver.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Driver for accessing the database linkpoint.db
 */

#ifndef DBDRIVER_H
#define DBDRIVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "stdio.h"
#include "sqlite3.h"
#include "iostream"
#include <memory>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UtiLite.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/Transform.h>
#include <stdlib.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <linkpoint/linkpoint.h>
#include <tuple>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <rtabmap/core/Compression.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

/**
 * @brief The DBDriver class: Driver for accessing the database linkpoint.db
 */
class DBDriver
{
public:

    /**
     * Method for opening the database for a give url
     *
     * @brief DBDriver::open_database
     * @param url (std::string)
     * @return conn (sqlite3*)
     */
    sqlite3* open_database(std::string url);

    //################### METHODS FOR TABLE LINKPOINTS ########################################################
    /**
     * Creates the tabel LINKPOINTS in the linkpoint.db
     *
     * @brief DBDriver::create_table_linkpoints
     * @param url
     */
    void create_table_linkpoints(std::string url);

    /**
     * Insert linkpoint in sqlite3 databse
     *
     * @brief DBDriver::insert_linkpoint
     * @param url (std::string)
     * @param lp (LinkPoint)
     * @return bool (if insertion succeeded)
     */
    bool insert_linkpoint(std::string url, LinkPoint lp);

    /**
     * Retrieve LinkPoint from a given databse with a certain id.
     *
     * @brief DBDriver::get_linkpoint
     * @param url (std::string)
     * @param id (int)
     * @return LinkPoint
     */
    LinkPoint get_linkpoint(std::string url, int id);

    /**
     * Retrieves an entry of type int from a database based on a specified table, property and id.
     *
     * @brief DBDriver::get_db_property_int
     * @param url (std::string)
     * @param id (int)
     * @param property (std::string)
     * @param table (std::string)
     * @return value (int)
     */
    int get_db_property_int(std::string url, int id, std::string property, std::string table);

    /**
     * Retrieves an entry of type double from a database based on a specified table, property and id.
     *
     * @brief DBDriver::get_db_property_double
     * @param url (std::string)
     * @param id (int)
     * @param property (std::string)
     * @param table (std::string)
     * @return value (double)
     */
    double get_db_property_double(std::string url, int id, std::string property, std::string table);

    /**
     * Retrieves an entry of type text (std::string) from a database based on a specified table, property and id.
     *
     * @brief DBDriver::get_db_property_text
     * @param url (std::string)
     * @param id (int)
     * @param property (std::string)
     * @param table (std::string)
     * @return text (std::string)
     */
    std::string get_db_property_text(std::string url, int id, std::string property, std::string table);

    /**
     * Get id of Linkpoint by providing the tag.
     *
     * @brief DBDriver::get_lp_id_by_tag
     * @param url (std::string)
     * @param tag (std::string)
     * @return linkpoint id (int)
     */
    int get_lp_id_by_tag(std::string url, std::string tag);

    /**
     * Insert linkpoint with images in database.
     * The images can be used for appearance-based loop closure detection.
     *
     * @brief DBDriver::insert_linkpoint_with_images
     * @param url (std::string)
     * @param lp (LinkPoint)
     * @param img_front (cv::Mat)
     * @param img_rear (cv::Mat)
     * @return bool (if succeeded)
     */
    bool insert_linkpoint_with_images(std::string url, LinkPoint lp, cv::Mat img_front, cv::Mat img_rear);

    /**
     * Get image from linkpoint. The cam position (FRONT or REAR) has to be determined.
     *
     * @brief DBDriver::get_linkpoint_image
     * @param url (std::string)
     * @param id (int)
     * @param cam_position (std::string)
     * @return image (cv::Mat)
     */
    cv::Mat get_linkpoint_image(std::string url, int id, std::string cam_position);

    /**
     * Method for deleting linkpoint
     *
     * @brief DBDriver::delete_linkpoint
     * @param url
     * @param id
     */
    void delete_linkpoint(std::string url, int id);

    /**
     * Get all the available linkpoint ids (from table (LINKPOINTS) in a vector.
     * This is used for text_io_.
     *
     * @brief DBDriver::get_linkpoint_ids
     * @param url (std::string)
     * @return linkpoint_ids (std::vector<int>)
     */
    std::vector<int> get_linkpoint_ids(std::string url);

    //################### METHODS FOR TABLE MAPS ########################################################

    /**
     * Method for creating the table maps in database
     *
     * @brief DBDriver::create_table_maps
     * @param url (std::string)
     */
    void create_table_maps(std::string url);

    /**
     * Inserting map in database with name and path for the given url.
     *
     * @brief DBDriver::insert_map
     * @param url (std::string)
     * @param map_id (int)
     * @param name (std::string)
     * @param path (std::string)
     * @param indoor (bool)
     */
    void insert_map(std::string url, int map_id, std::string name, std::string path, bool indoor);

    /**
     * Inserts the property (string) for the corresponding strands_db in this entry.
     * Comment: Somehow, the UPDATE feature in sqlite3 did not work, so the old row is deleted and a new one inserted.
     *
     * @brief DBDriver::insert_strandsDB_into_map
     * @param url (std::string) to the database
     * @param map_id (int)
     * @param strands_db (std::string) Information (name or path) which should be saved.
     */
    void insert_strandsDB_into_map(std::string url, int map_id, std::string strands_db);

    /**
     * Retrieve map troperty text (std::string).
     *
     * @brief DBDriver::get_map_property_text
     * @param url (std::string)
     * @param id (int)
     * @param property (std::string)
     * @return entry (std::string)
     */
    std::string get_map_property_text(std::string url, int map_id, std::string property);

    /**
     * Retrieve map property int.
     *
     * @brief DBDriver::get_map_property_int
     * @param url (std::string)
     * @param id (int)
     * @param property (std::string)
     * @return value (int)
     */
    int get_map_property_int(std::string url, int map_id, std::string property);

    /**
     * Deletes the map entry for the given id (int).
     *
     * @brief DBDriver::delete_map
     * @param url (std::string)
     * @param map_id (int)
     */
    void delete_map(std::string url, int map_id);

    /**
     * Get all the available map ids (from table (MAPS) in a vector.
     * This is used for text_io_.
     *
     * @brief DBDriver::get_map_ids
     * @param url (std::string)
     * @return map_ids (std::vector<int>)
     */
    std::vector<int> get_map_ids(std::string url);

    //################### METHODS FOR TABLE LINKPOINT CANDIDATES #################
    /**
     * Creates the table LINKPOINTCANDIDATES in the databse.
     *
     * @brief DBDriver::create_table_linkpoint_candidates
     * @param url (std::string)
     */
    void create_table_linkpoint_candidates(std::string url);

    /**
     * Insert linkpoint candiate into table LINKPOINTCANDIDATES
     *
     * @brief DBDriver::insert_linkpoint_candidate
     * @param url (std::string)
     * @param lp (LinkPoint)
     * @param img (cv::Mat) image for following loop closure detection
     * @param pointcloud_path (std::string) path to external stored pcl pointcloud
     * @return bool if succeded
     */
    bool insert_linkpoint_candidate(std::string url, LinkPoint lp, cv::Mat img, std::string pointcloud_path);

    /**
     * Retrieve link point candidate from database.
     *
     * @brief DBDriver::get_linkpoint_candidate
     * @param url (std::string)
     * @param id (int)
     * @return LinkPoint
     */
    LinkPoint get_linkpoint_candidate(std::string url, int id);

    /**
     * Retrieve image from linkpoint candidate.
     *
     * @brief DBDriver::get_linkpoint_candidate_image
     * @param url (std::string)
     * @param id (int)
     * @return image (cv::Mat)
     */
    cv::Mat get_linkpoint_candidate_image(std::string url, int id);

    /**
     * Retrieve pointcloud from linkpoint cadidate in table LINKPOINTCANDIDATES.
     * Note, the poincloud is not saved in the sqlite3 database but external.
     *
     * @brief DBDriver::get_linkpoint_candidate_pointcloud
     * @param url (std::string)
     * @param id (int)
     * @return pointcloud_ptr (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_linkpoint_candidate_pointcloud(std::string url, int id);

    /**
     * Deletes linkpoint candidates entry in table LINKPOINTCONDIDATES.
     *
     * @brief DBDriver::delete_linkpoint_candidate
     * @param url (std::string)
     * @param id (int)
     */
    void delete_linkpoint_candidate(std::string url, int id);

    /**
     * Get all the available linkpoint candidates ids (from table (LINKPOINTCANDIDATES) in a vector.
     * This is used for loop closure detection.
     *
     * @brief DBDriver::get_lp_candidate_ids
     * @param url (std::string)
     * @return lp_candidate_ids (std::vector<int>)
     */
    std::vector<int> get_lp_candidate_ids(std::string url);


    //###### UTILITY METHODS #############
    /**
     * Compresses the given image for insertion in sqlite3 database.
     * Code from rtabmap corelib/src/Compression.cpp line 102
     * @brief DBDriver::compress_image
     * @param image
     * @return compressed image (cv::Mat)
     */
    cv::Mat compress_image(cv::Mat img);

    /**
     * Converts a cv::Mat into a pcl pointcloud.
     *
     * @brief DBDriver::cv_mat_to_pcl
     * @param openCVpointcloud (cv::Mat)
     * @return point_cloud_ptr (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cv_mat_to_pcl(cv::Mat openCVpointcloud);

    /**
     * Returns the numer of rows in the table, which can be used for setting the id for a new entry.
     *
     * @brief DBDriver::get_number_of_rows
     * @param url (std::string)
     * @param table (std::string)
     * @return cnt_rows (int)
     */
    int get_number_of_rows(std::string url, std::string table);

};

#endif // DBDRIVER_H
