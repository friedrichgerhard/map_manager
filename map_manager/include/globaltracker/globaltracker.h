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
 * @file   global_tracker.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class for providing the feature to track the global robot position over multiple maps. Generally, a tf tree is created by composing the transforms between the map origins.
 *
 */

#ifndef GLOBALTRACKER_H
#define GLOBALTRACKER_H

#include <tf/transform_listener.h>
#include <dbdriver/dbdriver.h>
#include <linkpoint/linkpoint.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <array>
#include "ros/ros.h"

/**
 * @brief The GlobalTracker class: Class for providing the feature to track the global robot position over multiple maps. Generally, a tf tree is created by composing the transforms between the map origins.
 */
class GlobalTracker
{
public:

    //parameters
    tf::Transform tf_to_global_origin_;
    geometry_msgs::PoseWithCovariance map_pose_in_world_; //pose with covariance in for map_origin of current map where the robot is at
    double pose_variance_ = 0.003;
    bool rtabmap_status_previous_t_ = false;
    bool rtabmap_status_t_ = false;
    bool rtabmap_started_ = false;
    double var_x_;
    double var_y_;
    double var_theta_;


    //############# METHODS ################################################

    /**
     * Multiplying two transforms to get the new transform to from the world origin to the map_origin the robot is at.
     * @brief GlobalTracker::add_tf_to_global_tf
     * @param global_tf (tf::Transform)
     * @param local_tf (tf::Transform)
     * @return new_global_tf (tf::Transform)
     */
    tf::Transform add_tf_to_global_tf(tf::Transform tf_global, tf::Transform tf_local);

    /**
     * Computes the local tf between the map origins of the connected maps by using the linkpoint.
     *
     * @brief GlobalTracker::get_local_tf_from_lp
     * @param url (std::string)
     * @param id (int) of the linkpoint which connects the maps
     * @param previous_map_id (int) id of the previous map in the tf tree to get the direction of the transform.
     * @return
     */
    tf::Transform get_local_tf_from_lp(std::string url, int id, int previous_map_id);

    /**
     * Converts the transform between the map origins to a geometry_msgs::PoseWithCovariance.
     * Furthermore, the initial assumed covariance is added for composing the transforms to get information about the variance propagation.
     *
     * @brief GlobalTracker::get_local_pose_from_lp
     * @param url (std::string)
     * @param id (int)
     * @param previous_map_id (int)
     * @return local_pose (geometry_msgs::PoseWithCovariance)
     */
    geometry_msgs::PoseWithCovariance get_local_pose_from_lp(std::string url, int id, int previous_map_id);

    /**
     * This method adds the transformation from previous map origin to next map origin.
     * Composing the transformations with uncertainty.
     * @brief GlobalTracker::add_new_element_to_tf_path
     * @param url (std::string)
     * @param linkpoint_id (int)
     * @param previous_map_id (int)
     * @return tf_to_global_origin_ (tf::Transform)
     */
    tf::Transform add_new_element_to_tf_path(std::string url, int linkpoint_id, int previous_map_id);

    /**
     * Retrieves the tarnsformation from map to base_link on the current map.
     *
     * @brief GlobalTracker::get_local_tf
     * @return tf (tf::Transform)
     */
    tf::Transform get_local_tf();

    /**
     * Builds the tf tree from world origin to map origin the robot is at.
     *
     * @brief GlobalTracker::build_global_tf_tree
     * @param url (std::string)
     * @param topological_path (std::vector<int>) vector with linkpoint ids for the shortest path from origin to the robot.
     * @param variances (std::vector<double>) Vector containing the variances for x, y and theta
     * @return
     */
    tf::Transform build_global_tf_tree(std::string url, std::vector<int> topological_path, std::vector<double> variances);

    /**
     * Computes the global pose of the robot: Transform from world origin -> base_link with covariance.
     * @brief GlobalTracker::get_global_pose
     * @return global_robot_pose_msg (geometry_msgs::PoseWithCovarianceStamped)
     */
    geometry_msgs::PoseWithCovarianceStamped get_global_pose();

    /**
     * Computes the local pose of the robot: Transform from map origin -> base_link with covariance.
     * @brief GlobalTracker::get_local_pose
     * @return
     */
    geometry_msgs::PoseWithCovarianceStamped get_local_pose();

    /**
     * Tf-broadcaster for adding tf from world -> map to the tf tree.
     * @brief GlobalTracker::broadcast_tf_from_world_origin_to_current_map
     */
    void broadcast_tf_from_world_origin_to_current_map();

    /**
     * Initializes the Global Tracker.
     * @brief GlobalTracker::initialize
     */
    void initialize();

    /**
     * Prints out the transform for debugging purposes.
     * @brief GlobalTracker::print_tf_transform
     * @param transform (tf::Transform)
     */
    void print_tf_transform(tf::Transform transform);

    /**
     * Prints out the pose for debugging purposes.
     * @brief GlobalTracker::print_pose
     * @param pose (geometry_msgs::PoseWithCovariance)
     */
    void print_pose(geometry_msgs::PoseWithCovariance pose);

    /**
     * Creates a covariance matrix with the variances specified in the yaml params file.
     * @brief GlobalTracker::get_covariance_matrix_diag
     * @return covariance_matrix std::array<double, 36>
     */
    std::array<double, 36> get_covariance_matrix_diag();

    /**
     * Reset bools for RTAB-Map starting.
     * @brief GlobalTracker::reset_rtabmap_start_bools
     */
    void reset_rtabmap_start_bools();

    /**
     * Method for checking if RTAB-Map has started.
     * @brief GlobalTracker::check_if_rtabmap_started
     * @return
     */
    bool check_if_rtabmap_started();

    //objects
    DBDriver linkpoint_db_;
};

#endif // GLOBALTRACKER_H
