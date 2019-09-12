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
 * @file   automated_sm_mapping.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   25.07.2019
 *
 * @brief  State Machine for automated mapping with the environment detection
 */

#ifndef AUTOMATED_SM_MAPPING_H
#define AUTOMATED_SM_MAPPING_H

#include "ros/ros.h"
#include "ros/param.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "string.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_srvs/SetBool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include <stdio.h>
#include <fstream>
#include <ros/package.h>
#include "sqlite3.h"
#include "iostream"
#include <memory>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UtiLite.h>
#include <rtabmap/core/DBDriverSqlite3.h>
#include <rtabmap/core/Transform.h>
#include <dbdriver/dbdriver.h>
#include <linkpoint/linkpoint.h>
#include <boost/filesystem.hpp>
#include <text_input/text_input.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <map_manager/save_linkpoint_pose.h>
#include <tf/transform_listener.h>
#include <map_manager/start_env_detection.h>
#include <pcl/visualization/cloud_viewer.h>
#include <globaltracker/globaltracker.h>
#include <lpgraph/lpgraph.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <sound_play/sound_play.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <numeric>
#include <environment_detector/environment_detector.h>
#include <sensor_msgs/LaserScan.h>
#include <scanutils/scanutils.h>
#include <map_manager/end_env_detection.h>

/**
 * @brief The AutomatedSMMapping class: State Machine for automated mapping with the environment detection
 */
class AutomatedSMMapping
{
public:
    /**
     * Constructor for initializing parameters and callbacks and retrieving parameters from yaml
     * @brief AutomatedSMMapping::AutomatedSMMapping
     * @param node_handle a ROS node handle
     */
    AutomatedSMMapping(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber end_env_detector_subscriber_;
    ros::Subscriber in_out_classifier_subscriber_;
    ros::Subscriber front_cam_depth_subscriber_;
    ros::Subscriber velodyne_scan_subscriber_;

    ros::Publisher start_env_detector_pub_;

    ros::Publisher rtabmap_mapping_indoor_pub_;
    ros::Publisher rtabmap_mapping_outdoor_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Publisher global_pose_pub_;
    ros::Publisher sound_saying_pub_;
    ros::Publisher cur_map_id_pub_;

    ros::ServiceClient set_mode_mapping_service_client_;
    ros::ServiceClient set_mode_localization_service_client_;

    ros::Timer my_timer_;

    // parameters
    int State_Case;
    std::string lp_db_url_; //= "/home/simon/map_manager/linkpoints.db"; //default directory for linkpoints.db
    std::string working_dir_ = "/home/ehlers/map_manager"; //default working directory for saving rtabmap databases
    bool indoor_; //attribute of map: gets information from indoor_msg_
    bool indoor_msg_; //msg from in out classifier
    int map_index_ = -1; //initialized as -1;
    int cur_linkpoint_index_ = -1;
    int loop_closure_map_id_;
    int init_pose_linkpoint_id_;
    std::string cur_map_name_;
    std::string cur_database_path_;
    std::string pointcloud_dir_;
    double cur_localization_pose_x_;
    double cur_localization_pose_y_;
    bool only_lp_mode_;
    bool end_env_detector_;
    bool no_graph_connection_;
    bool on_origin_map_;
    bool use_old_map_;

    //parameters from yaml
    int config_criterion_;
    double range_thr_;
    double ssim_thr_;
    double psnr_thr_;
    double var_x_;
    double var_y_;
    double var_theta_;



    cv::Mat cam_back_depth_msg_;
    std::vector<cv::Mat> last_n_depth_images_;
    bool record_depth_images_;
    int num_depth_imgs_;

    sensor_msgs::LaserScan velodyne_scan_full_;
    geometry_msgs::PoseWithCovarianceStamped init_pose_;
    bool init_pose_published_;

    enum {
      Initialize = 1,
      Start_Mapping = 2,
      Mapping = 3,
      Kill_mapping_mode = 5,
      Save_linkpoint = 6,
      Main_Menu = 7,
      Abort = -1,
      End = -2
    };

    // Services
    std_srvs::SetBool srv_startSM;
    std_srvs::Empty srv_PyramideAnfahren;


    //########## CALLBACKS: SUBSCRIBER IN OUT CLASSIFIER ############################################################################

    /**
     * State Machine for automated mapping:
     * This is the heart of our automated multi-mapping approach.
     * Here the maps are created and saved and the configuration for the new map is determined.
     * The states are:
     * -> Initialize (Loading working directory and create databases)
     * -> Start Mapping (Determining of the new configuration (high ranges or small ranges) and launch of the SLAM mathod (RTAB-Map))
     * -> Mapping (publishing grobal pose and wait for environment detector to stop mapping due to the detection of a door)
     * -> Kill mapping mode (kills all the nodes that are relevant fro mapping)
     * -> Main Menu (Menu for choosing whether to map on a new or old map or to delete map or linkpoints from database)
     * -> Abort (aborts the state machine)
     *
     * @brief AutomatedSMMapping::timerCallback
     * @param evt (const ros::TimerEvent)
     */
    void timerCallback(const ros::TimerEvent &evt);

    /**
     * Subsciber Callback for receiving important information for the environment detector
     * after it finished the detection.
     * @brief AutomatedSMMapping::EndEnvDetectorSubscriberCallback
     * @param msg (const map_manager::end_env_detection)
     */
    void EndEnvDetectorSubscriberCallback(const map_manager::end_env_detection &msg);

    /**
     * Callback for receiving the classification from CNN (based on Places365) if the robot is indoor or outdoor
     * @brief AutomatedSMMapping::InOutClassifierCallback
     * @param msg (const std_msgs::String)
     */
    void InOutClassifierCallback(const std_msgs::String &msg);

    /**
     * Subscriber Callback for receiving the depth image from cam front
     * @brief AutomatedSMMapping::FrontCamDepthSubscriberCallback
     * @param image_msg (const sensor_msgs::ImageConstPtr)
     */
    void FrontCamDepthSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     * Subscriber callback for velodyne pointcloud.
     * Used for configuration search based on range-data.
     * @brief AutomatedSMMapping::VelodyneScanSubscriberCallback
     * @param msg (const sensor_msgs::LaserScan)
     */
    void VelodyneScanSubscriberCallback(const sensor_msgs::LaserScan &msg);

    //Objects
    DBDriver linkpoint_db_;
    TextInput text_io_;
    GlobalTracker global_tracker_;
    LPGraph lp_graph_;

    //########## METHODS ############################################################################

    /**
     * get localization pose (map -> base_link) using tf tree with tranform listener.
     * @brief AutomatedSMMapping::get_localization_pose
     * @return tf (tf::Transform)
     */
    tf::Transform get_localization_pose();

    /**
     * Method for publishing text from std::string to /robotsound as sound_msg
     * @brief AutomatedSMMapping::say_text
     * @param text (std::string)
     */
    void say_text(std::string text);

    /**
     * checks if to use the indoor config (small ranges) or the outdoor config (high ranges, retruns false)distances
     * Possible criteria are:
     * config_criterion parameter to set in yaml file: 0 = range-data, 1 = SSIM, 2 = PSNR 3 = indoor/outdoor classifier]
     *
     * 0) based on ranges (bounding box of low-pass filtered range data)
     * 1) SSIM (structural similarity)
     * 2) PSNR (peak signal-noise-ratio)
     * 3) indoor/outdoor classifier (CNN based on places365 database)
     * @brief AutomatedSMMapping::check_if_use_indoor_sensor_config
     * @return bool
     */
    bool check_if_use_indoor_sensor_config();

    /**
     *Method for computing the PSRN peak signal-to-noise-ratio
     * Based on https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#image-similarity-psnr-and-ssim
     * @brief AutomatedSMMapping::getPSNR
     * @param I1 (const cv::Mat)
     * @param I2 (const cv::Mat)
     * @return psnr (double)
     */
    double getPSNR(const cv::Mat& I1, const cv::Mat& I2);

    /**
     * Method for computing the SSIM (structural similarity)
     * Based on https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#image-similarity-psnr-and-ssim
     * @brief AutomatedSMMapping::getMSSIM
     * @param i1 (const cv::Mat)
     * @param i2 (const cv::Mat)
     * @return mssim (cv::Scalar)
     */
    cv::Scalar getMSSIM( const cv::Mat& i1, const cv::Mat& i2);

    /**
     * Method to get the initial pose of the robot based on a linkpoint id and the respective map.
     * @brief AutomatedSMMapping::get_init_pose
     * @param url (std::string)
     * @param init_pose_linkpoint_id (int)
     * @param map_id (int)
     * @return
     */
    geometry_msgs::PoseWithCovarianceStamped get_init_pose(std::string url, int init_pose_linkpoint_id,int  map_id);
};


#endif // AUTOMATED_SM_MAPPING_H
