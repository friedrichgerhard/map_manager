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
 * @file   environment_detector.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   25.07.2019
 *
 * @brief  Environment detector for detecting doors and environment changes
 * and saving linkpoints and linkpoint candidates
 */

#ifndef ENVIRONMENT_DETECTOR_H
#define ENVIRONMENT_DETECTOR_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <map_manager/start_env_detection.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <dbdriver/dbdriver.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <cmath>
#include <math.h>
#include <sound_play/sound_play.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <graphoptimizer/graphoptimizer.h>
#include <scanutils/scanutils.h>
#include <map_manager/end_env_detection.h>
#include <Eigen/Eigenvalues>
#include <complex>

/**
 * @brief The EnvironmentDetector class: Environment detector for detecting doors and environment changes and saving linkpoints and linkpoint candidates
 */
class EnvironmentDetector
{
public:

    /**
     * Constructor for ROS-node Environment-Detector. Initializing callbacks and member variables. Retrieving parameters from yaml file.
     * @brief EnvironmentDetector::EnvironmentDetector
     * @param node_handle
     */
     EnvironmentDetector(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    //subscriber
    ros::Subscriber velodyne_scan_subscriber_;
    ros::Subscriber start_env_detection_subscriber_;
    ros::Subscriber velodyne_pointcloud_subscriber_;
    ros::Subscriber rear_cam_image_subscriber_;
    ros::Subscriber front_cam_image_subscriber_;
    ros::Subscriber front_cam_depth_subsrciber_;
    ros::Subscriber global_robot_pose_subscriber_;

    //publisher
    ros::Publisher end_env_detector_pub_;
    ros::Publisher debug_image_pub_;
    ros::Publisher debug_pointcloud_pub_;
    ros::Publisher debug_feature_matching_img_pub_;
    ros::Publisher debug_icp_registered_pointcloud_pub_;
    ros::Publisher sound_saying_pub_;
    ros::Publisher debug_scan_pointcloud_pub_;

    //timer
    ros::Timer my_timer_;
    ros::Timer timer_scan_bounding_box_;
    ros::Time first_detection_time_;
    ros::Time env_change_dectect_time_;
    ros::Duration time_threshold_ = ros::Duration(3.0);
    ros::Duration time_threshold_full_detection_;

    //parameters
    bool door_front_detected_;
    int cnt_door_front_detected_;
    bool door_back_detected_;
    int cnt_door_back_detected_;
    bool searching_;
    bool search_started_;
    bool no_graph_connection_;
    bool environment_change_detected_;
    float PI = 3.1415926;
    int cur_map_id_;
    bool indoor_;
    bool loop_closure_detected_;
    std::string cur_pointcloud_path_; //path to current pointcloud of linkpoint candidate
    int lp_candidate_id_;
    int loop_closure_map_id_;
    double loop_closure_search_radius_;
    double loop_closure_search_radius_with_variance_;
    sensor_msgs::LaserScan velodyne_scan_front_;
    sensor_msgs::LaserScan velodyne_scan_left_;
    sensor_msgs::LaserScan velodyne_scan_right_;
    sensor_msgs::LaserScan velodyne_scan_back_;
    sensor_msgs::LaserScan velodyne_scan_full_;
    sensor_msgs::PointCloud2 velodyne_pointcloud_msg_; //for saving messages from subsriber callback
    sensor_msgs::PointCloud2 velodyne_pointcloud_linkpoint_candidate_;

    cv::Mat cam_rear_image_msg_; //for saving messages from subscriber callback
    cv::Mat cam_back_image_msg_;
    cv::Mat cam_back_depth_msg_;
    cv::Mat cam_rear_linkpoint_candidate_;
    cv::Mat cam_back_linkpoint_;

    tf::Transform pose_linkpoint_candidate_;
    tf::Transform world_pose_linkpoint_candidate_;

    //Parameters for laser scan change detector
    tf::Pose previous_scan_pose_;
    tf::Pose global_pose_;
    double mean_scan_left_t_2_;  //mean of the laserscans from prevoius timesteps for computing the derivative
    double mean_scan_left_t_1_;
    double mean_scan_right_t_2_;
    double mean_scan_right_t_1_;

    float x_max_t_1_; //values from previous timesteps
    float x_min_t_1_;
    float y_max_t_1_;
    float y_min_t_1_;

    std::vector<cv::Mat> last_n_images_;
    bool record_images_;
    std::vector<cv::Mat> last_n_rear_images_;
    bool record_rear_images_;
    bool rtabmap_started_;

    //parameters for map
    std::string cur_map_name_;
    std::string database_path_;
    std::string lp_db_url_;
    std::string pointcloud_dir_; //directory to pointclouds folder in map_manager folder

    //params from yaml
    double icp_fitness_score_thr_;
    int number_matches_thr_;
    double deriv_door_thr_;
    double deriv_range_thr_;
    double var_x_;
    double var_y_;
    double var_theta_;
    int low_pass_filter_size_env_;
    int num_last_imgs_;
    float NNDR_akaze_;
    float NNDR_sift_;


    //########## METHODS ############################################################################
    /**
     * Detects door from front by using the laser scan data front provided by the VelodyneScanSubsriberCallback
     * @brief EnvironmentDetector::detect_door_from_front
     * @param msg (sensor_msgs::LaserScan)
     * @return
     */
    bool detect_door_from_front(sensor_msgs::LaserScan msg);

    /**
     * Detects door from back by using the laser scan data rear provided by the VelodyneScanSubsriberCallback
     * @brief EnvironmentDetector::detect_door_from_back
     * @param msg (sensor_msgs::LaserScan)
     * @return
     */
    bool detect_door_from_back(sensor_msgs::LaserScan msg);

    /**
     * Receiving the localization pose from tf tree provided by the SLAM method by using an tf listener.
     * @brief EnvironmentDetector::get_localization_pose
     * @return tf::Transform (transform from map to base_link)
     */
    tf::Transform get_localization_pose();

    /**
     * Receiving the robot position in the world cs provided by the global tracker.
     * Used for preselection of link-points.
     * @brief EnvironmentDetector::get_world_pose
     * @return tf::Transform tf (Transform from world frame to base_link frame)
     */
    tf::Transform get_world_pose();

    /**
     * Method for publishing an image for debug purposes
     * @brief EnvironmentDetector::publish_debug_image
     * @param img (cv::Mat)
     */
    void publish_debug_image(cv::Mat img);

    /**
     * Method for publishing pointcloud in sensor_msgs::PointCloud2 format for debugging purposes.
     * @brief EnvironmentDetector::publish_debug_pointcloud
     * @param pointcloud_path (std::string)
     * @return pc2_msgs (sensor_msgs::PointCloud)
     */
    sensor_msgs::PointCloud2 publish_debug_pointcloud(std::string pointcloud_path);

    /**
     * Converts sensor_msgs::PointCloud2 to pcl::PointCloud.
     * @brief EnvironmentDetector::pc2_to_pcl
     * @param pointcloud (sensor_msgs::PointCloud2)
     * @return temp_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_to_pcl(sensor_msgs::PointCloud2 pointcloud);

    /**
     * Method for computing the number of AKAZE and SIFT feature matches between two images.
     * Used for appearance-based loop closure detection.
     * @brief EnvironmentDetector::get_num_matches_akaze_sift
     * @param id (int)
     * @param candidate_img (cv::Mat)
     * @param cam_img (cv::Mat)
     * @return num_good_matches (int)
     */
    int get_num_matches_akaze_sift(int id, cv::Mat candidate_img, cv::Mat cam_img);

    /**
     * Takes a vector and calculates the median.
     * Used for computing the median of all features.
     * @brief EnvironmentDetector::get_median
     * @param vector (std::vector<int>)
     * @return median (double)
     */
    double get_median(std::vector<int> vector);

    /**
     * Method for computing the transformation between the current robot pose
     * and the saved linkpoint be register the two pointclouds with ICP.
     * @brief EnvironmentDetector::get_transform_from_icp
     * @param pointcloud_velodyne (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     * @param pointcloud_candidate (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     * @return transform (tf::Transform)
     */
    tf::Transform get_transform_from_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_velodyne, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_candidate);

    /**
     * Compute the final transformation for the new pose after the loop closure.
     * @brief EnvironmentDetector::get_loop_closure_pose
     * @param lp_candidate (LinkPoint)
     * @param transform_icp (tf::Transform)
     * @return final_transformation (tf::Transform)
     */
    tf::Transform get_loop_closure_pose(LinkPoint lp_candidate, tf::Transform transform_icp);

    /**
     * Check if a new map has to be created or if the robot just visited an old map.
     * Using appearance-based loop closure detection.
     * Returns the id (int) of if the linkpoint when revisiting an old map.
     * Otherwise it returns -1 (for new map)
     * @brief EnvironmentDetector::check_for_new_map
     * @return loop_closure_lp_id (int)
     */
    int check_for_new_map();

    /**
     * Computes the rotation around z-axis (theta, yaw) based on an quaternion.
     * @brief EnvironmentDetector::theta_from_q
     * @param q (tf::Quaternion)
     * @return yaw (double)
     */
    double theta_from_q(tf::Quaternion q);

    /**
     * Checks, if the linkpoint or linkpoint candiate is within the search radius
     * and so considered for appearance-based loop closure detection
     * @brief EnvironmentDetector::lp_in_search_radius
     * @param url (std::string) URL of linkpoint database
     * @param lp_id (int) id of linkpoint to be checked
     * @param search_radius (double)
     * @param candidate (bool) if id belongs to a linkpoint or a linkpoint candidate
     * @return bool
     */
    bool lp_in_search_radius(std::string url, int lp_id, double search_radius, bool candidate);

    /**
     * Method for saying text with ros sound_play package.
     * Publishes on topic /robotsound
     * @brief EnvironmentDetector::say_text
     * @param text (std::string)
     */
    void say_text(std::string text);

    //########## CALLBACKS ############################################################################
    /**
     * Subscriber callback for velodyne scan data generated by node "pointcloud_to_laserscan".
     * In this callback the laser-scan message is devided into sectors which are then used for door detection.
     * It is also used for the determination of configuration by range data.
     * @brief EnvironmentDetector::VelodyneScanSubscriberCallback
     * @param msg
     */
    void VelodyneScanSubscriberCallback(const sensor_msgs::LaserScan &msg);

    /**
     * Subcriber callback for starting the enviroment detector. Subcribes on topic /environment_detector/start. Retrieves important informations from automated_sm_mapping.
     * @brief EnvironmentDetector::StartEnvDetectionSubscriberCallback
     * @param msg
     */
    void StartEnvDetectionSubscriberCallback(const map_manager::start_env_detection &msg);

    /**
     * Subscriber callback for velodyne pointcloud. Stores message in member variable.
     * @brief EnvironmentDetector::VelodynePointcloudSubscriberCallback
     * @param msg
     */
    void VelodynePointcloudSubscriberCallback(const sensor_msgs::PointCloud2 &msg);

    /**
     * Subscriber callback for recording the last n images for feature matching with an old image saved in database.
     * Used for loop closure purposes.
     * @brief EnvironmentDetector::RearCamImageSubscriberCallback
     * @param image_msg
     */
    void RearCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     * Subscriber callback for recording the last n images for feature matching with an old image saved in database.
     * Used for loop closure purposes.
     * @brief EnvironmentDetector::FrontCamImageSubscriberCallback
     * @param image_msg
     */
    void FrontCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     * Timer callback which is the heart of the environment detector.
     * It is similar to a state machine with different cases:
     * -> search for door front
     * -> search for door back
     * -> reset door detection when overtime
     * -> door detected: save linkpoint or find loop closure
     * -> only door front detected: save linkpoint candidate
     *
     * @brief EnvironmentDetector::timerCallback
     * @param evt
     */
    void timerCallback(const ros::TimerEvent &evt);

    /**
     * Timer callback for
     * 1)low-pass filtering the range data
     * 2) Transform into map CS
     * 3) Determining the bounding box by searching min/max values in x and y
     * 4) Computing the change (derivative) of range data for detecing an environment change
     *
     * @brief EnvironmentDetector::timerCallbackBoundingBox
     * @param evt
     */
    void timerCallbackBoundingBox(const ros::TimerEvent &evt);

    /**
     * Subscriber callback of the global pose published by the GlobalTracker in automated_sm_mapping.
     * Used for the search radius in loop closure detectio.
     * @brief EnvironmentDetector::GlobalPoseSubscriberCallback
     * @param msg
     */
    void GlobalPoseSubscriberCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

    //Objects
    DBDriver linkpoint_db_;
};

#endif // ENVIRONMENT_DETECTOR_H
