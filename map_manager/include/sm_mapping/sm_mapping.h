#ifndef SM_MAPPING_H
#define SM_MAPPING_H

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
#include <pcl/visualization/cloud_viewer.h>
#include <globaltracker/globaltracker.h>
#include <lpgraph/lpgraph.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <sound_play/sound_play.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <numeric>
#include <sensor_msgs/LaserScan.h>
#include <scanutils/scanutils.h>


class SMMapping
{
public:
  SMMapping(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber velodyne_pointcloud_subscriber_;
    ros::Subscriber rear_cam_image_subscriber_;
    ros::Subscriber front_cam_image_subscriber_;

    ros::Publisher rtabmap_mapping_indoor_pub_;
    ros::Publisher rtabmap_mapping_outdoor_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Publisher global_pose_pub_;
    ros::Publisher sound_saying_pub_;
    ros::Publisher cur_map_id_pub_;

    ros::ServiceClient set_mode_mapping_service_client_;
    ros::ServiceClient set_mode_localization_service_client_;

    ros::ServiceServer set_linkpoint_service_server_;
    ros::ServiceServer set_linkpoint_candidate_service_server_;
    ros::ServiceServer save_linkpoint_rear_image_service_server_;

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
    std::string map_saving_path_;
    double cur_localization_pose_x_;
    double cur_localization_pose_y_;
    bool only_lp_mode_;
    bool end_mapping_;
    bool no_graph_connection_;
    bool on_origin_map_;
    bool use_old_map_;
    sensor_msgs::PointCloud2 velodyne_pointcloud_msg_;
    cv::Mat cam_rear_image_msg_;
    cv::Mat cam_back_image_msg_;
    cv::Mat rear_img_of_lp_;
    geometry_msgs::PoseWithCovarianceStamped world_pose_; 
    tf::Transform local_pose_; 

    //parameters from yaml
    int config_criterion_;
    double range_thr_;
    double ssim_thr_;
    double psnr_thr_;
    double var_x_;
    double var_y_;
    double var_theta_;
    bool use_global_tracker_;
    bool record_imgs_and_pc_;



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
     * State Machine for multi-mapping:
     * This is the heart of our multi-mapping approach. But it is not automated, so linkpoints can be set by services
     * Here the maps are created and saved and the configuration for the new map is determined.
     * The states are:
     * -> Initialize (Loading working directory and create databases)
     * -> Start Mapping (Determining of the new configuration (high ranges or small ranges) and launch of the SLAM mathod (RTAB-Map))
     * -> Mapping (publishing grobal pose and wait for environment detector to stop mapping due to the detection of a door)
     * -> Kill mapping mode (kills all the nodes that are relevant fro mapping)
     * -> Main Menu (Menu for choosing whether to map on a new or old map or to delete map or linkpoints from database)
     * -> Abort (aborts the state machine)
     *
     * @brief SMMapping::timerCallback
     * @param evt (const ros::TimerEvent)
     */
    void timerCallback(const ros::TimerEvent &evt);

    /**
     * Subscriber Callback for receiving the depth image from cam front
     * @brief SMMapping::FrontCamDepthSubscriberCallback
     * @param image_msg (const sensor_msgs::ImageConstPtr)
     */
    void FrontCamDepthSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     * Subscriber callback for velodyne pointcloud.
     * Used for configuration search based on range-data.
     * @brief SMMapping::VelodyneScanSubscriberCallback
     * @param msg (const sensor_msgs::LaserScan)
     */
    void VelodyneScanSubscriberCallback(const sensor_msgs::LaserScan &msg);

    /**
     * Service server to set linkpoint.
     * @brief SMMapping::setLinkpointServiceCallback
     * @param req (std_srvs::Empty::Request)
     * @param res (std_srvs::Empty::Response)
     * @return bool
     */
    bool setLinkpointServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * Service to save image in front of the linkpoint for compatibility to automated_sm_mapping approach.
     * @brief SMMapping::setLinkpointServiceCallback
     * @param req (std_srvs::Empty::Request)
     * @param res (std_srvs::Empty::Response)
     * @return bool
     */
    bool recordLinkpointImageServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * Service to save a linkpoint candidate to the database.
     * @brief SMMapping::setLinkpointCandidateServiceCallback
     * @param req (std_srvs::Empty::Request)
     * @param res (std_srvs::Empty::Response)
     * @return bool
     */
    bool setLinkpointCandidateServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * Callback for receiving Pointcloud from Velodyne.
     * @brief SMMapping::VelodynePointcloudSubscriberCallback
     * @param msg (const sensor_msgs::PointCloud2)
     */
    void VelodynePointcloudSubscriberCallback(const sensor_msgs::PointCloud2 &msg);

    /**
     * callback for receiving rear image for saving in linkpoint.
     * @brief SMMapping::RearCamImageSubscriberCallback
     * @param image_msg (const sensor_msgs::ImageConstPtr)
     */
    void RearCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    /**
     * @brief SMMapping::FrontCamImageSubscriberCallback
     * @param image_msg (const sensor_msgs::ImageConstPtr)
     */
    void FrontCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg);

    //Objects
    DBDriver linkpoint_db_;
    TextInput text_io_;
    GlobalTracker global_tracker_;
    LPGraph lp_graph_;

    //########## METHODS ############################################################################

    /**
     * get localization pose (map -> base_link) using tf tree with tranform listener.
     * @brief SMMapping::get_localization_pose
     * @return tf (tf::Transform)
     */
    tf::Transform get_localization_pose();

    /**
     * get world pose (world -> base_link) using tf tree with tranform listener. (only if use_global_tracker_ = true)
     * @brief SMMapping::get_world_pose
     * @return tf (tf::Transform)
     */
    tf::Transform get_world_pose();

    /**
     * Method for publishing text from std::string to /robotsound as sound_msg
     * @brief SMMapping::say_text
     * @param text (std::string)
     */
    void say_text(std::string text);

    /**
     * Method to get the initial pose of the robot based on a linkpoint id and the respective map.
     * @brief SMMapping::get_init_pose
     * @param url (std::string)
     * @param init_pose_linkpoint_id (int)
     * @param map_id (int)
     * @return
     */
    geometry_msgs::PoseWithCovarianceStamped get_init_pose(std::string url, int init_pose_linkpoint_id, int map_id);

    /**
     * Converts PointCloud2 to pcl format to save velodyne points in file.
     * @brief SMMapping::pc2_to_pcl
     * @param pointcloud (sensor_msgs::PointCloud2)
     * @return temp_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_to_pcl(sensor_msgs::PointCloud2 pointcloud);
};

#endif // SM_MAPPING_H
