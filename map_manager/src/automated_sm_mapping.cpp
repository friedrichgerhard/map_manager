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
 * @file   automated_sm_mapping.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   24.07.2019
 *
 * @brief  State Machine for automated mapping with the environment detection
 */

#include "automated_sm_mapping/automated_sm_mapping.h"

//########## CONSTRUCTOR ###############################################################################################
AutomatedSMMapping::AutomatedSMMapping(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // === PARAMETERS ===
    node_->param("map_manager/config_criterion", config_criterion_, 0);
    node_->param("map_manager/range_thr", range_thr_, 10.0);
    node_->param("map_manager/ssim_thr", ssim_thr_, 0.7);
    node_->param("map_manager/psnr_thr", psnr_thr_, 0.0);
    node_->param("map_manager/initial_variance_x", var_x_, 0.003);
    node_->param("map_manager/initial_variance_y", var_y_, 0.003);
    node_->param("map_manager/initial_variance_theta", var_theta_, 0.003);
    node_->param("map_manager/num_depth_imgs", num_depth_imgs_, 5);

    // === SUBSCRIBERS ===
    end_env_detector_subscriber_ = node_->subscribe("/environment_detector/end", 1, &AutomatedSMMapping::EndEnvDetectorSubscriberCallback, this);
    in_out_classifier_subscriber_ = node_->subscribe("/in_out_classifier_node/classification", 1, &AutomatedSMMapping::InOutClassifierCallback, this);
    front_cam_depth_subscriber_ = node_->subscribe("/cam_front/aligned_depth_to_color/image_raw", 1, &AutomatedSMMapping::FrontCamDepthSubscriberCallback, this);
    velodyne_scan_subscriber_ = node_->subscribe("/velodyne_laserScan", 1, &AutomatedSMMapping::VelodyneScanSubscriberCallback, this);

    // === PUBLISHERS ===
    start_env_detector_pub_ = node_->advertise<map_manager::start_env_detection>("/environment_detector/start",20);
    rtabmap_mapping_indoor_pub_ = node_->advertise<std_msgs::Bool>("/sm_mapping/start_mapping_indoor",20);
    rtabmap_mapping_outdoor_pub_ = node_->advertise<std_msgs::Bool>("/sm_mapping/start_mapping_outdoor",20);
    initial_pose_pub_ = node_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/rtabmap/initialpose",20);
    global_pose_pub_ = node_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/map_manager/global_pose",20);
    sound_saying_pub_ = node_->advertise<sound_play::SoundRequest>("/robotsound",20);
    cur_map_id_pub_ = node_->advertise<std_msgs::Int16>("/map_manager/cur_map_id",20);

    // === SERVICE CLIENTS ===
    set_mode_mapping_service_client_ = node_->serviceClient<std_srvs::Empty>("/rtabmap/set_mode_mapping");
    set_mode_localization_service_client_ = node_->serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.1), &AutomatedSMMapping::timerCallback, this);

    State_Case = Initialize;
    end_env_detector_ = false;
    loop_closure_map_id_ = -1;
    no_graph_connection_ = false;
    record_depth_images_ = false;
    init_pose_published_ = false;
    on_origin_map_ = false;
    use_old_map_ = false;
}

//######### DEFINE: GET LOCALIZATION POSE ##########################################################################
tf::Transform AutomatedSMMapping::get_localization_pose()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "base_link";
    std::string sourceFrameID = "map";
    double waitTime = 0.5;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
            ROS_INFO("Got Transform!");
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }
    else{
        ROS_WARN("Transform from '%s' to '%s' not available!",sourceFrameID.c_str(),targetFrameID.c_str());
    }
    tf::Transform tf;
    tf.setRotation(transformStamped.getRotation());
    tf.setOrigin(transformStamped.getOrigin());

    return tf;
}


//########## CALLBACK: SUBSCRIBER END ENVIRONMENT DETECTION ############################################################################
void AutomatedSMMapping::EndEnvDetectorSubscriberCallback(const map_manager::end_env_detection &msg)
{

    // msg.data == -1 -> no loop closure detected, start new map
    if(msg.map_id == -1){
        end_env_detector_ = true;
        loop_closure_map_id_ = -1;
        init_pose_linkpoint_id_ = -1;
        ROS_WARN("End environment detection");
    }
    else if (msg.map_id > -1){
        loop_closure_map_id_ = msg.map_id;
        init_pose_linkpoint_id_ = msg.linkpoint_id;
        end_env_detector_ = true;
    }
    else {
        end_env_detector_ = false;
        ROS_ERROR("Got incorrect message on topic /environment_detector/end");
    }
}

//########## CALLBACK: SUBSCRIBER IN OUT CLASSIFIER ############################################################################
void AutomatedSMMapping::InOutClassifierCallback(const std_msgs::String &msg)
{
    std::string input = msg.data;
    bool indoor_old = indoor_msg_;

    if(input == "indoor"){
        indoor_msg_ = true;
    }
    else{
        indoor_msg_ = false;
    }

    if(indoor_old != indoor_msg_){
        ROS_WARN("Environment change detected from %d to %d (1 = indoor, 0 = outdoor).", indoor_old, indoor_msg_);
    }
}

//########## CALLBACK: SUBSCRIBER FRONT CAM DEPTH IMAGE ############################################################################
void AutomatedSMMapping::FrontCamDepthSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // convert image to cv::Mat
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg);
      cam_back_depth_msg_ = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("Failed to convert image");
    }

    //record 10 images for robust feature detection
    if(record_depth_images_ && last_n_depth_images_.size() <= num_depth_imgs_)
    {
        last_n_depth_images_.push_back(cam_back_depth_msg_);
        ROS_INFO("Push back message in last_n_images!");
    }
    else if(record_depth_images_ && last_n_depth_images_.size() > num_depth_imgs_){
        record_depth_images_ = false;
        ROS_INFO("Got %i depth images!", num_depth_imgs_);
    }
}

//########## CALLBACK: SUBSCRIBER VELODYNE SCAN ############################################################################
void AutomatedSMMapping::VelodyneScanSubscriberCallback(const sensor_msgs::LaserScan &msg)
{
    sensor_msgs::LaserScan scan_full;
    scan_full.header = msg.header;
    scan_full.angle_max = msg.angle_max;
    scan_full.angle_min = msg.angle_min;
    scan_full.angle_increment = msg.angle_increment;
    scan_full.ranges = msg.ranges;

    velodyne_scan_full_ = scan_full;
}

//########## CALLBACK: TIMER (STATE MACHINE) ###########################################################################################
void AutomatedSMMapping::timerCallback(const ros::TimerEvent &evt)
{
    if(ros::ok())
    {
        switch (State_Case)
        {
            // --------------------------------------------------------
            case Initialize:
            {

                ROS_INFO("STATE: Initialize");

                working_dir_ = text_io_.get_working_dir(working_dir_);
                lp_db_url_ = working_dir_ + "/linkpoints.db";
                pointcloud_dir_ = working_dir_ + "/data/pointclouds";
                linkpoint_db_.create_table_linkpoints(lp_db_url_);
                linkpoint_db_.create_table_maps(lp_db_url_);
                linkpoint_db_.create_table_linkpoint_candidates(lp_db_url_);
                text_io_.create_data_dir(working_dir_);
                text_io_.create_pointcloud_dir(working_dir_);
                global_tracker_.initialize();

                //test of sound play
                AutomatedSMMapping::say_text("Initialized");

                State_Case = Main_Menu;
                break;
            }

            // ---------------------------------------------------------
            case Start_Mapping:
            {
                ROS_INFO("STATE: Record_new_map");

                std::string start;
                start = text_io_.get_mapping_start();

                if(start == "M"){
                    State_Case = Main_Menu;
                    break;
                }

                if(start == "S"){
                    cur_map_name_ = "map_" + std::to_string(map_index_);
                    cur_database_path_ = working_dir_ + "/data/" + std::to_string(map_index_) + "_" + cur_map_name_ + ".db";
                    ros::param::set("/rtabmap/rtabmap/database_path", cur_database_path_);
                    std::cout << "Creating database:" << cur_database_path_ << "\n\n";

                    //if map exists already
                    init_pose_published_ = false;
                    if(loop_closure_map_id_ != -1){
                        init_pose_ = AutomatedSMMapping::get_init_pose(lp_db_url_, init_pose_linkpoint_id_, map_index_);
                    }

                    //starting rtabmap
                    std_msgs::Bool msg;
                    msg.data = true;
                    if(AutomatedSMMapping::check_if_use_indoor_sensor_config()){
                        rtabmap_mapping_indoor_pub_.publish(msg); //start rtabmap in mapping mode indoor
                    }
                    else {
                        rtabmap_mapping_outdoor_pub_.publish(msg); //start rtabmap in mapping mode outdoor
                    }

                    AutomatedSMMapping::say_text("I am going to start RTAB MAP now");
                    ros::Duration(5.0).sleep(); // wait 15 sec to let rtabmap start

                    //build tf_to_world_origin completely new based on topological graph
                    if(map_index_ > 0){
                        on_origin_map_ = false;
                        LPGraph graph_global_tracker;
                        graph_global_tracker.build_graph_from_db(lp_db_url_, 0, map_index_);
                        std::vector<int> path_to_world_origin = graph_global_tracker.get_topological_path_from_world_orig_to_robot(map_index_);
                        if(path_to_world_origin.size() > 0){
                            std::vector<double> vec_variances = {var_x_, var_y_, var_theta_};
                            global_tracker_.build_global_tf_tree(lp_db_url_, path_to_world_origin, vec_variances);
                            no_graph_connection_ = false;
                        }
                        else{
                            no_graph_connection_ = true;
                            ROS_WARN("No graph connection");
                        }
                    }
                    else if(map_index_ == 0 && loop_closure_map_id_ == 0){
                        on_origin_map_ = true;
                        }

                    //Insert map entry into linkpoint.db
                    linkpoint_db_.insert_map(lp_db_url_, map_index_, cur_map_name_, cur_database_path_, indoor_);
                    std::string text_saying = "Entry for map with I D " + std::to_string(map_index_) + "inserted into database.";
                    AutomatedSMMapping::say_text(text_saying);
                    ros::Duration(2.0).sleep();

                    //start environment detection
                    map_manager::start_env_detection start_msg;
                    start_msg.start = true;
                    start_msg.map_name = cur_map_name_;
                    start_msg.database_path = cur_database_path_;
                    start_msg.linkpoint_db_path = lp_db_url_;
                    start_msg.pointcloud_dir = pointcloud_dir_;
                    start_msg.map_id = map_index_;
                    start_msg.loop_closure_search_radius = 20;
                    start_msg.no_graph_connection = no_graph_connection_;
                    start_msg.indoor = indoor_;
                    start_env_detector_pub_.publish(start_msg);

                    global_tracker_.reset_rtabmap_start_bools();
                    end_env_detector_ = false;
                    State_Case = Mapping;
                    ROS_INFO("STATE: Mapping");
                    break;
                }
            }

            // ---------------------------------------------------------
            case Mapping:
            {
                ros::Duration(0.2).sleep();

                //update pose for global tracker and publish
                if(on_origin_map_){ //when only the local pose is required
                    geometry_msgs::PoseWithCovarianceStamped local_pose_msg;
                    local_pose_msg = global_tracker_.get_local_pose();
                    global_pose_pub_.publish(local_pose_msg);

                    //broadcast also world frame (which is equal the map frame in this case)
                    tf::Transform world_eq_map_transform;
                    world_eq_map_transform.setOrigin(tf::Vector3(0, 0, 0)); //initialize to origin
                    world_eq_map_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
                    static tf::TransformBroadcaster br;
                    br.sendTransform(tf::StampedTransform(world_eq_map_transform, ros::Time::now(), "world", "map"));

                    if(global_tracker_.check_if_rtabmap_started()){
                        AutomatedSMMapping::say_text("RTABMAP has started, finally. Start now moving the robot.");

                        if(loop_closure_map_id_ != -1 && !init_pose_published_){
                            //setting localization mode, since init pose can only be set in localization mode
                            std_srvs::Empty set_mode_msg;
                            set_mode_localization_service_client_.call(set_mode_msg);
                            ros::Duration(1).sleep();

                            initial_pose_pub_.publish(init_pose_);
                            init_pose_published_ = true;
                            ROS_WARN("Init Pose published!");

                             std::cout << "Init Pose:\n"
                                        << "x: " << init_pose_.pose.pose.position.x << "\n"
                                        << "y: " << init_pose_.pose.pose.position.y << "\n"
                                        << "q_z: " << init_pose_.pose.pose.orientation.z << "\n"
                                        << "q_w: " << init_pose_.pose.pose.orientation.w << "\n\n";

                            ros::Duration(2).sleep();

                            //set mode back to mapping mode
                            set_mode_mapping_service_client_.call(set_mode_msg);
                            ros::Duration(1).sleep();
                            AutomatedSMMapping::say_text("Set initial pose.");

                            global_tracker_.print_pose(init_pose_.pose);
                        }
                   }
                }
                else{
                      geometry_msgs::PoseWithCovarianceStamped global_pose_msg;
                      global_pose_msg = global_tracker_.get_global_pose();
                      global_pose_pub_.publish(global_pose_msg);
                      global_tracker_.broadcast_tf_from_world_origin_to_current_map();
                      if(global_tracker_.check_if_rtabmap_started()){
                          AutomatedSMMapping::say_text("RTABMAP has started, finally. Start now moving the robot.");

                          if(loop_closure_map_id_ != -1 && !init_pose_published_){
                              //setting localization mode, since init pose can only be set in localization mode
                              std_srvs::Empty set_mode_msg;
                              set_mode_localization_service_client_.call(set_mode_msg);
                              ros::Duration(2).sleep();

                              initial_pose_pub_.publish(init_pose_);
                              init_pose_published_ = true;
                              ROS_WARN("Init Pose published!");
                              std::cout << "Init Pose:\n"
                                        << "x: " << init_pose_.pose.pose.position.x << "\n"
                                        << "y: " << init_pose_.pose.pose.position.y << "\n"
                                        << "q_z: " << init_pose_.pose.pose.orientation.z << "\n"
                                        << "q_w: " << init_pose_.pose.pose.orientation.w << "\n\n";

                              ros::Duration(2).sleep();

                              //set mode back to mapping mode
                              set_mode_mapping_service_client_.call(set_mode_msg);
                              ros::Duration(1).sleep();
                              AutomatedSMMapping::say_text("Set initial pose.");

                              global_tracker_.print_pose(init_pose_.pose);
                          }
                     }
                }

		    //publish current map id for rviz graph visualization
		    std_msgs::Int16 map_id_msg;
		    map_id_msg.data = map_index_;
		    cur_map_id_pub_.publish(map_id_msg);

                //wait until environment_detector detected a new map
                if(end_env_detector_){
                  State_Case = Kill_mapping_mode;
                  break;
                }
                else {
                  State_Case = Mapping;
                  break;
                }
            }

            // ---------------------------------------------------------
            case Kill_mapping_mode:
            {
                ROS_INFO("STATE: Kill_mapping_mode");
                if(indoor_){
                    ROS_INFO("Kill rtabmap mapping indoor");

                    //----online mapping ------
                    system("rosnode kill rtabmap/rtabmap points_xyzrgb ekf_se cam_back/rgbd_sync_back cam_front/rgbd_sync_front"); //stop rtabmap in mapping mode
                }
                else {

                    ROS_INFO("Kill rtabmap mapping outdoor");

                    //----online mapping ------
                    system("rosnode kill rtabmap/rtabmap points_xyzrgb ekf_se cam_back/rgbd_sync_velodyne_back cam_front/rgbd_sync_velodyne_front /cam_front/register_velodyne_front /cam_back/register_velodyne_back"); //stop rtabmap in mapping mode with register velodyne nodes
                }
                ROS_INFO("Killed rtabmap mapping, sleep for 6 sec!");
                ros::Duration(6.0).sleep();
                State_Case=Start_Mapping;

                //check if map already exists or not
                if(loop_closure_map_id_ == -1){
                    map_index_ = linkpoint_db_.get_number_of_rows(lp_db_url_, "MAPS");
                    State_Case=Start_Mapping;
                }
                else{ //enter an already existing map
                    map_index_ = loop_closure_map_id_;
                    State_Case=Main_Menu;
                }

            break;
            }


            // --------------------------------------------------------
            case Main_Menu:
                {

                  ROS_INFO("STATE: Main Menu");
                  std::string input;
                  input = text_io_.main_menu();

                  //record new map
                  if(input == "N"){

                      //check if map already exists or not
                      if(loop_closure_map_id_ == -1){
                          map_index_ = linkpoint_db_.get_number_of_rows(lp_db_url_, "MAPS");
                      }
                      else{ //enter a already existing map
                          map_index_ = loop_closure_map_id_;
                      }
                      State_Case = Start_Mapping;
                      use_old_map_ = false;
                  }
                  //mapping on old map
                  else if (input == "O"){
                      map_index_ = text_io_.get_current_map_index();
                      cur_map_name_ = "map_" + std::to_string(map_index_);
                      use_old_map_ = true;

                      State_Case = Start_Mapping;
                  }
                  //delete map
                  else if (input == "DM") {
                      int delete_map_index = text_io_.get_existing_map_id();
                      linkpoint_db_.delete_map(lp_db_url_, delete_map_index);
                      cur_map_name_ = "map_" + std::to_string(delete_map_index);
                      cur_database_path_ = working_dir_ + "/data/" + std::to_string(delete_map_index) + "_" + cur_map_name_ + ".db";
                      text_io_.delete_rtabmap_db(cur_database_path_);

                      State_Case = Main_Menu;
                  }
                  //delete linkpoint
                  else if (input == "DL"){
                      int delete_lp_index = text_io_.get_existing_lp_id();
                      linkpoint_db_.delete_linkpoint(lp_db_url_, delete_lp_index);

                      State_Case = Main_Menu;
                  }
                  //abort
                  else if (input == "-"){
                      State_Case = Abort;
                  }

                  break;
                }
            // --------------------------------------------------------
            case Abort:
            {
                ROS_INFO("STATE: Abort");
                State_Case = End; // end of StateMachine
                break;
            }
        }
    }
}

//########### SAY TEXT ######################
void AutomatedSMMapping::say_text(std::string text){
    sound_play::SoundRequest sound_msg;
    sound_msg.command = 1; //play sound once
    sound_msg.sound = -3; //say
    sound_msg.volume = 1.0;
    sound_msg.arg = text; //text to say
    sound_msg.arg2 = "voice_kal_diphone";

    sound_saying_pub_.publish(sound_msg);
}

//############ CHECK IF USE INDOOR SENSOR CONFIG ####################
bool AutomatedSMMapping::check_if_use_indoor_sensor_config(){
    if(loop_closure_map_id_ == -1 && use_old_map_ == false){ // if entering a new unknown map

    //----------------------------------------------------------------------------
        if(config_criterion_ == 0){ //range-data
            //########## Approach using low pass average filtered scan data ###########
            bool high_distance_detected = false;
            if(velodyne_scan_full_.ranges.size() > 0){

                ScanUtils scan_utils;
            // 1) low pass filter the velodyne_laser_scan message
                sensor_msgs::LaserScan scan_filtered;
                scan_filtered = scan_utils.lowPassAveraging(velodyne_scan_full_, 60);

            // 2) transform the scan points in map coordinate frame
                sensor_msgs::PointCloud scan_cloud = scan_utils.transform_scan_to_base_link_frame(scan_filtered);

            // 3) compute the bounding box parameter x_max/x_min y_max/y_min for map coordinate frame
                float x_max = scan_utils.get_x_max(scan_cloud);
                float x_min = scan_utils.get_x_min(scan_cloud);
                float y_max = scan_utils.get_y_max(scan_cloud);
                float y_min = scan_utils.get_y_min(scan_cloud);

            // 4) check if extreme values are over a certain threshold
                float distance_threshold = range_thr_; //meters
                int cnt = 0;
                if(fabs(x_max) > distance_threshold){
                    cnt++;
                    ROS_WARN("X_MAX over threshold!");
                }

                if(fabs(x_min) > distance_threshold){
                    cnt++;
                    ROS_WARN("X_MIN over threshold!");
                }

                if(fabs(y_max) > distance_threshold){
                    cnt++;
                    ROS_WARN("Y_MAX over threshold!");
                }

                if(fabs(y_min) > distance_threshold){
                    cnt++;
                    ROS_WARN("Y_MIN over threshold!");
                }

                if(cnt >= 2){
                    high_distance_detected = true;
                    ROS_WARN("Large room / high distances detected!");
                    indoor_ = false;
                    return false;
                }
                else{
                    ROS_WARN("Small room detected: Suggest indoor config!");
                    indoor_ = true;
                    return true;
                }

            }
            else{
                ROS_ERROR("No velodyne_scan_full_ for calculation of max/min values found!!");
            }
        }
   //-----------------------------------------------------------------------------
        else if (config_criterion_ == 1) { //ssim

            //fill last_10_depth images with data from subscriber callback
            last_n_depth_images_.clear();
            ROS_INFO("last_n_depth_images_.size() before filling: %i", last_n_depth_images_.size());
            record_depth_images_ = true;
            //wait unti the last 10 images are collected
            ROS_INFO("Collect %i Images now for loop closing!", num_depth_imgs_);
            while(last_n_depth_images_.size() <= num_depth_imgs_){
                ros::spinOnce();
            }
            ROS_INFO("last_n_depth_images_.size() after filling: %i", last_n_depth_images_.size());

            std::vector<double> vec_ssim;
            for(int t = 0; t < last_n_depth_images_.size()-1; t++){
                cv::Mat img1 = last_n_depth_images_[t];
                cv::Mat img2 = last_n_depth_images_[t+1];
                cv::Scalar ssim_scalar = AutomatedSMMapping::getMSSIM(img1, img2);
                vec_ssim.push_back(ssim_scalar[0]);
            }

            double sum_ssim = std::accumulate(vec_ssim.begin(), vec_ssim.end(), 0.0);
            double mean_ssim = sum_ssim / vec_ssim.size();
            std::cout << "mean SSIM:" << mean_ssim << "\n";

            if(mean_ssim < ssim_thr_){
                ROS_WARN("Under SSIM Threshold: High noise in depth image detected! Using OUTDOOR config!");
                indoor_ = false;
                return false;
            }
            else{
                ROS_WARN("Over SSIM Threshold: Low noise in depth image detected! Using INDOOR config!");
                indoor_ = true;
                return true;
            }
        }
   //-----------------------------------------------------------------------------
        else if (config_criterion_ == 2) { //psnr
            //fill last_n_depth images with data from subscriber callback
            last_n_depth_images_.clear();
            ROS_INFO("last_n_depth_images_.size() before filling: %i", last_n_depth_images_.size());
            record_depth_images_ = true;
            //wait unti the last 10 images are collected
            ROS_INFO("Collect n Images now for loop closing!");
            while(last_n_depth_images_.size() <= num_depth_imgs_){
                ros::spinOnce();
            }
            ROS_INFO("last_n_depth_images_.size() after filling: %i", last_n_depth_images_.size());

            //############ Approach using PSNR and SSIM for noise evaluation ##################################
            std::vector<double> vec_psnr;
            for(int t = 0; t < last_n_depth_images_.size()-1; t++){
                cv::Mat img1 = last_n_depth_images_[t];
                cv::Mat img2 = last_n_depth_images_[t+1];
                vec_psnr.push_back(AutomatedSMMapping::getPSNR(img1, img2));
            }

            //calculate mean of PSNR and SSIM
            double sum_psnr = std::accumulate(vec_psnr.begin(), vec_psnr.end(), 0.0);
            double mean_psnr = sum_psnr / vec_psnr.size();
            std::cout << "mean PSNR:" << mean_psnr << "\n";

            if(mean_psnr < psnr_thr_){
                ROS_WARN("Under PSNR Threshold: High noise in depth image detected! Using OUTDOOR config!");
                indoor_ = false;
                return false;
            }
            else {
                ROS_WARN("Over PSNR Threshold: Low noise in depth image detected! Using INDOOR config!");
                indoor_ = true;
                return true;
            }
        }
   //-----------------------------------------------------------------------------
        else if (config_criterion_ == 3) { //indoor/outdoor classifier
            //spin once to update indoor outdoor classifier callback to update the indoor_ member variable
            ros::spinOnce();
            ros::Duration(2).sleep();

            indoor_ = indoor_msg_;
            ROS_INFO("Detected indoor_ = %d! (1=indoor; 0=outdoor", indoor_);
            return indoor_;
        }
    }
    else { //if map already exists, search for used configuration on map
        if(use_old_map_){
              loop_closure_map_id_ = map_index_;
        }
        int indoor_int = linkpoint_db_.get_map_property_int(lp_db_url_, loop_closure_map_id_, "INDOOR");
        if (indoor_int == 1){
            std::cout << "Use INDOOR config because of properties of loaded map\n";
            indoor_ = true;
            return true;
        }
        else {
            std::cout << "Use OUTDOOR config because of properties of loaded map\n";
            indoor_ = false;
            return false;
        }
    }
}

//########## COMPUTE PSNR: PEAK SIGNAL-TO-NOISE-RATIO ####################
double AutomatedSMMapping::getPSNR(const cv::Mat& I1, const cv::Mat& I2)
{
    cv::Mat s1;
    cv::absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    cv::Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
     return 0;
    else
    {
        double  mse =sse /(double)(I1.channels() * I1.total());
        double psnr = 10.0*log10((255*255)/mse);
        return psnr;
    }
}

//############# COMPUTE SSIM: STRUCTURAL SIMILARITY ############
cv::Scalar AutomatedSMMapping::getMSSIM( const cv::Mat& i1, const cv::Mat& i2){
    //based on https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#image-similarity-psnr-and-ssim
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d     = CV_32F;

    cv::Mat I1, I2;
    i1.convertTo(I1, d);           // cannot calculate on one byte large values
    i2.convertTo(I2, d);

    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2

    /***********************PRELIMINARY COMPUTING ******************************/

    cv::Mat mu1, mu2;   //
    cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);

    cv::Mat sigma1_2, sigma2_2, sigma12;

    cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    ///////////////////////////////// FORMULA ////////////////////////////////
    cv::Mat t1, t2, t3;

    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    cv::Mat ssim_map;
    cv::divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

    cv::Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
    return mssim;
}

// ############# GET INIT POSE #############################
geometry_msgs::PoseWithCovarianceStamped AutomatedSMMapping::get_init_pose(std::string url, int init_pose_linkpoint_id, int map_id){
    //get linkpoint
    LinkPoint link_point = linkpoint_db_.get_linkpoint(url, init_pose_linkpoint_id);

    int map_id_0 = link_point.get_map_id(0);
    int map_id_1 = link_point.get_map_id(1);

    int index;
    if(map_id == map_id_0){
        index = 0;
    }
    else if(map_id == map_id_1){
        index = 1;
    }

    //check if direction (heading) of pose is correct
    tf::Quaternion q_orig(0, 0, link_point.get_q_z_from_theta(index), link_point.get_q_w_from_theta(index));

    if(map_id != link_point.get_direction()){
//        tf::Quaternion q_rot;
//        double r=0, p=0, y=3.14159;  // Rotate the previous pose by 180* about Z
//        q_rot.setRPY(r, p, y);

//        q_orig = q_rot*q_orig;  // Calculate the new orientation
//        q_orig.normalize();

        //get transform to link point saved in database
        tf::Transform lp_transform;
        lp_transform.setOrigin(tf::Vector3(link_point.get_pos_x(index), link_point.get_pos_y(index), 0));
        lp_transform.setRotation(tf::Quaternion(0, 0, link_point.get_q_z_from_theta(index), link_point.get_q_w_from_theta(index)));

        //transform to rotate 180 deg
        tf::Transform rot_180;
        tf::Quaternion q_rot;
        double r=0, p=0, y=3.14159;  // Rotate the previous pose by 180* about Z
        q_rot.setRPY(r, p, y);
        rot_180.setOrigin(tf::Vector3(0, 0, 0));
        rot_180.setRotation(q_rot);

        //transform to move the robot 3,5 meters in positive x direction
        tf::Transform move_x;
        move_x.setOrigin(tf::Vector3(3.5, 0, 0));
        move_x.setRotation(tf::Quaternion(0, 0, 0, 1));

        //compose all transforms
        tf::Transform init_pose_transform;
        init_pose_transform = lp_transform * rot_180 * move_x;

        //convert transform to pose msg
        geometry_msgs::PoseWithCovarianceStamped init_pose;

        init_pose.pose.pose.position.x = init_pose_transform.getOrigin().x();
        init_pose.pose.pose.position.y = init_pose_transform.getOrigin().y();
        init_pose.pose.pose.position.z = 0;

        init_pose.pose.pose.orientation.x = init_pose_transform.getRotation().x();
        init_pose.pose.pose.orientation.y = init_pose_transform.getRotation().y();
        init_pose.pose.pose.orientation.z = init_pose_transform.getRotation().z();
        init_pose.pose.pose.orientation.w = init_pose_transform.getRotation().w();

        init_pose.pose.covariance.fill(0.0);
        init_pose.header.stamp = ros::Time::now();

        return init_pose;
    }
    else{
        geometry_msgs::PoseWithCovarianceStamped init_pose;

        init_pose.pose.pose.position.x = link_point.get_pos_x(index);
        init_pose.pose.pose.position.y = link_point.get_pos_y(index);
        init_pose.pose.pose.position.z = 0;

        init_pose.pose.pose.orientation.x = q_orig.x();
        init_pose.pose.pose.orientation.y = q_orig.y();
        init_pose.pose.pose.orientation.z = q_orig.z();
        init_pose.pose.pose.orientation.w = q_orig.w();

        init_pose.pose.covariance.fill(0.0);
        init_pose.header.stamp = ros::Time::now();

        return init_pose;
    }
}


//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "automated_sm_mapping");

    ros::NodeHandle node_handle;
    AutomatedSMMapping automated_sm_mapping(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
