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
 * @file   environment_detector.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   25.07.2019
 *
 * @brief  Environment detector for detecting doors and environment changes
 * and saving linkpoints and linkpoint candidates
 */

#include "environment_detector/environment_detector.h"
#include "cv_bridge/cv_bridge.h"

//########## CONSTRUCTOR ###############################################################################################
EnvironmentDetector::EnvironmentDetector(ros::NodeHandle &node_handle):
    node_(&node_handle)
{

    // === PARAMETERS ===
    node_->param("map_manager/feature_matching_thr", number_matches_thr_, 30);
    node_->param("map_manager/icp_fitness_score_thr", icp_fitness_score_thr_, 0.5);
    node_->param("map_manager/deriv_door_thr", deriv_door_thr_, 1.0);
    double time_limit_door_detection;
    node_->param("map_manager/time_limit_door_detection", time_limit_door_detection, 15.0);
    time_threshold_full_detection_ = ros::Duration(time_limit_door_detection);
    node_->param("map_manager/deriv_range_thr", deriv_range_thr_, 2.0);
    node_->param("map_manager/initial_variance_x", var_x_, 0.003);
    node_->param("map_manager/initial_variance_y", var_y_, 0.003);
    node_->param("map_manager/initial_variance_theta", var_theta_, 0.003);
    node_->param("map_manager/low_pass_filter_size_env", low_pass_filter_size_env_, 50);
    node_->param("map_manager/num_last_imgs", num_last_imgs_, 5);
    node_->param("map_manager/NNDR_akaze", NNDR_akaze_, 0.7f);
    node_->param("map_manager/NNDR_sift", NNDR_sift_, 0.6f);


    // === SUBSCRIBER ===
    velodyne_scan_subscriber_ = node_->subscribe("/velodyne_laserScan", 1, &EnvironmentDetector::VelodyneScanSubscriberCallback, this);
    start_env_detection_subscriber_ = node_->subscribe("/environment_detector/start", 1, &EnvironmentDetector::StartEnvDetectionSubscriberCallback, this);
    velodyne_pointcloud_subscriber_ = node_->subscribe("/velodyne_points", 1, &EnvironmentDetector::VelodynePointcloudSubscriberCallback, this);
    rear_cam_image_subscriber_ = node_->subscribe("/cam_back/color/image_raw", 1, &EnvironmentDetector::RearCamImageSubscriberCallback, this);
    front_cam_image_subscriber_ = node_->subscribe("/cam_front/color/image_raw", 1, &EnvironmentDetector::FrontCamImageSubscriberCallback, this);
    global_robot_pose_subscriber_ = node_->subscribe("/map_manager/global_pose", 1, &EnvironmentDetector::GlobalPoseSubscriberCallback, this);

    // === PUBLISHER ===
    end_env_detector_pub_ = node_->advertise<map_manager::end_env_detection>("/environment_detector/end",20);
    debug_image_pub_ = node_->advertise<sensor_msgs::Image>("/environment_detector/debug_image",20);
    debug_pointcloud_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/environment_detector/debug_pointcloud",20);
    debug_feature_matching_img_pub_ = node_->advertise<sensor_msgs::Image>("/environment_detector/debug_img_feature_matching",20);
    debug_icp_registered_pointcloud_pub_= node_->advertise<sensor_msgs::PointCloud2>("/environment_detector/debug_icp_registered_pointcloud",20);
    sound_saying_pub_ = node_->advertise<sound_play::SoundRequest>("/robotsound",20);
    debug_scan_pointcloud_pub_ = node_->advertise<sensor_msgs::PointCloud>("/environment_detector/scan_point_cloud_filtered",20);

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.1), &EnvironmentDetector::timerCallback, this);
    timer_scan_bounding_box_ = node_->createTimer(ros::Duration(3.0), &EnvironmentDetector::timerCallbackBoundingBox, this);

    //initialize parameters
    door_back_detected_ = false;
    door_front_detected_ = false;
    searching_ = false;
    search_started_ = false;
    environment_change_detected_ = false;

    cnt_door_front_detected_ = 0;
    cnt_door_back_detected_ = 0;

    first_detection_time_ = ros::Time::now();

    record_images_ = false;
    last_n_images_.clear();

    record_rear_images_ = false;
    last_n_rear_images_.clear();

    mean_scan_left_t_1_ = 0;
    mean_scan_left_t_2_ = 0;
    mean_scan_right_t_1_ = 0;
    mean_scan_right_t_2_ = 0;

    x_max_t_1_ = 0;
    x_min_t_1_ = 0;
    y_max_t_1_ = 0;
    y_min_t_1_ = 0;
}

//########## CALLBACK: SUBSCRIBER START ENVIRONMENT DETECTION ############################################################################
void EnvironmentDetector::StartEnvDetectionSubscriberCallback(const map_manager::start_env_detection &msg)
{
    if(msg.start){
        searching_ = true;
        cur_map_name_ = msg.map_name;
        database_path_ = msg.database_path;
        lp_db_url_ = msg.linkpoint_db_path;
        cur_map_id_ = msg.map_id;
        pointcloud_dir_ = msg.pointcloud_dir;
        loop_closure_search_radius_ = msg.loop_closure_search_radius;
        no_graph_connection_ = msg.no_graph_connection;
        indoor_ = msg.indoor;
        ROS_WARN("Start environment detection");

        pose_linkpoint_candidate_.setOrigin(tf::Vector3(100000, 100000, 0)); //make sure that previous saved linkpoint cadidates positions are deleted

        //reset old timesteps from environment detection
        x_max_t_1_ = 0;
        x_min_t_1_ = 0;
        y_max_t_1_ = 0;
        y_min_t_1_ = 0;
    }
}

//########## CALLBACK: SUBSCRIBER VELODYNE POINTCLOUD ############################################################################
void EnvironmentDetector::VelodynePointcloudSubscriberCallback(const sensor_msgs::PointCloud2 &msg)
{
    velodyne_pointcloud_msg_ = msg;
}

//########## CALLBACK: SUBSCRIBER REAR CAM IMAGE ############################################################################
void EnvironmentDetector::RearCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // convert image to cv::Mat
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, "bgr8");
      cam_rear_image_msg_ = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("Failed to convert image");
    }

    //record 10 images for robust feature detection
    if(record_rear_images_ && last_n_rear_images_.size() <= num_last_imgs_)
    {
        last_n_rear_images_.push_back(cam_rear_image_msg_);
        ROS_INFO("Push back message in last_n_rear_images!");
    }
    else if(record_rear_images_ && last_n_rear_images_.size() > num_last_imgs_){
        record_rear_images_ = false;
        ROS_INFO("Got %i rear images!", num_last_imgs_);
    }
}

//########## CALLBACK: SUBSCRIBER FRONT CAM IMAGE ############################################################################
void EnvironmentDetector::FrontCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // convert image to cv::Mat
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, "bgr8");
      cam_back_image_msg_ = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("Failed to convert image");
    }

    //record n images for robust feature detection
    if(record_images_ && last_n_images_.size() <= num_last_imgs_)
    {
        last_n_images_.push_back(cam_back_image_msg_);
        ROS_INFO("Push back message in last_n_images!");
    }
    else if(record_images_ && last_n_images_.size() > num_last_imgs_){
        record_images_ = false;
        ROS_INFO("Got %i images!", num_last_imgs_);
    }
}


//########## CALLBACK: SUBSCRIBER VELODYNE SCAN ############################################################################
void EnvironmentDetector::VelodyneScanSubscriberCallback(const sensor_msgs::LaserScan &msg)
{

    int scan_size = msg.ranges.size();

    //Attention: laser scan message is counter-clockwise
    float scan_front_min_angle = -0.872665; //50 deg
    float scan_front_max_angle = 0.872665; // 50 deg

    float scan_back_min_angle = 2.26893; //130 deg
    float scan_back_max_angle = -2.26893; //-130 deg

    float scan_right_min_angle = -2.00712; //-110 deg //50 degree angle
    float scan_right_max_angle = -1.13446; // -65 deg

    float scan_left_min_angle = 1.13446; //65 deg //50 degree angle
    float scan_left_max_angle = 2.00712; // 115 deg

    float current_angle;

    //create new LaserScan message scan_front
    sensor_msgs::LaserScan scan_front;
    scan_front.header = msg.header;
    scan_front.angle_max = scan_front_max_angle;
    scan_front.angle_min = scan_front_min_angle;
    scan_front.angle_increment = msg.angle_increment;

    //create new LaserScan message scan_back
    sensor_msgs::LaserScan scan_back;
    scan_back.header = msg.header;
    scan_back.angle_max = scan_back_max_angle;
    scan_back.angle_min = scan_back_min_angle;
    scan_back.angle_increment = msg.angle_increment;

    //create new LaserScan message scan_left for scan_change detection
    sensor_msgs::LaserScan scan_left;
    scan_left.header = msg.header;
    scan_left.angle_max = scan_left_max_angle;
    scan_left.angle_min = scan_left_min_angle;
    scan_left.angle_increment = msg.angle_increment;

    //create new LaserScan message scan_right for scan_change detection
    sensor_msgs::LaserScan scan_right;
    scan_right.header = msg.header;
    scan_right.angle_max = scan_right_max_angle;
    scan_right.angle_min = scan_right_min_angle;
    scan_right.angle_increment = msg.angle_increment;

    //create new LaserScan message scan_full for scan_change detection
    sensor_msgs::LaserScan scan_full;
    scan_full.header = msg.header;
    scan_full.angle_max = msg.angle_max;
    scan_full.angle_min = msg.angle_min;
    scan_full.angle_increment = msg.angle_increment;

    for (int i = 0; i < scan_size; i++) {
        current_angle = msg.angle_min + i * msg.angle_increment;

        //detected point in range for scan front
        if(current_angle >= scan_front_min_angle && current_angle <= scan_front_max_angle){
            scan_front.ranges.push_back(msg.ranges[i]);
        }

        //detected point in range for scan back
        if(current_angle >= scan_back_min_angle || current_angle <= scan_back_max_angle){
            scan_back.ranges.push_back(msg.ranges[i]);
        }

        //detected point in range for scan left
        if(current_angle >= scan_left_min_angle && current_angle <= scan_left_max_angle){
            scan_left.ranges.push_back(msg.ranges[i]);
        }

        //detected point in range for scan right
        if(current_angle >= scan_right_min_angle && current_angle <= scan_right_max_angle){
            scan_right.ranges.push_back(msg.ranges[i]);
        }

        //bush back everything in velodyne_scan_full
        scan_full.ranges.push_back(msg.ranges[i]);
    }

    //store both created LaserScans in member variable
    velodyne_scan_front_ = scan_front;
    velodyne_scan_back_ = scan_back;
    velodyne_scan_left_ = scan_left;
    velodyne_scan_right_ = scan_right;
    velodyne_scan_full_ = scan_full;
}

//########## CALLBACK: SUBSCRIBER GLOBAL ROBOT POSE ############################################################################
void EnvironmentDetector::GlobalPoseSubscriberCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
    double variance_x = msg.pose.covariance.at(0);
    double variance_y = msg.pose.covariance.at(7);

    //old computation of search radius
    //loop_closure_search_radius_with_variance_ = sqrt(pow(loop_closure_search_radius_+2.58 * sqrt(fabs(variance_x)), 2) + pow(2.58 * sqrt(fabs(variance_y)), 2)); //calcualte new search radius with variance

    //new computation of search radius
    Eigen::Matrix2d cov_x_y;
    cov_x_y(0,0) = msg.pose.covariance.at(0);
    cov_x_y(0,1) = msg.pose.covariance.at(1);
    cov_x_y(1,0) = msg.pose.covariance.at(6);
    cov_x_y(1,1) = msg.pose.covariance.at(7);

    Eigen::EigenSolver<Eigen::Matrix2d> es(cov_x_y, false);
    complex<double> e1 = es.eigenvalues().col(0)[0];
    complex<double> e2 = es.eigenvalues().col(0)[1];

    //get the higer eigenvector, should be only real for covariance
    double lambda_max;
    if(e1.real() > e2.real()){
        lambda_max = e1.real();
    }
    else{
        lambda_max = e2.real();
    }

    //add the largest major axis of the 95% confidence ellipse with the largest eigenvalue
    loop_closure_search_radius_with_variance_ = loop_closure_search_radius_ + 2*sqrt(fabs(5.991*lambda_max));


    global_pose_.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
    global_pose_.setRotation(tf::Quaternion(0, 0, 0, 1)); //rotation is not relevant!
}


//########## CALLBACK: TIMER ###########################################################################################
void EnvironmentDetector::timerCallback(const ros::TimerEvent &evt)
{
    //###### Search for door front #####################
    if(searching_ && !door_front_detected_ && cnt_door_front_detected_ < 3)
    {
        //ROS_INFO("In search front door now");
        if(EnvironmentDetector::detect_door_from_front(velodyne_scan_front_)){
            if(cnt_door_front_detected_ == 0){
                first_detection_time_ = ros::Time::now();
                search_started_ = true;
            }
            cnt_door_front_detected_++;
            ROS_INFO("cnt_door_front_detected = %i", cnt_door_front_detected_);
            ros::Duration(0.5).sleep(); //sleep a while to have some time between the three detections

            //check if detection was within the expected time
            ros::Duration delta_t = ros::Time::now() - first_detection_time_;
            if(delta_t > time_threshold_){
                cnt_door_front_detected_ = 0; //detection for door front over time threshold -> reset counter
                ROS_WARN("Reset counter for door front");
            }

            // if all requrements are fulfilled, set door_front detected to true
            if(cnt_door_front_detected_ == 3){
                door_front_detected_ = true;
                cnt_door_front_detected_ = 0;
                EnvironmentDetector::say_text("Door front detected!");

                //store here in member variable the point cloud, rear picture and position on map and current map_name
                velodyne_pointcloud_linkpoint_candidate_ = velodyne_pointcloud_msg_;
                cam_rear_linkpoint_candidate_ = cam_rear_image_msg_;
                //check if the robot moved since the last linkpoint candidate has been detected
                tf::Transform door_front_pose =  EnvironmentDetector::get_localization_pose();
                double delta_x = door_front_pose.getOrigin().x() - pose_linkpoint_candidate_.getOrigin().x(); //comparison between last saved pose and current pose
                double delta_y = door_front_pose.getOrigin().y() - pose_linkpoint_candidate_.getOrigin().y();
                if(sqrt(pow(delta_x, 2) + pow(delta_y, 2)) > 2){ //check if euclidian distance between the two poses is larger than 2m
                    pose_linkpoint_candidate_ = door_front_pose;
                    world_pose_linkpoint_candidate_ = EnvironmentDetector::get_world_pose();
                    ROS_WARN("New linkpoint candidate detected");
                }
                else{
                    ROS_WARN("Distance between detected doors is under 2 meters. Rejecting door_front_detected!!");
                    door_front_detected_=false;
                }
            }
        }
    }

    //###### Search for door back #####################
    else if (searching_ && door_front_detected_ && !door_back_detected_ && cnt_door_back_detected_ < 3) {
        if(EnvironmentDetector::detect_door_from_back(velodyne_scan_back_)){
            cnt_door_back_detected_++;
            ROS_INFO("cnt_door_back_detected = %i", cnt_door_back_detected_);
            ros::Duration(0.15).sleep(); //sleep a while to have some time between the three detections
            if(cnt_door_back_detected_ == 3){
                //check if distance between positions from door_front and door_back are in an right distance and the orientation is the same!
                tf::Transform door_back_pose =  EnvironmentDetector::get_localization_pose();
                double delta_x = door_back_pose.getOrigin().x() - pose_linkpoint_candidate_.getOrigin().x(); //comparison between last saved pose and current pose
                double delta_y = door_back_pose.getOrigin().y() - pose_linkpoint_candidate_.getOrigin().y();
                double delta_theta = fabs(EnvironmentDetector::theta_from_q(door_back_pose.getRotation()) - EnvironmentDetector::theta_from_q(pose_linkpoint_candidate_.getRotation()));
                double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
                if( distance > 1.5
                        && distance < 5
                        && (delta_theta < (3.14159*30/180) || delta_theta > (3.14159*330/180))){ //check if distance and angle is correct
                    door_back_detected_ = true;
                    cam_back_linkpoint_ = cam_back_image_msg_; //store cam front image for every linkpoint
                    cnt_door_back_detected_ = 0;
                }
                else {
                    ROS_WARN("Door back was rejected tue to unrealistic conditions!");
                    EnvironmentDetector::say_text("Door back was rejected due to unrealistic conditions!");
                    std::cout << "\n delta_x = " << delta_x
                              << "\n delta_y = " << delta_y
                              << "\n distance = " << distance
                              << "\n delta_theta = " << delta_theta;
                    door_back_detected_ = false;
                    cnt_door_back_detected_ = 0;
                }
            }
        }
    }

    //######### when overtime ##############
    if (searching_
        && search_started_
        && (ros::Time::now() - first_detection_time_) > time_threshold_full_detection_){ //it takes too long to detect door front and door back

        if(door_front_detected_
                && !door_back_detected_
                && !environment_change_detected_){

            lp_candidate_id_ = linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTCANDIDATES"); //subtract 1 to start count with 0

            LinkPoint lp_candidate;
            lp_candidate.set_id(lp_candidate_id_); //TODO: count number of linkpoint_candidates in db
            lp_candidate.set_pos_x(pose_linkpoint_candidate_.getOrigin().x(), 0);
            lp_candidate.set_pos_y(pose_linkpoint_candidate_.getOrigin().y(), 0);
            lp_candidate.set_theta_from_z_w(pose_linkpoint_candidate_.getRotation().z(), pose_linkpoint_candidate_.getRotation().w(), 0);
            lp_candidate.set_pos_x(world_pose_linkpoint_candidate_.getOrigin().x(), 2);
            lp_candidate.set_pos_y(world_pose_linkpoint_candidate_.getOrigin().y(), 2);
            lp_candidate.set_theta_from_z_w(world_pose_linkpoint_candidate_.getRotation().z(), world_pose_linkpoint_candidate_.getRotation().w(), 2);
            lp_candidate.set_map_id(cur_map_id_, 0);
            lp_candidate.set_difficulty(1); //TODO: find a way to define difficulty
            if(indoor_){
                lp_candidate.set_environment(1);} //1=indoor
            else {
                lp_candidate.set_environment(0);} //0=outdoor

            lp_candidate.print_linkpoint_properties();

            cur_pointcloud_path_ = pointcloud_dir_ + "/" + std::to_string(lp_candidate_id_) + "_pointcloud.pcd";
            linkpoint_db_.insert_linkpoint_candidate(lp_db_url_, lp_candidate, cam_rear_linkpoint_candidate_, cur_pointcloud_path_);
            EnvironmentDetector::say_text("New linkpoint candidate saved.");


            //convert cloud to pcl and save it under cur_pointcloud_path
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = EnvironmentDetector::pc2_to_pcl(velodyne_pointcloud_linkpoint_candidate_);
            pcl::io::savePCDFileASCII (cur_pointcloud_path_, *cloud);
            std::cerr << "Saved " << cloud->points.size () << " data points to " << cur_pointcloud_path_ << "." << std::endl;

            //publish saved image and pointcloud for debugging
            cv::Mat debug_image = linkpoint_db_.get_linkpoint_candidate_image(lp_db_url_, 0);
            EnvironmentDetector::publish_debug_image(debug_image);

            //retrieve pointcloud from file and publish it
            EnvironmentDetector::publish_debug_pointcloud(cur_pointcloud_path_);
        }

            //reset all parameters for new search
            door_front_detected_ = false;
            door_back_detected_ = false;
            environment_change_detected_ = false;
            search_started_ = false;
	    cnt_door_front_detected_ = 0;
	    cnt_door_back_detected_ = 0;
            ROS_WARN("Reset door detection to beginning due to overtime");
            EnvironmentDetector::say_text("Reset door detection to beginning due to overtime.");
    }

    //########### when door detected ############
    if (searching_
         && door_front_detected_
         && door_back_detected_
         && (ros::Time::now() - first_detection_time_) < time_threshold_full_detection_){
        searching_ = false;
        search_started_ = false;
        ROS_WARN("##############Door sucessfully detected!!!!!###################");
        EnvironmentDetector::say_text("Door detected!");

        //check if map is already known by feature matching and point cloud registration and global coordinate frame
        last_n_images_.clear();
        ROS_INFO("last_n_images.size() before filling: %i", last_n_images_.size());
        record_images_ = true;
        //wait unti the last n images are collected
        ROS_INFO("Collect %i Images now for loop closing!", num_last_imgs_);
        while(last_n_images_.size() <= num_last_imgs_){
            ros::spinOnce();
        }
        ROS_INFO("last_n_images.size() after filling: %i", last_n_images_.size());

        //collect rear images
        last_n_rear_images_.clear();
        ROS_INFO("last_n_rear_images.size() before filling: %i", last_n_rear_images_.size());
        record_rear_images_ = true;
        //wait unti the last n images are collected
        ROS_INFO("Collect n rear Images now for loop closing!");
        while(last_n_rear_images_.size() <= num_last_imgs_){
            ros::spinOnce();
        }
        ROS_INFO("last_n_rear_images.size() after filling: %i", last_n_rear_images_.size());

        //calculate medians of matches for all candidates
        std::vector<double> matches_median; //medians of matches from all candidates
        std::vector<int> lp_candidates_ids = linkpoint_db_.get_lp_candidate_ids(lp_db_url_);
        ROS_INFO("Number of possible linkpoint candidates: %i", lp_candidates_ids.size());

        std::cout << "loop_closure_search_radius_with_variance_:\n" << loop_closure_search_radius_with_variance_ << "\n";

        for(int index=0; index<lp_candidates_ids.size(); index++){
            int id = lp_candidates_ids.at(index);
            //candidate filtering by global pose
            if(EnvironmentDetector::lp_in_search_radius(lp_db_url_, id, loop_closure_search_radius_with_variance_, true) || no_graph_connection_){
                if(no_graph_connection_){
                      std::cout << "Linkpoint is still considered because no_graph_connection_ == true";
                }
                cv::Mat lp_candidate_input_img = linkpoint_db_.get_linkpoint_candidate_image(lp_db_url_, id);
                std::vector<int> matches_for_candidate; //to store the 10 results for one candidate
                for (int k=0; k<=num_last_imgs_; k++) {
                    matches_for_candidate.push_back(EnvironmentDetector::get_num_matches_akaze_sift(id, lp_candidate_input_img, last_n_images_[k]));
                }
                matches_median.push_back(EnvironmentDetector::get_median(matches_for_candidate));
            }
        }

        //detect if one median is over the detection threshold
        int num_matches_thr = number_matches_thr_;
        int loop_closure_lp_id;
        loop_closure_detected_ = false;
        for (int k = 0; k<matches_median.size(); k++) {
            if(matches_median[k] > num_matches_thr){
                loop_closure_lp_id = k;
                num_matches_thr = matches_median[k]; //in case there is even a larger one
                loop_closure_detected_ = true;
                ROS_INFO("Loop closure detected by visual feature matching with link point cnadidate id = %i!", loop_closure_lp_id);
                ROS_INFO("Entry %i of matches_median with value %d was over threshold", k, matches_median[k]);
            }
        }

        tf::Transform transform_icp;
        if(loop_closure_detected_){
            //check if icp registration between pointclouds work
            std::string pointcloud_path = pointcloud_dir_ + "/" + std::to_string(loop_closure_lp_id) + "_pointcloud.pcd";
            sensor_msgs::PointCloud2 lp_candidate_pointcloud = EnvironmentDetector::publish_debug_pointcloud(pointcloud_path);
            transform_icp = EnvironmentDetector::get_transform_from_icp(EnvironmentDetector::pc2_to_pcl(velodyne_pointcloud_msg_),
                                                                        EnvironmentDetector::pc2_to_pcl(lp_candidate_pointcloud));
        }

        int old_linkpoint_id = -1;
        if(loop_closure_detected_){
            ROS_INFO("In case: Loop closure detected!");
            EnvironmentDetector::say_text("Loop closure with link point candidate detected.");

            //if already known map:
            //save new linkpoint with position on current map and old linkpoint candidates position on old map
            tf::Transform cur_pose = EnvironmentDetector::get_localization_pose();
            tf::Transform cur_world_pose = EnvironmentDetector::get_world_pose();
            LinkPoint lp_loop_closure;
            lp_loop_closure.set_id(linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS")); //set id after the next number of linkpoints
            lp_loop_closure.set_tag("-");
            lp_loop_closure.set_pos_x(cur_pose.getOrigin().x(), 0);
            lp_loop_closure.set_pos_y(cur_pose.getOrigin().y(), 0);
            lp_loop_closure.set_theta_from_z_w(cur_pose.getRotation().z(), cur_pose.getRotation().w(), 0);
            lp_loop_closure.set_pos_x(cur_world_pose.getOrigin().x(), 2);
            lp_loop_closure.set_pos_y(cur_world_pose.getOrigin().y(), 2);
            lp_loop_closure.set_theta_from_z_w(cur_world_pose.getRotation().z(), cur_world_pose.getRotation().w(), 2);
            lp_loop_closure.set_map_id(cur_map_id_, 0);
            lp_loop_closure.set_difficulty(1);
            if(indoor_){
                lp_loop_closure.set_environment(1);} //1=indoor
            else {
                lp_loop_closure.set_environment(0);} //0=outdoor


            //retrieve linkpoint candidate from database, compute current position of robot in coordinate frame of linkpoint candidates map
            LinkPoint lp_candidate = linkpoint_db_.get_linkpoint_candidate(lp_db_url_, loop_closure_lp_id);
            //compute current position of robot in coordinate frame of linkpoint candidates map
            tf::Transform loop_closure_pose = EnvironmentDetector::get_loop_closure_pose(lp_candidate, transform_icp);
            lp_loop_closure.set_pos_x(loop_closure_pose.getOrigin().x(), 1);
            lp_loop_closure.set_pos_y(loop_closure_pose.getOrigin().y(), 1);
            lp_loop_closure.set_theta_from_z_w(loop_closure_pose.getRotation().z(), loop_closure_pose.getRotation().w(), 1);
            lp_loop_closure.set_map_id(lp_candidate.get_map_id(0), 1);
            lp_loop_closure.set_direction(lp_candidate.get_map_id(0)); //Direction is from current map to map with lp candidate

            //insert linkpoint in linkpoints.db
            linkpoint_db_.insert_linkpoint_with_images(lp_db_url_, lp_loop_closure, cam_back_linkpoint_, cam_rear_linkpoint_candidate_);
            //delete old entry from the linkpoint candidate
            linkpoint_db_.delete_linkpoint_candidate(lp_db_url_, loop_closure_lp_id);

            //set loop closure map_id so that this int is publshed for automated_sm_mapping node
            loop_closure_map_id_ = lp_candidate.get_map_id(0);

            //fill old_linkpoint_id with new created linkpoint, so that init pose can be set
            old_linkpoint_id = lp_loop_closure.get_id();

            //execute graph optimization with g2o
            ROS_INFO("Optimize Graph now!!!");
	    std::cout << "Optimize graph now!";
            GraphOptimizer optimizer;
            optimizer.g2o_optimize_graph(lp_db_url_, lp_candidate, lp_loop_closure, transform_icp, var_x_, var_y_, var_theta_);
            std::cout << "Optimization done!";
            ROS_INFO("Optimization done!");
        }

        if(!loop_closure_detected_){
            old_linkpoint_id = EnvironmentDetector::check_for_new_map();
            if(old_linkpoint_id == -1){
                ROS_INFO("In case: No Loop closure detected, start new map!");
                EnvironmentDetector::say_text("No Loop closure detected, start new map!");
                //if new environment:
                //save linkpoint with old data from linkpoint candidate and the origin of the new map (0,0)
                tf::Transform cur_pose = EnvironmentDetector::get_localization_pose();
                tf::Transform cur_world_pose = EnvironmentDetector::get_world_pose();
                LinkPoint lp_new_map;
                lp_new_map.set_id(linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS")); //set id after the next number of linkpoints
                lp_new_map.set_tag("-");
                lp_new_map.set_pos_x(cur_pose.getOrigin().x(), 0);
                lp_new_map.set_pos_y(cur_pose.getOrigin().y(), 0);
                lp_new_map.set_theta_from_z_w(cur_pose.getRotation().z(), cur_pose.getRotation().w(), 0);
                lp_new_map.set_pos_x(cur_world_pose.getOrigin().x(), 2);
                lp_new_map.set_pos_y(cur_world_pose.getOrigin().y(), 2);
                lp_new_map.set_theta_from_z_w(cur_world_pose.getRotation().z(), cur_world_pose.getRotation().w(), 2);
                lp_new_map.set_map_id(cur_map_id_, 0);
                lp_new_map.set_difficulty(1);
                if(indoor_){
                    lp_new_map.set_environment(1);} //1=indoor
                else {
                    lp_new_map.set_environment(0);} //0=outdoor

                //the linkpoint on the new map is also the origin
                lp_new_map.set_pos_x(0, 1);
                lp_new_map.set_pos_y(0, 1);
                lp_new_map.set_theta_from_z_w(0, 1, 1);
                lp_new_map.set_map_id(linkpoint_db_.get_number_of_rows(lp_db_url_, "MAPS"), 1); //set the next id with respect to the existing maps
                lp_new_map.set_direction(linkpoint_db_.get_number_of_rows(lp_db_url_, "MAPS")); //set direction from current map to next map

                lp_new_map.print_linkpoint_properties();

                ROS_INFO("Inserting linkpoint now!");
                linkpoint_db_.insert_linkpoint_with_images(lp_db_url_, lp_new_map, cam_back_linkpoint_, cam_rear_linkpoint_candidate_);
                EnvironmentDetector::say_text("Saved new linkpoint.");

                loop_closure_map_id_ = -1; //new map started
            }
            else { //linkpoint already exists

                //find out the id of the already known map which the robot just entered
                LinkPoint existing_lp = linkpoint_db_.get_linkpoint(lp_db_url_, old_linkpoint_id);
                if(existing_lp.get_map_id(0) == cur_map_id_){
                    loop_closure_map_id_ =existing_lp.get_map_id(1); //take the other map id, since this is the map the robot just entered
                }
                else if (existing_lp.get_map_id(1) == cur_map_id_) {
                    loop_closure_map_id_ = existing_lp.get_map_id(0); //take the other map id, since this is the map the robot just entered
                }

                ROS_WARN("This linkpoint already exists!!!");
                EnvironmentDetector::say_text("This linkpoint already exists!");
            }
        }

        //send message to end environment detection
        ROS_INFO("Publish end of environment detection");
        map_manager::end_env_detection end_msg;
        end_msg.map_id = loop_closure_map_id_;
        end_msg.linkpoint_id = old_linkpoint_id;
        end_env_detector_pub_.publish(end_msg);

        //reset all parameter
        door_front_detected_ = false;
        door_back_detected_ = false;
        environment_change_detected_ = false;
        search_started_ = false;
        cnt_door_front_detected_ = 0;
        cnt_door_back_detected_ = 0;
    }
}

//############ TIMER CALLBACK BOUNDING BOX ##########################################
void EnvironmentDetector::timerCallbackBoundingBox(const ros::TimerEvent &evt)
{

    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "map";
    std::string sourceFrameID = "base_link";
    double waitTime = 1;

    bool bTransformAvailable = false;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    if(velodyne_scan_full_.ranges.size() > 0 && bTransformAvailable && searching_){

        std::cout << "in timer callback\n";
        ScanUtils scan_utils;
    // 1) low pass filter the velodyne_laser_scan message
        sensor_msgs::LaserScan scan_filtered;
        scan_filtered = scan_utils.lowPassAveraging(velodyne_scan_full_, low_pass_filter_size_env_);

   // 2) transform the scan points in map coordinate frame
        sensor_msgs::PointCloud scan_cloud = scan_utils.transform_scan_to_map_frame(scan_filtered);
        debug_scan_pointcloud_pub_.publish(scan_cloud);
   // 3) compute the bounding box parameter x_max/x_min y_max/y_min for map coordinate frame
        float x_max = scan_utils.get_x_max(scan_cloud);
        float x_min = scan_utils.get_x_min(scan_cloud);
        float y_max = scan_utils.get_y_max(scan_cloud);
        float y_min = scan_utils.get_y_min(scan_cloud);

   // 4) compute the derivation of m_max/x_min y_max/y_min
        float deriv_x_max = 0;
        float deriv_x_min = 0;
        float deriv_y_max = 0;
        float deriv_y_min = 0;
        if(x_max_t_1_ != 0){ //check for fist time step
            deriv_x_max = fabs((x_max - x_max_t_1_)/2);
            deriv_x_min = fabs((x_min - x_min_t_1_)/2);
            deriv_y_max = fabs((y_max - y_max_t_1_)/2);
            deriv_y_min = fabs((y_min - y_min_t_1_)/2);
        }
        //store for next time step
        x_max_t_1_ = x_max;
        x_min_t_1_ = x_min;
        y_max_t_1_ = y_max;
        y_min_t_1_ = y_min;

   // 5) detect environment change based on derivatives
        float deriv_threshold = deriv_range_thr_;
        float distance_max = 10.0;  //check for maximum distance while being outdoor
        int cnt = 0;

        //debug
        std::cout << "X_MAX deriv: " << deriv_x_max << "\n";
        std::cout << "X_MIN deriv: " << deriv_x_min << "\n";
        std::cout << "Y_MAX deriv: " << deriv_y_max << "\n";
        std::cout << "Y_MIN deriv: " << deriv_y_min << "\n";

        //if indoor: don't do the distance check
        if(indoor_){
		ROS_INFO("In indoor check");
		if(deriv_x_max > deriv_threshold){
		    ROS_WARN("X_MAX deriv over threshold!");
		    cnt++;
		}
		if(deriv_x_min > deriv_threshold){
		    ROS_WARN("X_MIN deriv over threshold!");
		    cnt++;
		}
		if(deriv_y_max > deriv_threshold){
		    ROS_WARN("Y_MAX deriv over threshold!");
		    cnt++;
		}
		if(deriv_y_min > deriv_threshold){
		    ROS_WARN("Y_MIN deriv over threshold!");
		    cnt++;
		}
        }
        else{
		ROS_INFO("In outdoor check -> checking also distances!");
		if(deriv_x_max > deriv_threshold && (fabs(x_max_t_1_) < distance_max || fabs(x_max) < distance_max)){
		    ROS_WARN("X_MAX deriv over threshold!");
		    cnt++;
		}
		if(deriv_x_min > deriv_threshold && (fabs(x_min_t_1_) < distance_max || fabs(x_min) < distance_max)){
		    ROS_WARN("X_MIN deriv over threshold!");
		    cnt++;
		}
		if(deriv_y_max > deriv_threshold && (fabs(y_max_t_1_) < distance_max || fabs(y_max) < distance_max)){
		    ROS_WARN("Y_MAX deriv over threshold!");
		    cnt++;
		}
		if(deriv_y_min > deriv_threshold && (fabs(y_min_t_1_) < distance_max || fabs(y_min) < distance_max)){
		    ROS_WARN("Y_MIN deriv over threshold!");
		    cnt++;
		}
        }

        if(cnt >= 2){
            ROS_WARN("Environment change detected!!!");
            //set member variable true
            environment_change_detected_ = true;

            //set timer
            env_change_dectect_time_ = ros::Time::now();

            if(!door_front_detected_){
                door_front_detected_ = true;
                //in this case door front was not detected -> set it to true so that door back can be detected
                ROS_WARN("Environment change detected without door front detected -> set door_front_detected_ to true and search for door back!");
            }
            else if(door_front_detected_ && !door_back_detected_){
                door_back_detected_ = true;
                //in this case door front was detected and door back was not detected (yet) -> set door detected to true
                ROS_WARN("Environment change detected without door back detected -> set door_back_detected_ to true and save linkpoint!");
                cam_back_linkpoint_ = cam_back_image_msg_; //store cam front image for every linkpoint
                ros::Duration(3.0).sleep(); //sleep, so that in case of a loop closure the robot can be moved to the position where the ICP registration works well
            }
        }

        //reset environment detection after timeout
        ros::Duration delta_t = ros::Time::now() - env_change_dectect_time_;
	if(environment_change_detected_ && delta_t > time_threshold_full_detection_){
	       environment_change_detected_ = false;
               door_front_detected_ = false;
               door_back_detected_ = false;
               ROS_WARN("Reset door detection due to timeout of environment change detection");
	}
    }
}

//############ CUSTOM METHODS ##########################################################################################
bool EnvironmentDetector::detect_door_from_front(sensor_msgs::LaserScan msg)
{
    //Parameter definition
    float derivation_thresh = deriv_door_thr_;
    float door_frame_left_angle;   //angle position for detected door frame
    float door_frame_right_angle;
    bool frame_detected_left = false;
    bool frame_detected_right = false;

    int scan_size = msg.ranges.size();
    float scan_raw[scan_size];

    //store LaserScan message in in vector
    for(int i = 0; i < scan_size; i++)
    {
        scan_raw[i] = msg.ranges[i];
    }

    // calculate the derivation of the range after the angle
    float scan_deriv;
    for (int j=1; j < scan_size; j++) {
        scan_deriv = (scan_raw[j+10] - scan_raw[j-10])/2; //calculate derivation

        //search for left frame
        if(scan_deriv > derivation_thresh
                && (scan_raw[j-5] < 2 && scan_raw[j-5] > 1)
                && frame_detected_left == false
                && frame_detected_right == false) //check if dervivation is big (positive sign) and range left of door frame is between 1 and 2 meters
        {
            frame_detected_left = true;
            door_frame_left_angle = msg.angle_min + (j-1) * msg.angle_increment; //calculate angle for detected left_door_frame
            ROS_INFO("Frame left detected at angle %f deg with derivation %f", door_frame_left_angle/PI*180, scan_deriv);
        }

        //search for right frame
        if(scan_deriv < (-derivation_thresh)
                && (scan_raw[j-5] < 2 && scan_raw[j-5] > 1)
                && frame_detected_left == true //only accept right frame if left is already detected
                && frame_detected_right == false) //check if dervivation is big (negative sign) and range left of door frame is between 1 and 2 meters
        {
            frame_detected_right = true;
            door_frame_right_angle = msg.angle_min + (j-1) * msg.angle_increment; //calculate angle for detected left_door_frame
            ROS_INFO("Frame right detected at angle %f with derivation %f", door_frame_right_angle/PI*180, scan_deriv);
        }
    }

    //debug
    if(frame_detected_left == true && frame_detected_right == true){
        ROS_INFO("Angle between: %f", (door_frame_right_angle - door_frame_left_angle)/PI*180);
    }

    if (frame_detected_left == true
            && frame_detected_right == true
            && ((door_frame_right_angle - door_frame_left_angle)/PI*180 > 20)
            && ((door_frame_right_angle - door_frame_left_angle)/PI*180 < 60)) //check if frame left/right have a certain angle in between
    {
        ROS_WARN("Door detected from front!");
        return true;
    }
    else{
        return false;
    }
}

bool EnvironmentDetector::detect_door_from_back(sensor_msgs::LaserScan msg)
{
    //Parameter definition
    float derivation_thresh = deriv_door_thr_; //based on distance behind door 5m and to door 1.5m
    float door_frame_left_angle;   //angle position for detected door frame
    float door_frame_right_angle;
    bool frame_detected_left = false;
    bool frame_detected_right = false;

    int scan_size = msg.ranges.size();
    float scan_raw[scan_size];

    //store LaserScan message in in vector
    for(int i = 0; i < scan_size; i++)
    {
        scan_raw[i] = msg.ranges[i];
    }

    // calculate the derivation of the range after the angle
    float scan_deriv;
    for (int j=1; j < scan_size; j++) {
        scan_deriv = (scan_raw[j+10] - scan_raw[j-10])/2; //calculate derivation

        //search for left frame
        if(scan_deriv < (-derivation_thresh)
                && (scan_raw[j-5] < 2 && scan_raw[j-5] > 1)
                && frame_detected_left == false
                && frame_detected_right == false) //check if dervivation is big (positive sign) and range left of door frame is between 1 and 2 meters
        {
            frame_detected_right = true;
            door_frame_right_angle = -PI + (j-1) * msg.angle_increment; //calculate angle for detected left_door_frame
            ROS_INFO("Frame right detected at angle %f deg with derivation %f", door_frame_right_angle/PI*180, scan_deriv);
        }

        //search for right frame
        if(scan_deriv > derivation_thresh
                && (scan_raw[j-5] < 2 && scan_raw[j-5] > 1)
                && frame_detected_right == true //only accept right frame if left is already detected
                && frame_detected_left == false) //check if dervivation is big (negative sign) and range left of door frame is between 1 and 2 meters
        {
            frame_detected_left = true;
            door_frame_left_angle = msg.angle_min + (j-1) * msg.angle_increment - PI - msg.angle_max; //calculate angle for detected left_door_frame
            ROS_INFO("Frame left detected at angle %f with derivation %f", door_frame_left_angle/PI*180, scan_deriv);
        }
    }

    //debug
    if(frame_detected_left == true && frame_detected_right == true){
        ROS_INFO("Angle between: %f", (2* PI  + door_frame_right_angle - door_frame_left_angle)/PI*180);
    }

    if (frame_detected_left == true
            && frame_detected_right == true
            && (2* PI  + door_frame_right_angle - door_frame_left_angle)/PI*180 > 20
            && (2* PI  + door_frame_right_angle - door_frame_left_angle)/PI*180 < 60) //check if frame left/right have a certain angle in between
    {
        ROS_WARN("Door detected from back!");
        return true;
    }
    else{
        return false;
    }
}

//######### GET LOCLAIZATION POSE ##########################################################################
tf::Transform EnvironmentDetector::get_localization_pose()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "map";
    std::string sourceFrameID = "base_link";
    double waitTime = 0.5;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
            ROS_INFO("Got Transform from tf listener!");
            rtabmap_started_ = true;
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

    //debug
    std::cout << tf.getOrigin().x() << "\n" << tf.getOrigin().y() << "\n" << tf.getOrigin().z() << "\n";

    return tf;
}

//######### DEFINE: GET WORLD POSE ##########################################################################
tf::Transform EnvironmentDetector::get_world_pose()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "world";
    std::string sourceFrameID = "base_link";
    double waitTime = 1;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
            ROS_INFO("Got Transform world -> base_link from tf listener!");
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

    //debug
    std::cout << tf.getOrigin().x() << "\n" << tf.getOrigin().y() << "\n" << tf.getOrigin().z() << "\n";

    return tf;
}

//########## PUBLISH DEBUG IMAGE #####################
void EnvironmentDetector::publish_debug_image(cv::Mat img){
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    debug_image_pub_.publish(img_msg);
    ROS_INFO("Published debug image!");
}

//######### PUBLISH DEBUG POINTCLOUD #####################
sensor_msgs::PointCloud2 EnvironmentDetector::publish_debug_pointcloud(std::string pointcloud_path){

    //load pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pointcloud_path, *debug_cloud_ptr) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read pointcloud file!\n");
    }
    std::cout << "Loaded "
              << debug_cloud_ptr->width * debug_cloud_ptr->height
              << " data points from .pcd. "
              << std::endl;

    //convert to pc2 format and publish
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(*debug_cloud_ptr, pc2_msg);

    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.frame_id = "velodyne";

    debug_pointcloud_pub_.publish(pc2_msg);
    ROS_INFO("Published debug pointcloud!");

    return  pc2_msg;
}

//######## CONVERT PC2 to PCL ###############
pcl::PointCloud<pcl::PointXYZ>::Ptr EnvironmentDetector::pc2_to_pcl(sensor_msgs::PointCloud2 pointcloud){
    //conversion from PointCloud2 to pcl pointXYZ
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pointcloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    return temp_cloud;
}

//########### GET NUMBER OF FEATURE-MATCHES BETWEEN TWO IMAGES WITH AKAZE + SIFT #################
int EnvironmentDetector::get_num_matches_akaze_sift(int id, cv::Mat candidate_img, cv::Mat cam_img){

    const float nn_match_ratio = NNDR_akaze_;   // Nearest neighbor matching ratio

    if ( candidate_img.empty() || cam_img.empty() )
    {
        ROS_ERROR("Could not open or find the image!\n");
        return -1;
    }

    //convert both images to greyscale
    cv::cvtColor(candidate_img, candidate_img, CV_RGB2GRAY);
    cv::cvtColor(cam_img, cam_img, CV_RGB2GRAY);

    // detect with AKAZE
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    akaze->detectAndCompute(candidate_img, cv::noArray(), kpts1, desc1);
    akaze->detectAndCompute(cam_img, cv::noArray(), kpts2, desc2);

    bool features_detected = true;
    if ( desc1.empty() ){
         features_detected = false;
         ROS_WARN("1st descriptor (AKAZE) empty");
         return 0; //if no features detected return zero matches
    }
    if ( desc2.empty() ){
         features_detected = false;
         ROS_WARN("2nd descriptor (AKAZE) empty");
         return 0;
    }

    std::vector<cv::DMatch> good_matches;
    if(features_detected){
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector< std::vector<cv::DMatch> > nn_matches;
        matcher.knnMatch(desc1, desc2, nn_matches, 2);

        for(size_t i = 0; i < nn_matches.size(); i++) {
            cv::DMatch first = nn_matches[i][0];
            float dist1 = nn_matches[i][0].distance;
            float dist2 = nn_matches[i][1].distance;
            if(dist1 < nn_match_ratio * dist2) {
                good_matches.push_back(nn_matches[i][0]);
            }
        }
    }

    //detect with SIFT
    int num_features = 1000;
    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(num_features);
    cv::Ptr<cv::DescriptorMatcher> matcher_sift = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute( candidate_img, cv::noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( cam_img, cv::noArray(), keypoints2, descriptors2 );

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used

    features_detected = true;
    if ( descriptors1.empty() ){
         //cvError(0,"EnvironmentDetector","1st descriptor (SIFT) empty",__FILE__,__LINE__);
         ROS_WARN("1st descriptor (SIFT) empty");
         features_detected = false;
         return 0;
    }
    if ( descriptors2.empty() ){
         //cvError(0,"EnvironmentDetector","2nd descriptor(SIFT) empty",__FILE__,__LINE__);
         features_detected = false;
         ROS_WARN("2nd descriptor (SIFT) empty");
         return 0;
    }

    //-- Filter matches using the Lowe's ratio test
    std::vector<cv::DMatch> good_matches_sift;

    if(features_detected){
        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher_sift->knnMatch( descriptors1, descriptors2, knn_matches, 2 );

        const float ratio_thresh = NNDR_sift_;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches_sift.push_back(knn_matches[i][0]);
            }
        }
    }

    ROS_INFO("Number of good matches AKAZE: %i", good_matches.size());
    ROS_INFO("Number of good matches SIFT: %i", good_matches_sift.size());
    ROS_INFO("Number of good matches AKAZE + SIFT: %i", good_matches_sift.size()+good_matches.size());

    //-- Draw matches
    std::vector<cv::KeyPoint> keypoints_1_all, keypoints_2_all;

    cv::Mat img_matches;
    //draw AKAZE
    cv::drawMatches( candidate_img, kpts1, cam_img, kpts2, good_matches, img_matches, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //draw SIFT
    cv::drawMatches( candidate_img, keypoints1, cam_img, keypoints2, good_matches_sift, img_matches, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //Publish matches image on debug topic
    //-- Show detected matches
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_matches);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    debug_image_pub_.publish(img_msg);
    debug_feature_matching_img_pub_.publish(img_msg);

    return good_matches_sift.size()+good_matches.size();
}

//######## CALCULATES THE MEDIAN OF VECTOR #############
double EnvironmentDetector::get_median(std::vector<int> vector){
    std::size_t size = vector.size();

    if (size == 0)
    {
      return 0;  // Undefined, really.
    }
    else
    {
      std::sort(vector.begin(), vector.end());
      if (size % 2 == 0)
      {
        return (vector[size / 2 - 1] + vector[size / 2]) / 2;
      }
      else
      {
        return vector[size / 2];
      }
    }
}

//######### GET TRANSFORM BETWEEN TWO POINTCLOUDS BY ICP-REGISTRATION ####################
tf::Transform EnvironmentDetector::get_transform_from_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_velodyne, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_candidate){

    //Rotate the pointcloud candidate about 180 deg around z-axes
    Eigen::Affine3f transform_180_deg = Eigen::Affine3f::Identity();
    double theta =  3.14159265359;
    transform_180_deg.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pointcloud_candidate (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*pointcloud_candidate, *transformed_pointcloud_candidate, transform_180_deg);

    //compute transformatin between current pointcloud from velodyne to prevoius recorded pointcloud from lp candidate
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pointcloud_velodyne);
    icp.setInputTarget(transformed_pointcloud_candidate);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    double fitness_score;
    if(icp.hasConverged()){
        fitness_score = icp.getFitnessScore();
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        fitness_score << std::endl;
        ROS_WARN("SCORE: %f", fitness_score);

        //if fitness score is too high -> not a good match -> no loop closure
        if(fitness_score > icp_fitness_score_thr_){
            loop_closure_detected_ = false;
            ROS_WARN("Fitness score is over threshold of 0.5. Loop closure rejected!");
        }
	else{
             ROS_WARN("Fitness score is under threshold of 0.5. Loop closure accepted!");
	}

    }
    else {
        ROS_ERROR("Error aligning Pointclouds with ICP!");
        loop_closure_detected_ = false; //jump into case: new map
        tf::Transform empty;
        return empty;
    }
    std::cout << icp.getFinalTransformation() << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();


    //publish debug pointcloud
    //convert to pc2 format and publish
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(Final, pc2_msg);

    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.frame_id = "velodyne";

    debug_icp_registered_pointcloud_pub_.publish(pc2_msg);
    ROS_INFO("Published debug icp registration pointcloud!");

    //convert Eigen transformation to ros tf::Transform
    tf::Vector3 origin;
    origin.setValue(static_cast<double>(transformation(0,3)),static_cast<double>(transformation(1,3)),static_cast<double>(transformation(2,3)));

    tf::Matrix3x3 tf_3d;
    tf_3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)),
                   static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)),
                   static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

    tf::Quaternion tf_q;
    tf_3d.getRotation(tf_q);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tf_q);

    return transform;
}

//######### GET TRANSFORM BETWEEN TWO POINTCLOUDS BY ICP-REGISTRATION ####################
tf::Transform EnvironmentDetector::get_loop_closure_pose(LinkPoint lp_candidate, tf::Transform transform_icp){
    //transform current position of robot in coordinate frame of linkpoint candidates map
    tf::Transform orig_to_lp_candidate;
    tf::Vector3 translation(lp_candidate.get_pos_x(0), lp_candidate.get_pos_y(0), 0);
    orig_to_lp_candidate.setOrigin(translation);
    tf::Quaternion rotation_q(0, 0, lp_candidate.get_q_z_from_theta(0), lp_candidate.get_q_w_from_theta(0));
    orig_to_lp_candidate.setRotation(rotation_q);

    //rotation of the pointcloud about 180 deg
    tf::Transform rotation_180deg;
    rotation_180deg.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion rot_180(0, 0, 3.14159265359); //rotation of pi around z
    rotation_180deg.setRotation(rot_180);

    //transformation between rotated pointcloud and position of robot with ICP
    tf::Transform transform_icp_inverted = transform_icp.inverse(); //TODO: not sure about the inverse -> check out!!!

    //Multiply all transformations to get final transformation
    tf::Transform final_transformation = orig_to_lp_candidate * rotation_180deg * transform_icp_inverted;
    return final_transformation;
}

//################## CHECK FOR NEW MAP WITH FEATURE DETECTION ##########################
int EnvironmentDetector::check_for_new_map(){
    //calculate medians of matches for all candidates
    std::vector<double> matches_median; //medians of matches from all candidates
    int num_linkpoints = linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS");
    ROS_INFO("Number of possible linkpoints: %i", num_linkpoints);

    //debug
    std::cout << "loop_closure_search_radius_with_variance_:\n" << loop_closure_search_radius_with_variance_  << "\n";

    for(int id=1; id<=num_linkpoints; id++){

        //implement linkpoint filtering by world pose
        if(EnvironmentDetector::lp_in_search_radius(lp_db_url_, id-1, loop_closure_search_radius_with_variance_, false) || no_graph_connection_){
            if(no_graph_connection_){
                  std::cout << "Linkpoint is still considered because no_graph_connection_ == true";
            }
            cv::Mat lp_img_front = linkpoint_db_.get_linkpoint_image(lp_db_url_, id-1, "IMAGE_FRONT");
            cv::Mat lp_img_rear = linkpoint_db_.get_linkpoint_image(lp_db_url_, id-1, "IMAGE_REAR");
            std::vector<int> matches_for_candidate; //to store the 10 results for one candidate
            for (int k=0; k<=num_last_imgs_; k++) {
                //compute sum of matches from front and rear cam -> maximum one of both will match, but sum is then high for linkpoint
                matches_for_candidate.push_back(EnvironmentDetector::get_num_matches_akaze_sift(id-1, lp_img_front, last_n_images_[k])
                                                + EnvironmentDetector::get_num_matches_akaze_sift(id-1, lp_img_rear, last_n_images_[k])
                                                + EnvironmentDetector::get_num_matches_akaze_sift(id-1, lp_img_front, last_n_rear_images_[k])
                                                + EnvironmentDetector::get_num_matches_akaze_sift(id-1, lp_img_rear, last_n_rear_images_[k]));
            }
            matches_median.push_back(EnvironmentDetector::get_median(matches_for_candidate));
        }
	else{
	  matches_median.push_back(0);
	}
    }

    //detect if one median is over the detection threshold
    int num_matches_thr = 30;
    int loop_closure_lp_id = -1;
    for (int k = 0; k<matches_median.size(); k++) {
        //debug
        std::cout << "matches_median[" << k <<"] = " << matches_median[k] << "\n\n";
        if(matches_median[k] > num_matches_thr){
            loop_closure_lp_id = k;
            num_matches_thr = matches_median[k]; //in case there is even a larger one
            ROS_INFO("Existing map detected by visual feature matching with linkpoint id = %i!", loop_closure_lp_id);
            ROS_INFO("Entry %i of matches_median with value %d was over threshold", k, matches_median[k]);
        }
    }
    return loop_closure_lp_id;
}

//################## COMPUTE THETA FROM QUATERNION ##########################
double EnvironmentDetector::theta_from_q(tf::Quaternion q){
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

//################## CHECK DISTANCE FROM LINKPOINT(CANDIDATE) TO ROBOT ##########################
bool EnvironmentDetector::lp_in_search_radius(std::string url, int lp_id, double search_radius, bool candidate){
    LinkPoint lp;
    if(candidate){
        lp = linkpoint_db_.get_linkpoint_candidate(url, lp_id);
    }
    else{
        lp = linkpoint_db_.get_linkpoint(url, lp_id);
    }

    tf::Transform robot_pose_world = EnvironmentDetector::get_world_pose();

    double delta_x = robot_pose_world.getOrigin().x() - lp.get_pos_x(2);
    double delta_y = robot_pose_world.getOrigin().y() - lp.get_pos_y(2);
    double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

    if(distance <= search_radius){
        std::cout << "Linkpoint (candidate) with id = " << lp_id << " is in search radius and considered for feature matching\n";
        return true;
    }
    else{
        std::cout << "Linkpoint (candidate) with id = " << lp_id << " is NOT in search radius and therefore NOT considered for feature matching\n";
        return false;
    }
}

//########### SAY TEXT ######################
void EnvironmentDetector::say_text(std::string text){
    sound_play::SoundRequest sound_msg;
    sound_msg.command = 1; //play sound once
    sound_msg.sound = -3; //say
    sound_msg.volume = 1.0;
    sound_msg.arg = text; //text to say
    sound_msg.arg2 = "voice_kal_diphone";

    sound_saying_pub_.publish(sound_msg);
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "environment_detector");

    ros::NodeHandle node_handle;
    EnvironmentDetector environment_detector(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
