#include "sm_mapping/sm_mapping.h"


//########## CONSTRUCTOR ###############################################################################################
SMMapping::SMMapping(ros::NodeHandle &node_handle):
    node_(&node_handle)
{

    // === PARAMETERS ===
    node_->param("/initial_variance_x", var_x_, 0.003);
    node_->param("/initial_variance_y", var_y_, 0.003);
    node_->param("/initial_variance_theta", var_theta_, 0.003);
    node_->param("/use_global_tracker", use_global_tracker_, false);
    node_->param("/record_imgs_and_pc", record_imgs_and_pc_, false);

    ROS_WARN("Parameters:");
    std::cout << "initial_variance_x: " << var_x_ << "\n";
    std::cout << "initial_variance_y: " << var_y_ << "\n";
    std::cout << "initial_variance_theta: " << var_theta_ << "\n";
    std::cout << "use_global_tracker: " << use_global_tracker_ << "\n";
    std::cout << "record_imgs_and_pc: " << record_imgs_and_pc_ << "\n";


    // === SUBSCRIBERS ===
    velodyne_pointcloud_subscriber_ = node_->subscribe("/velodyne_points", 1, &SMMapping::VelodynePointcloudSubscriberCallback, this);
    rear_cam_image_subscriber_ = node_->subscribe("/cam_back/color/image_raw", 1, &SMMapping::RearCamImageSubscriberCallback, this);
    front_cam_image_subscriber_ = node_->subscribe("/cam_front/color/image_raw", 1, &SMMapping::FrontCamImageSubscriberCallback, this);

    // === SERVICE SERVERS ===
    set_linkpoint_service_server_ = node_->advertiseService("sm_mapping/set_linkpoint", &SMMapping::setLinkpointServiceCallback, this);
    save_linkpoint_rear_image_service_server_ = node_->advertiseService("sm_mapping/save_linkpoint_rear_img", &SMMapping::recordLinkpointImageServiceCallback, this);
    set_linkpoint_candidate_service_server_ = node_->advertiseService("sm_mapping/set_linkpoint_candidate", &SMMapping::setLinkpointCandidateServiceCallback, this);

    // === PUBLISHERS ===
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
    my_timer_ = node_->createTimer(ros::Duration(0.1), &SMMapping::timerCallback, this);

    State_Case = Initialize;
    end_mapping_ = false;
    loop_closure_map_id_ = -1;
    no_graph_connection_ = false;
    record_depth_images_ = false;
    init_pose_published_ = false;
    on_origin_map_ = false;
    use_old_map_ = false;
}

//######### DEFINE: GET LOCALIZATION POSE ##########################################################################
tf::Transform SMMapping::get_localization_pose()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "map";
    std::string sourceFrameID = "base_link";
    double waitTime = 1.5;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
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

//######### DEFINE: GET WORLD POSE ##########################################################################
tf::Transform SMMapping::get_world_pose()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "world";
    std::string sourceFrameID = "base_link";
    double waitTime = 1.5;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
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

// ################ CALLBACK: VELODYNE POINCLOUD ############################################
void SMMapping::VelodynePointcloudSubscriberCallback(const sensor_msgs::PointCloud2 &msg){
    velodyne_pointcloud_msg_ = msg;
}

//########## CALLBACK: SUBSCRIBER REAR CAM IMAGE ############################################################################
void SMMapping::RearCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg)
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
}

//########## CALLBACK: SUBSCRIBER FRONT CAM IMAGE ############################################################################
void SMMapping::FrontCamImageSubscriberCallback(const sensor_msgs::ImageConstPtr& image_msg)
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
}


//########## CALLBACK: TIMER (STATE MACHINE) ###########################################################################################
void SMMapping::timerCallback(const ros::TimerEvent &evt)
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
                if(use_global_tracker_){
                    global_tracker_.initialize();
                    SMMapping::say_text("Using global tracker");
                    ros::Duration(3.0).sleep();
                }

                SMMapping::say_text("Initialized");

                State_Case = Main_Menu;
                break;
            }

            // ---------------------------------------------------------
            case Start_Mapping:
            {
                ROS_INFO("STATE: Start mapping");

                std::string start;
                start = text_io_.get_mapping_start();

                if(start == "M"){
                    State_Case = Main_Menu;
                    break;
                }

                if(start == "S"){
                    cur_map_name_ = "map_" + std::to_string(map_index_);
                    cur_database_path_ = working_dir_ + "/data/" + std::to_string(map_index_) + "_" + cur_map_name_ + ".db";
                    map_saving_path_ = working_dir_ + "/data/" + std::to_string(map_index_) + "_" + cur_map_name_;
                    ros::param::set("/rtabmap/rtabmap/database_path", cur_database_path_);
                    std::cout << "Creating database:" << cur_database_path_ << "\n\n";

                    //ask for configuration (indoor/outdoor)
                    if(text_io_.get_indoor_info()){
                        indoor_ = true;
                    }
                    else{
                        indoor_ = false;
                    }

                    //starting rtabmap
                    std_msgs::Bool msg;
                    msg.data = true;
                    if(indoor_){
                        rtabmap_mapping_indoor_pub_.publish(msg); //start rtabmap in mapping mode indoor
                    }
                    else {
                        rtabmap_mapping_outdoor_pub_.publish(msg); //start rtabmap in mapping mode outdoor
                    }

                    SMMapping::say_text("I am going to start RTAB MAP now");
                    ros::Duration(5.0).sleep(); // wait 15 sec to let rtabmap start

                    if(use_global_tracker_){
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
                        else if(map_index_ == 0){
                            on_origin_map_ = true;
                        }
                    }

                    //Insert map entry into linkpoint.db
                    linkpoint_db_.insert_map(lp_db_url_, map_index_, cur_map_name_, cur_database_path_, indoor_);
                    std::string text_saying = "Entry for map with I D " + std::to_string(map_index_) + "inserted into database.";
                    SMMapping::say_text(text_saying);
                    ros::Duration(2.0).sleep();

                    if(use_global_tracker_){
                        global_tracker_.reset_rtabmap_start_bools();
                    }

                    //ask for initial pose
                    int init_pose_lp_id = text_io_.get_initial_pose_lp();
                    if(init_pose_lp_id >= 0){
                        init_pose_ = SMMapping::get_init_pose(lp_db_url_, init_pose_lp_id, map_index_);
                        init_pose_published_ = false;
                    }
                    else{
                        init_pose_published_ = true; //no need to publish init pose
                    }

                    //publish current map id for rviz graph visualization
                    std_msgs::Int16 map_id_msg;
                    map_id_msg.data = map_index_;
                    cur_map_id_pub_.publish(map_id_msg);

                    end_mapping_ = false;
                    State_Case = Mapping;
                    ROS_INFO("STATE: Mapping");
                    break;
                }
            }

            // ---------------------------------------------------------
            case Mapping:
            {
                ros::Duration(0.2).sleep();

                if(use_global_tracker_){
                    //update pose for global tracker and publish
                    if(on_origin_map_){ //when only the local pose is required
                        geometry_msgs::PoseWithCovarianceStamped local_pose_msg;
                        local_pose_msg = global_tracker_.get_local_pose();
                        global_pose_pub_.publish(local_pose_msg);
                        world_pose_ = local_pose_msg;

                        //broadcast also world frame (which is equal the map frame in this case)
                        tf::Transform world_eq_map_transform;
                        world_eq_map_transform.setOrigin(tf::Vector3(0, 0, 0)); //initialize to origin
                        world_eq_map_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
                        static tf::TransformBroadcaster br;
                        br.sendTransform(tf::StampedTransform(world_eq_map_transform, ros::Time::now(), "world", "map"));
                        if(global_tracker_.check_if_rtabmap_started()){
                            SMMapping::say_text("RTABMAP has started, finally. Start now moving the robot.");
                        }
                    }
                    else{
                        geometry_msgs::PoseWithCovarianceStamped global_pose_msg;
                        global_pose_msg = global_tracker_.get_global_pose();
                        global_pose_pub_.publish(global_pose_msg);
                        global_tracker_.broadcast_tf_from_world_origin_to_current_map();
                        world_pose_ = global_pose_msg;
                        if(global_tracker_.check_if_rtabmap_started()){
                          SMMapping::say_text("RTABMAP has started, finally. Start now moving the robot.");
                        }
                     }
                }

                //set initial pose (if required)
                if(!init_pose_published_){
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
                    SMMapping::say_text("Set initial pose.");

                    global_tracker_.print_pose(init_pose_.pose);
                }

                //wait until service was called which ends mapping
                if(end_mapping_){
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

                //save map by map server to pgm and yaml for further usage
                //start rtabmap_ros
                //ROS_INFO("Saving map now to pgm and yaml.");
                //std::string start_rtabmap_ros = "rosrun rtabmap_ros rtabmap _database_path:=" + cur_database_path_;
                //const char * start_rtabmap_ros_char = start_rtabmap_ros.c_str();
                //system(start_rtabmap_ros_char);
                //ros::Duration(5.0).sleep(); // wait until rtabmap has started

                //std::string system_call = "rosrun map_server map_saver "+map_saving_path_+":=proj_map";
                //const char * system_call_char = system_call.c_str();
                //system(system_call_char);
                //ros::Duration(2.0).sleep(); // wait until map server has started

                //system("rosservice call /publish_map 1 1 0");
                //ROS_INFO("Saved map");
                //SMMapping::say_text("Saved map!");
                //ros::Duration(2.0).sleep();

                State_Case=Main_Menu;
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
                      map_index_ = text_io_.get_new_map_id(lp_db_url_);
                      use_old_map_ = false;
                      State_Case = Start_Mapping;
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
void SMMapping::say_text(std::string text){
    sound_play::SoundRequest sound_msg;
    sound_msg.command = 1; //play sound once
    sound_msg.sound = -3; //say
    sound_msg.volume = 1.0;
    sound_msg.arg = text; //text to say
    sound_msg.arg2 = "voice_kal_diphone";

    sound_saying_pub_.publish(sound_msg);
}



// ############# GET INIT POSE #############################
geometry_msgs::PoseWithCovarianceStamped SMMapping::get_init_pose(std::string url, int init_pose_linkpoint_id, int map_id){
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
        tf::Quaternion q_rot;
        double r=0, p=0, y=3.14159;  // Rotate the previous pose by 180* about Z
        q_rot.setRPY(r, p, y);

        q_orig = q_rot*q_orig;  // Calculate the new orientation
        q_orig.normalize();
    }

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

//########### SERVICE SERVER: SET LINKPOINT ############################################################################
bool SMMapping::setLinkpointServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("In service callback SET LINKPOINT");

    // ask if loop closure with linkpoint candidate exists
    int loop_closure_lp_candidate = text_io_.get_lp_candidate_id();

    if(loop_closure_lp_candidate == -1){
        //creates new linkpoint on new map
        local_pose_ = SMMapping::get_localization_pose();
        LinkPoint lp_new_map;
        lp_new_map.set_id(text_io_.get_new_linkpoint_id(lp_db_url_)); //set id after the next number of linkpoints
        lp_new_map.set_tag(text_io_.get_lp_tag());
        lp_new_map.set_pos_x(local_pose_.getOrigin().x(), 0);
        lp_new_map.set_pos_y(local_pose_.getOrigin().y(), 0);
        lp_new_map.set_theta_from_z_w(local_pose_.getRotation().z(), local_pose_.getRotation().w(), 0);
        lp_new_map.set_map_id(map_index_, 0);

        //set global pose if global tracker is activated
        if(use_global_tracker_){
            lp_new_map.set_pos_x(world_pose_.pose.pose.position.x, 2);
            lp_new_map.set_pos_y(world_pose_.pose.pose.position.y, 2);
            lp_new_map.set_theta_from_z_w(world_pose_.pose.pose.orientation.z, world_pose_.pose.pose.orientation.w, 2);
        }
        else{
            lp_new_map.set_pos_x(0, 2);
            lp_new_map.set_pos_y(0, 2);
            lp_new_map.set_theta_from_z_w(0, 1, 2);
        }

        lp_new_map.set_difficulty(1);
        if(indoor_){
            lp_new_map.set_environment(1);} //1=indoor
        else {
            lp_new_map.set_environment(0);} //0=outdoor

        //the linkpoint on the new map is also the origin
        lp_new_map.set_pos_x(0, 1);
        lp_new_map.set_pos_y(0, 1);
        lp_new_map.set_theta_from_z_w(0, 1, 1);
        int new_map_id = text_io_.get_new_map_id(lp_db_url_);
        lp_new_map.set_map_id(new_map_id, 1); //set the next id with respect to the existing maps
        lp_new_map.set_direction(new_map_id); //set direction from current map to next map

        lp_new_map.print_linkpoint_properties();

        ROS_INFO("Inserting linkpoint now!");
        if(record_imgs_and_pc_){
            linkpoint_db_.insert_linkpoint_with_images(lp_db_url_, lp_new_map, cam_back_image_msg_, rear_img_of_lp_);
        }
        else {
            linkpoint_db_.insert_linkpoint(lp_db_url_, lp_new_map);
        }
        SMMapping::say_text("Saved new linkpoint.");
    }
    else{
        // creates linkpoint with existing linkpoint candidate
        // save new linkpoint with position on current map and old linkpoint candidates position on old map
        local_pose_ = SMMapping::get_localization_pose();
        LinkPoint lp_loop_closure;
        lp_loop_closure.set_id(linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS")); //set id after the next number of linkpoints
        lp_loop_closure.set_tag(text_io_.get_lp_tag());
        lp_loop_closure.set_pos_x(local_pose_.getOrigin().x(), 0);
        lp_loop_closure.set_pos_y(local_pose_.getOrigin().y(), 0);
        lp_loop_closure.set_theta_from_z_w(local_pose_.getRotation().z(), local_pose_.getRotation().w(), 0);
        lp_loop_closure.set_map_id(map_index_, 0);

        if(use_global_tracker_){
            lp_loop_closure.set_pos_x(world_pose_.pose.pose.position.x, 2);
            lp_loop_closure.set_pos_y(world_pose_.pose.pose.position.y, 2);
            lp_loop_closure.set_theta_from_z_w(world_pose_.pose.pose.orientation.z, world_pose_.pose.pose.orientation.w, 2);
        }
        else{
            lp_loop_closure.set_pos_x(0, 2);
            lp_loop_closure.set_pos_y(0, 2);
            lp_loop_closure.set_theta_from_z_w(0, 1, 2);
        }

        lp_loop_closure.set_difficulty(1);
        if(indoor_){
            lp_loop_closure.set_environment(1);} //1=indoor
        else {
            lp_loop_closure.set_environment(0);} //0=outdoor

        //retrieve linkpoint candidate from database, compute current position of robot in coordinate frame of linkpoint candidates map
        LinkPoint lp_candidate = linkpoint_db_.get_linkpoint_candidate(lp_db_url_, loop_closure_lp_candidate);
        lp_loop_closure.set_pos_x(lp_candidate.get_pos_x(0), 1);
        lp_loop_closure.set_pos_y(lp_candidate.get_pos_y(0), 1);
        lp_loop_closure.set_theta(lp_candidate.get_theta(0), 1);
        lp_loop_closure.set_map_id(lp_candidate.get_map_id(0), 1);
        lp_loop_closure.set_direction(lp_candidate.get_map_id(0)); //Direction is from current map to map with lp candidate

        ROS_INFO("Inserting linkpoint now!");
        if(record_imgs_and_pc_){
            linkpoint_db_.insert_linkpoint_with_images(lp_db_url_, lp_loop_closure, cam_back_image_msg_, rear_img_of_lp_);
        }
        else {
            linkpoint_db_.insert_linkpoint(lp_db_url_, lp_loop_closure);
        }
        SMMapping::say_text("Saved new linkpoint.");
        lp_loop_closure.print_linkpoint_properties();
    }

    //end the mapping
    end_mapping_ = true;
    return true;
}

//########### SERVICE SERVER: RECORD LINKPOINT REAR IMAGE ############################################################################
bool SMMapping::recordLinkpointImageServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //save image in front of the linkpoint for compatibility to automated_sm_mapping approach
    rear_img_of_lp_ = cam_rear_image_msg_;
    ROS_INFO("Saved rear image in front of Linkpoint!");
    return true;
}

//########### SERVICE SERVER: SET LINKPOINT CANDIDATE ############################################################################
bool SMMapping::setLinkpointCandidateServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //save linkpoint candidate on current map for loop closure detection

    //ask user for linkpoint candidate id
    int lp_candidate_id = text_io_.get_new_linkpoint_candidate_id(lp_db_url_);

    local_pose_ = SMMapping::get_localization_pose();
    LinkPoint lp_candidate;
    lp_candidate.set_id(lp_candidate_id);
    lp_candidate.set_pos_x(local_pose_.getOrigin().x(), 0);
    lp_candidate.set_pos_y(local_pose_.getOrigin().y(), 0);
    lp_candidate.set_theta_from_z_w(local_pose_.getRotation().z(), local_pose_.getRotation().w(), 0);
    lp_candidate.set_map_id(map_index_, 0);

    if(use_global_tracker_){
        lp_candidate.set_pos_x(world_pose_.pose.pose.position.x, 2);
        lp_candidate.set_pos_y(world_pose_.pose.pose.position.y, 2);
        lp_candidate.set_theta_from_z_w(world_pose_.pose.pose.orientation.z, world_pose_.pose.pose.orientation.w, 2);
    }
        else{
            lp_candidate.set_pos_x(0, 2);
            lp_candidate.set_pos_y(0, 2);
            lp_candidate.set_theta_from_z_w(0, 1, 2);
        }

    lp_candidate.set_difficulty(1);
    if(indoor_){
        lp_candidate.set_environment(1);} //1=indoor
    else {
        lp_candidate.set_environment(0);} //0=outdoor

    lp_candidate.print_linkpoint_properties();

    if(record_imgs_and_pc_){
        std::string cur_pointcloud_path = pointcloud_dir_ + "/" + std::to_string(lp_candidate_id) + "_pointcloud.pcd";
        linkpoint_db_.insert_linkpoint_candidate(lp_db_url_, lp_candidate, cam_rear_image_msg_, cur_pointcloud_path);

        //convert cloud to pcl and save it under cur_pointcloud_path
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = SMMapping::pc2_to_pcl(velodyne_pointcloud_msg_);
        pcl::io::savePCDFileASCII (cur_pointcloud_path, *cloud);
        std::cerr << "Saved " << cloud->points.size () << " data points to " << cur_pointcloud_path << "." << std::endl;
    }
    else{
        linkpoint_db_.insert_linkpoint_candidate(lp_db_url_, lp_candidate, cam_rear_image_msg_, ""); //TODO: find alternative that cam_rear_image_msg_ does not have to be saved
    }

    SMMapping::say_text("New linkpoint candidate saved.");

    return true;
}

//######## CONVERT PC2 to PCL ###############
pcl::PointCloud<pcl::PointXYZ>::Ptr SMMapping::pc2_to_pcl(sensor_msgs::PointCloud2 pointcloud){
    //conversion from PointCloud2 to pcl pointXYZ
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pointcloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    return temp_cloud;
}


//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sm_mapping");

    ros::NodeHandle node_handle;
    SMMapping sm_mapping(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
