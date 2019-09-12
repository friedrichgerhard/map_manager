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
 * @file   global_tracker.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class for providing the feature to track the global robot position over multiple maps. Generally, a tf tree is created by composing the transforms between the map origins.
 *
 */

#include "globaltracker/globaltracker.h"

tf::Transform GlobalTracker::add_tf_to_global_tf(tf::Transform global_tf, tf::Transform local_tf){
    tf::Transform new_global_tf = global_tf * local_tf;
    //pose_cov_ops::compose()
    return new_global_tf;
}

tf::Transform GlobalTracker::get_local_tf_from_lp(std::string url, int id, int previous_map_id){
    LinkPoint lp;
    lp = linkpoint_db_.get_linkpoint(url, id);

    int index; //index of current map
    int index_next;
    double lp_pos_x;
    double lp_pos_y;
    double lp_rot_z;
    double lp_rot_w;

    if(previous_map_id == lp.get_map_id(0)){
        index = 0;
        index_next = 1;
    }
    else if (previous_map_id == lp.get_map_id(1)) {
        index = 1;
        index_next = 0;
    }
    else {
        std::cerr << "[Global Tracker]: Prevoius map_id does not match with given linkpoint!!!";
    }

    //add transformation from previous origin to connecting linkpoint
    lp_pos_x = lp.get_pos_x(index);
    lp_pos_y = lp.get_pos_y(index);
    lp_rot_z = lp.get_q_z_from_theta(index);
    lp_rot_w = lp.get_q_w_from_theta(index);

    tf::Transform tf_to_connecting_lp;
    tf_to_connecting_lp.setOrigin(tf::Vector3(lp_pos_x, lp_pos_y, 0));
    tf_to_connecting_lp.setRotation(tf::Quaternion(0, 0, lp_rot_z, lp_rot_w));

    //add transformation from connecting linkpoint to new origin
    //search for the origin on the next map, therefore search the linkpoint which is the map origin
    int next_map_id = lp.get_map_id(index_next);

    double lp_pos_next_map_x = lp.get_pos_x(index_next);
    double lp_pos_next_map_y = lp.get_pos_y(index_next);
    double lp_rot_next_map_z = lp.get_q_z_from_theta(index_next);
    double lp_rot_next_map_w = lp.get_q_w_from_theta(index_next);

    tf::Transform tf_to_connecting_lp_next_map;
    tf_to_connecting_lp_next_map.setOrigin(tf::Vector3(lp_pos_next_map_x, lp_pos_next_map_y, 0));
    tf_to_connecting_lp_next_map.setRotation(tf::Quaternion(0, 0, lp_rot_next_map_z, lp_rot_next_map_w));

    //computing final transformation between the origins of both connecting maps
    tf::Transform local_tf = tf_to_connecting_lp * tf_to_connecting_lp_next_map.inverse();

    std::cout << "tf from old map origin to connecting linkpoint:\n";
    GlobalTracker::print_tf_transform(tf_to_connecting_lp);

    std::cout << "tf from connecting linkpoint to new map_origin:\n";
    GlobalTracker::print_tf_transform(tf_to_connecting_lp_next_map.inverse());

    std::cout << "local tf from old map origin to new map_origin [local_tf = tf_to_connecting_lp * tf_to_connecting_lp_next_map.inverse()]:\n";
    GlobalTracker::print_tf_transform(local_tf);

    return  local_tf;
}


geometry_msgs::PoseWithCovariance GlobalTracker::get_local_pose_from_lp(std::string url, int id, int previous_map_id){
    tf::Transform local_tf = GlobalTracker::get_local_tf_from_lp(url, id, previous_map_id);
    geometry_msgs::PoseWithCovariance local_pose;
    local_pose.pose.position.x = local_tf.getOrigin().x();
    local_pose.pose.position.y = local_tf.getOrigin().y();
    local_pose.pose.position.z = local_tf.getOrigin().z();

    local_pose.pose.orientation.x = local_tf.getRotation().x();
    local_pose.pose.orientation.y = local_tf.getRotation().y();
    local_pose.pose.orientation.z = local_tf.getRotation().z();
    local_pose.pose.orientation.w = local_tf.getRotation().w();

    std::array<double, 36> covariance_matrix = GlobalTracker::get_covariance_matrix_diag();
    for(int i = 0; i < 36; i++){
        local_pose.covariance.at(i) = covariance_matrix[i];
    }

    return local_pose;
}


tf::Transform GlobalTracker::add_new_element_to_tf_path(std::string url, int linkpoint_id, int previous_map_id){

    //compute transform between origin from prevoius map to connecting linkpoint
    tf::Transform local_tf;
    local_tf = GlobalTracker::get_local_tf_from_lp(url, linkpoint_id, previous_map_id);

    //add to local tf the transformation from linkpoint to origin of the new map

    geometry_msgs::PoseWithCovariance local_pose = GlobalTracker::get_local_pose_from_lp(url, linkpoint_id, previous_map_id);

    //add new tf to tf path
    tf_to_global_origin_ = GlobalTracker::add_tf_to_global_tf(tf_to_global_origin_, local_tf); //Multiplies both tf

    //calucalte new global origin of current map
    // p = p1 (+) p2
    // Pose composition, including uncertainty in both p1 and p2:
    pose_cov_ops::compose(map_pose_in_world_, local_pose,  map_pose_in_world_);
    std::cout << "\map_pose_in_world_ after multiplication:\n";
    GlobalTracker::print_pose(map_pose_in_world_);

    ROS_INFO("[Global Tracker]: Added new map transform (map_id = %i) to tf_to_global_path_ with use of link_point %i", previous_map_id, linkpoint_id);
    return tf_to_global_origin_;
}


tf::Transform GlobalTracker::get_local_tf(){
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "map";
    std::string sourceFrameID = "base_link";
    double waitTime = 0.5;

    bool bTransformAvailable = true;
    tf::Transform tf;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
            ROS_INFO("[Global Tracker]: Got Transform from tf listener!");
            rtabmap_status_t_ = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("[Global Tracker]: %s",ex.what());
        }
        tf.setRotation(transformStamped.getRotation());
        tf.setOrigin(transformStamped.getOrigin());
    }
    else{
        ROS_WARN("[Global Tracker]: Transform from '%s' to '%s' not available!",sourceFrameID.c_str(),targetFrameID.c_str());
        rtabmap_status_t_ = false;
        tf.setOrigin(tf::Vector3(0, 0, 0)); //insert, so that there is no error
        tf.setRotation(tf::Quaternion(0, 0, 0, 1));
    }
    
    if(rtabmap_status_t_ != rtabmap_status_previous_t_){
        rtabmap_started_ = true;
    }
    rtabmap_status_previous_t_ = rtabmap_status_t_;

    return tf;
}


geometry_msgs::PoseWithCovarianceStamped GlobalTracker::get_global_pose(){
    geometry_msgs::PoseWithCovariance global_robot_pose;
    geometry_msgs::PoseWithCovariance local_pose;
    tf::Transform local_tf = get_local_tf();

    local_pose.pose.position.x = local_tf.getOrigin().x();
    local_pose.pose.position.y = local_tf.getOrigin().y();
    local_pose.pose.position.z = local_tf.getOrigin().z();
    local_pose.pose.orientation.x = local_tf.getRotation().x();
    local_pose.pose.orientation.y = local_tf.getRotation().y();
    local_pose.pose.orientation.z = local_tf.getRotation().z();
    local_pose.pose.orientation.w = local_tf.getRotation().w();

    //fill the covariance matrix
    std::array<double, 36> covariance_matrix = GlobalTracker::get_covariance_matrix_diag();
    for(int i = 0; i < 36; i++){
        local_pose.covariance.at(i) = covariance_matrix[i];
    }

    //compose both poses -> calculate global robot pose with covariance
    pose_cov_ops::compose(map_pose_in_world_, local_pose,  global_robot_pose);

    std::cout << "\nGlobal Robot Pose:\n";
    GlobalTracker::print_pose(global_robot_pose);

    geometry_msgs::PoseWithCovarianceStamped global_robot_pose_msg;
    global_robot_pose_msg.pose = global_robot_pose;
    global_robot_pose_msg.header.stamp = ros::Time::now();

    return global_robot_pose_msg;
}


void GlobalTracker::initialize(){

    //initialize transform
    tf_to_global_origin_.setOrigin(tf::Vector3(0, 0, 0)); //initialize to origin
    tf_to_global_origin_.setRotation(tf::Quaternion(0, 0, 0, 1));

    //initialize global pose
    map_pose_in_world_.pose.position.x = 0;
    map_pose_in_world_.pose.position.y = 0;
    map_pose_in_world_.pose.position.z = 0;
    map_pose_in_world_.pose.orientation.x = 0;
    map_pose_in_world_.pose.orientation.y = 0;
    map_pose_in_world_.pose.orientation.z = 0;
    map_pose_in_world_.pose.orientation.w = 1;
    ROS_INFO("[Global Tracker]: Initialized!");
}


void GlobalTracker::print_tf_transform(tf::Transform transform){

    tf::Quaternion q(0,0,transform.getRotation().z(),transform.getRotation().w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout << "\nTranslation:\n"
              << "\nx = " << transform.getOrigin().x()
              << "\ny = " << transform.getOrigin().y()
              << "\nz = " << transform.getOrigin().z()
              << "\nRotation:\n"
              << "theta = " << yaw << "\n";
}


void GlobalTracker::print_pose(geometry_msgs::PoseWithCovariance pose){
    tf::Quaternion q(0,0,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout << "\nTranslation:\n"
              << "\nx = " << pose.pose.position.x
              << "\ny = " << pose.pose.position.y
              << "\nz = " << pose.pose.position.z
              << "\nRotation:\n"
              << "theta = " << yaw << "\n"
              << "\nCovariance:\n"
              << "cov_matrix = " << pose.covariance.elems << "\n";
}


tf::Transform GlobalTracker::build_global_tf_tree(std::string url, std::vector<int> topological_path, std::vector<double> variances){
    GlobalTracker::initialize(); //initialize global tracker before new tf tree build (reset tf_to_global_origin_)
    var_x_ = variances.at(0);
    var_y_ = variances.at(1);
    var_theta_ = variances.at(2);
    int previous_map_id = 0; //starting on map 0
    for(int i = 1; i < topological_path.size()-1; i++){

        //compute transform from map origin of previous map to map origin of next map (linkpoint which connects both maps in graph does not necessarily have to be the map origin)
        GlobalTracker::add_new_element_to_tf_path(url, topological_path[i], previous_map_id);

        std::cout << "Added linkpoint " << topological_path[i] << " to global tf.\n";
        std::cout << "tf_to_global_origin_:\n";
        GlobalTracker::print_tf_transform(tf_to_global_origin_);

        //don't search the last loop for the previous map id
        if(i < topological_path.size()-2){
            //find prevoius map id
            LinkPoint previous_lp = linkpoint_db_.get_linkpoint(url, topological_path[i]);
            LinkPoint next_lp = linkpoint_db_.get_linkpoint(url, topological_path[i]+1);

            //find out which map id both linkpoints share -> this is previous map id necessary to get the transform
            if(previous_lp.get_map_id(0) == next_lp.get_map_id(0)
            || previous_lp.get_map_id(0) == next_lp.get_map_id(1))
            {
                previous_map_id = previous_lp.get_map_id(0);
            }
            else if (previous_lp.get_map_id(1) == next_lp.get_map_id(0)
                  || previous_lp.get_map_id(1) == next_lp.get_map_id(1))
            {
                previous_map_id = previous_lp.get_map_id(1);
            }
        }
    }
}


void GlobalTracker::broadcast_tf_from_world_origin_to_current_map(){
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(tf_to_global_origin_, ros::Time::now(), "world", "map"));
  }


std::array<double, 36> GlobalTracker::get_covariance_matrix_diag(){
    std::array<double, 36> covariance_matrix = {                 var_x_,               0,             0,             0,             0,             0,
                                                                 0,                    var_y_,        0,             0,             0,             0,
                                                                 0,                    0,             0,             0,             0,             0,
                                                                 0,                    0,             0,             0,             0,             0,
                                                                 0,                    0,             0,             0,             0,             0,
                                                                 0,                    0,             0,             0,             0,             var_theta_};
    return covariance_matrix;
}


void GlobalTracker::reset_rtabmap_start_bools(){
    rtabmap_status_t_ = false;
    rtabmap_status_previous_t_ = false;
    rtabmap_started_ = false;
}


bool GlobalTracker::check_if_rtabmap_started(){
    if(rtabmap_started_){
	rtabmap_started_ = false;
        return true;       
    }
    else{
        return false;
    }
}


geometry_msgs::PoseWithCovarianceStamped GlobalTracker::get_local_pose(){
    geometry_msgs::PoseWithCovarianceStamped local_pose;
    tf::Transform local_tf = get_local_tf();

    local_pose.pose.pose.position.x = local_tf.getOrigin().x();
    local_pose.pose.pose.position.y = local_tf.getOrigin().y();
    local_pose.pose.pose.position.z = local_tf.getOrigin().z();
    local_pose.pose.pose.orientation.x = local_tf.getRotation().x();
    local_pose.pose.pose.orientation.y = local_tf.getRotation().y();
    local_pose.pose.pose.orientation.z = local_tf.getRotation().z();
    local_pose.pose.pose.orientation.w = local_tf.getRotation().w();

    //fill the covariance matrix
    std::array<double, 36> covariance_matrix = GlobalTracker::get_covariance_matrix_diag();
    for(int i = 0; i < 36; i++){
        local_pose.pose.covariance.at(i) = covariance_matrix[i];
    }

    GlobalTracker::print_pose(local_pose.pose);

    local_pose.header.stamp = ros::Time::now();

    return local_pose;
}
