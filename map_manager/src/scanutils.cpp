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
 * @file   scanutils.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Utility functions for processing laser scan data.
 */

#include "scanutils/scanutils.h"

//############ LOW-PASS AVERAGE FILTER FOR LASER SCAN DATA #########################################
sensor_msgs::LaserScan ScanUtils::lowPassAveraging(sensor_msgs::LaserScan input_scan, int filter_size){

    std::vector<double> input_vector;
    for (int i = 0; i < input_scan.ranges.size(); i++) {
        input_vector.push_back(input_scan.ranges.at(i));
    }

    //zero padding on both ends of the vector
    int input_size = input_vector.size();
    std::vector<double> zeros;
    zeros.assign(filter_size, 0); //assign entries with value 0.0

    std::vector<double> zero_padded_scan;
    zero_padded_scan.insert(zero_padded_scan.end(), zeros.begin(), zeros.end());
    zero_padded_scan.insert(zero_padded_scan.end(), input_vector.begin(), input_vector.end());
    zero_padded_scan.insert(zero_padded_scan.end(), zeros.begin(), zeros.end());

    std::vector<double> filtered_ranges;

    for(int i=filter_size; i < (filter_size+input_size); i++){ //iterate through original input_vector entries
        int cnt_entires = 0; //to avoid dividing through the number of zeros
        double sum = 0;
        for (int k = i-filter_size; k <= i+filter_size; k++) { //iterate through the neigboring entries vor averaging
            if(zero_padded_scan.at(k) > 0 && std::isfinite(zero_padded_scan.at(k))){
                sum = sum + zero_padded_scan.at(k);
                cnt_entires++;
            }
        }
        double average = sum/cnt_entires;
        filtered_ranges.push_back(average);
    }

    sensor_msgs::LaserScan filtered_scan;
    filtered_scan.header = input_scan.header;
    filtered_scan.angle_max = input_scan.angle_max;
    filtered_scan.angle_min = input_scan.angle_min;
    filtered_scan.angle_increment = input_scan.angle_increment;

    for(int i = 0; i < filtered_ranges.size(); i++){
        filtered_scan.ranges.push_back(filtered_ranges.at(i));
    }

    return filtered_scan;
}

//############ TRANSFORM SCAN TO MAP FRAME #########################################
sensor_msgs::PointCloud ScanUtils::transform_scan_to_map_frame(sensor_msgs::LaserScan input_scan){
    tf::TransformListener listener;
    listener.waitForTransform("/velodyne", "/map", ros::Time(0), ros::Duration(1.0));

    sensor_msgs::PointCloud scan_cloud;
    scan_cloud.header.stamp = ros::Time::now();
    scan_cloud.header.frame_id = "map";

    for (int i = 0; i < input_scan.ranges.size();i++)
    {
        double range = input_scan.ranges[i];
        double angle  = input_scan.angle_min +(i * input_scan.angle_increment);

        geometry_msgs::PointStamped laser_point;

        laser_point.header.frame_id = "velodyne";
        laser_point.header.stamp = ros::Time();
        laser_point.point.x = range*cos(angle) ;
        laser_point.point.y = range*sin(angle) ;
        laser_point.point.z = 0.0;

        geometry_msgs::PointStamped map_point;

        try{
            listener.transformPoint("map", laser_point, map_point);

        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        }

        geometry_msgs::Point32 map_point_32;
        map_point_32.x = map_point.point.x;
        map_point_32.y = map_point.point.y;
        map_point_32.z = map_point.point.z;

        //map point x,y are in odom frame
        scan_cloud.points.push_back(map_point_32);
    }
    return scan_cloud;
}

//############ TRANSFORM SCAN TO BASE LINK FRAME #########################################
sensor_msgs::PointCloud ScanUtils::transform_scan_to_base_link_frame(sensor_msgs::LaserScan input_scan){
    tf::TransformListener listener;
    listener.waitForTransform("/velodyne", "/base_link", ros::Time(0), ros::Duration(1.0));

    sensor_msgs::PointCloud scan_cloud;
    scan_cloud.header.stamp = ros::Time::now();
    scan_cloud.header.frame_id = "base_link";

    for (int i = 0; i < input_scan.ranges.size();i++)
    {
        double range = input_scan.ranges[i];
        double angle  = input_scan.angle_min +(i * input_scan.angle_increment);

        geometry_msgs::PointStamped laser_point;

        laser_point.header.frame_id = "velodyne";
        laser_point.header.stamp = ros::Time();
        laser_point.point.x = range*cos(angle) ;
        laser_point.point.y = range*sin(angle) ;
        laser_point.point.z = 0.0;

        geometry_msgs::PointStamped map_point;

        try{
            listener.transformPoint("base_link", laser_point, map_point);

        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        }

        geometry_msgs::Point32 map_point_32;
        map_point_32.x = map_point.point.x;
        map_point_32.y = map_point.point.y;
        map_point_32.z = map_point.point.z;

        //map point x,y are in odom frame
        scan_cloud.points.push_back(map_point_32);
    }
    return scan_cloud;
}

//############ COMPUTE X_MAX/X_MIN AND Y_MAX/Y_MIN #########################

float ScanUtils::get_x_max(sensor_msgs::PointCloud cloud){
    float x_max = cloud.points.at(0).x;
    for(int i = 0; i < cloud.points.size(); i++){
        if(cloud.points.at(i).x > x_max){
            x_max = cloud.points.at(i).x;
        }
    }
    return x_max;
}


float ScanUtils::get_y_max(sensor_msgs::PointCloud cloud){
    float y_max = cloud.points.at(0).y;
    for(int i = 0; i < cloud.points.size(); i++){
        if(cloud.points.at(i).y > y_max){
            y_max = cloud.points.at(i).y;
        }
    }
    return y_max;
}


float ScanUtils::get_x_min(sensor_msgs::PointCloud cloud){
    float x_min = cloud.points.at(0).x;
    for(int i = 0; i < cloud.points.size(); i++){
        if(cloud.points.at(i).x < x_min){
            x_min = cloud.points.at(i).x;
        }
    }
    return x_min;
}


float ScanUtils::get_y_min(sensor_msgs::PointCloud cloud){
    float y_min = cloud.points.at(0).y;
    for(int i = 0; i < cloud.points.size(); i++){
        if(cloud.points.at(i).y < y_min){
            y_min = cloud.points.at(i).y;
        }
    }
    return y_min;
}
