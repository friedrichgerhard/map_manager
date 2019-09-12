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
 * @file   scanutils.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Utility functions for processing laser scan data.
 */

#ifndef SCANUTILS_H
#define SCANUTILS_H

#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include <vector>

/**
 * @brief The ScanUtils class: Utilities for processing laser scan data.
 */
class ScanUtils
{
public:

    /**
     * Method for low pass averaging laser scan data with given filter size.
     * @brief ScanUtils::lowPassAveraging
     * @param input_scan (sensor_msgs::LaserScan)
     * @param filter_size (int)
     * @return filtered_scan (sensor_msgs::LaserScan)
     */
    sensor_msgs::LaserScan lowPassAveraging(sensor_msgs::LaserScan input_scan, int filter_size);

    /**
     * Transforms laser scan data to map frame of local map and converts it into a PointCloud.
     * @brief ScanUtils::transform_scan_to_map_frame
     * @param input_scan (sensor_msgs::LaserScan)
     * @return scan_cloud (sensor_msgs::PointCloud)
     */
    sensor_msgs::PointCloud transform_scan_to_map_frame(sensor_msgs::LaserScan input_scan);

    /**
     * Transforms laser scan data to base_link frame of robot and converts it into a PointCloud.
     * @brief ScanUtils::transform_scan_to_base_link_frame
     * @param input_scan (sensor_msgs::LaserScan)
     * @return scan_cloud (sensor_msgs::PointCloud)
     */
    sensor_msgs::PointCloud transform_scan_to_base_link_frame(sensor_msgs::LaserScan input_scan);

    /**
     * Computes maximum x value from PointCloud.
     * @brief ScanUtils::get_x_max
     * @param cloud (sensor_msgs::PointCloud)
     * @return x_max (float)
     */
    float get_x_max(sensor_msgs::PointCloud cloud);

    /**
     * Computes maximum y value from PointCloud.
     * @brief ScanUtils::get_y_max
     * @param cloud (sensor_msgs::PointCloud)
     * @return y_max (float)
     */
    float get_y_max(sensor_msgs::PointCloud cloud);

    /**
     * Computes minimum x value from PointCloud.
     * @brief ScanUtils::get_x_min
     * @param cloud (sensor_msgs::PointCloud)
     * @return x_min (float)
     */
    float get_x_min(sensor_msgs::PointCloud cloud);

    /**
     * Computes minimum y value from PointCloud.
     * @brief ScanUtils::get_y_min
     * @param cloud (sensor_msgs::PointCloud)
     * @return y_min (float)
     */
    float get_y_min(sensor_msgs::PointCloud cloud);
};

#endif // SCANUTILS_H
