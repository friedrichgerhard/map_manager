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
 * @file   register_velodyne.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  ROS-node for computing a depth image from the velodyne point cloud.
 * Therefore, the pointcloud is projected in the camera frame and then a dilation and vertical interpolation is executed for generating continous depth data.
 */

#include "register_velodyne/register_velodyne.h"

//########## CONSTRUCTOR #####################################################################################

RegisterVelodyne::RegisterVelodyne(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
    // === PARAMETERS FROM LAUNCHFILE ===
    ros::NodeHandle nh("~");
    std::string param;
    nh.getParam("scan_topic", scan_topic_);
    nh.getParam("cam_info_topic", cam_info_topic_);
    nh.getParam("depth_image_topic", depth_image_topic_);
    nh.getParam("camera_frame", camera_frame_);
    nh.getParam("horizontal_image", horizontal_image_);

    ROS_WARN("Parameters:");
    std::cout << "scan_topic: " << scan_topic_ << "\n";
    std::cout << "cam_info_topic: " << cam_info_topic_ << "\n";
    std::cout << "depth_image_topic: " << depth_image_topic_ << "\n";
    std::cout << "camera_frame: " << camera_frame_ << "\n";
    std::cout << "horizontal_image: " << horizontal_image_ << "\n";

    // === SUBSCRIBERS ===
    pointcloud_subscriber_ = node_->subscribe(scan_topic_, 10, &RegisterVelodyne::subscriberCallback, this);
    camera_info_subscriber_ = node_->subscribe(cam_info_topic_, 10, &RegisterVelodyne::camInfoSubscriberCallback, this); //TODO: check topic name

    // === PUBLISHERS ==
    depth_img_publisher_ = node_->advertise<sensor_msgs::Image>(depth_image_topic_, 10);

    RegisterVelodyne::get_camera_transform();
    RegisterVelodyne::get_velodyne_transform();

    received_camera_info_ = false;
}

//########## CALLBACK: CAM INFO SUBSCRIBER CALLBACK ############################################################################
void RegisterVelodyne::camInfoSubscriberCallback(const sensor_msgs::CameraInfo &msg){
    //only fill in the information once at the beginning
    if(!received_camera_info_){
        f_x_ = msg.K.at(0);
        f_y_ = msg.K.at(4);
        c_x_ = msg.K.at(2);
        c_y_ = msg.K.at(5);

        imageSize_ = cv::Size(msg.width, msg.height);

        received_camera_info_ = true;
        ROS_INFO("Received camera info!");
    }
}

//########## CALLBACK: SUBSCRIBER ############################################################################
void RegisterVelodyne::subscriberCallback(const sensor_msgs::PointCloud2 &msg)
{

    //only compute depth image when camera info is available
    if(received_camera_info_){

        //get pcl pointcloud from message
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (msg, *laserScan);

        // Executing the transformation
        const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*laserScan, *laserScan_ptr, translation_, q_.cast<float>());

        cv::Mat  registered = cv::Mat::zeros(imageSize_, CV_32FC1);

        //############################# Inserted from rtabmap::util3d::projectCloudToCamera ########################
        rtabmap::Transform t = cameraTransform_.inverse();

        int count = 0;
        for(int i=0; i<(int)laserScan -> size(); ++i)
        {
                // Get 3D from laser scan
                pcl::PointXYZ ptScan = laserScan -> at(i);
                ptScan = rtabmap::util3d::transformPoint(ptScan, t);

                // re-project in camera frame
                float z = ptScan.z;
                float invZ = 1.0f/z;
                int dx = (f_x_*ptScan.x)*invZ + c_x_;
                int dy = (f_y_*ptScan.y)*invZ + c_y_;

                if(z > 0.0f && dx > 10 && dx <= registered.cols-10 && dy > 10 && dy <= registered.rows-10)
                {
                        ++count;
                        float &zReg = registered.at<float>(dy, dx);
                        if(zReg == 0 || z < zReg)
                        {
                                zReg = z;
                        }
                }
        }


        //############################### END INSERT ################################################

        //apply dilation
        registered = RegisterVelodyne::dilate_scanpoints(registered, 3);

        if(horizontal_image_){ 
               //vertical filling
               rtabmap::util3d::fillProjectedCloudHoles(registered, true, false);
        }
        else{
               //horizontal filling
               rtabmap::util3d::fillProjectedCloudHoles(registered, false, false);
        }

        std_msgs::Header scan_header = msg.header;
        std_msgs::Header depth_header;
        depth_header.seq = scan_header.seq;
        depth_header.stamp = scan_header.stamp;
        depth_header.frame_id = camera_frame_;

        cv_bridge::CvImage out_msg;
        out_msg.header   = depth_header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_msg.image    = registered; // Your cv::Mat

	sensor_msgs::ImagePtr img_to_pub = out_msg.toImageMsg();

        depth_img_publisher_.publish(img_to_pub);
    }
}

//########## DILATION SCANPOINTS ############################################################################
cv::Mat RegisterVelodyne::dilate_scanpoints(cv::Mat registered, int dilation_size){
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         cv::Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    cv::dilate( registered, registered, element);
    return registered;
}

//########## GET VELODYNE TRANSFORM ############################################################################
void RegisterVelodyne::get_velodyne_transform()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "velodyne";
    std::string sourceFrameID = "base_link";

    tf::StampedTransform transformStamped;


    //bool got_trans =false;
    while (!listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(3.0)))
	{
        ROS_WARN("Hallo: No transfomation received yet!");
    }

	try{
	    listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
	    ROS_INFO("Got Transform!");

	}
	catch (tf::TransformException ex){
	    ROS_ERROR("Hallo %s",ex.what());
	}

    tf::Transform tf;
    tf.setRotation(transformStamped.getRotation());
    tf.setOrigin(transformStamped.getOrigin());
    tf::Vector3 origin;
    tf::Quaternion rotation;
    origin = tf.getOrigin();
    rotation = tf.getRotation();

    translation_(0,0) = origin.getX();
    translation_(1,0) = origin.getY();
    translation_(2,0) = origin.getZ();

    q_.x() = rotation.getX();
    q_.y() = rotation.getY();
    q_.z() = rotation.getZ();
    q_.w() = rotation.getW();

    std::cout << "\nTransform from " << sourceFrameID << " to " << targetFrameID << ":\n";
    std::cout << "Origin:\n"
              << "x = " << origin.getX() << "\n"
              << "y = " << origin.getY() << "\n"
              << "z = " << origin.getZ() << "\n\n";

    std::cout << "Rotation:\n"
              << "x = " << rotation.getX() << "\n"
              << "y = " << rotation.getY() << "\n"
              << "z = " << rotation.getZ() << "\n"
              << "w = " << rotation.getW() << "\n";
}

//########## GET CAMERA TRANSFORM ############################################################################
void RegisterVelodyne::get_camera_transform()
{
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = camera_frame_;
    std::string sourceFrameID = "velodyne";
    tf::StampedTransform transformStamped;

    while (!listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(3.0)))
	{
        ROS_WARN("Hallo: No transfomation received yet!");
    }

	try{
	    listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
	    ROS_INFO("Got Transform!");

	}
	catch (tf::TransformException ex){
	    ROS_ERROR("Hallo %s",ex.what());
	}

    tf::Transform tf;
    tf.setRotation(transformStamped.getRotation());
    tf.setOrigin(transformStamped.getOrigin());
    tf::Vector3 origin;
    tf::Quaternion rotation;
    origin = tf.getOrigin();
    rotation = tf.getRotation();

    const rtabmap::Transform cameraTransform_inverse(origin.getX(), origin.getY(), origin.getZ(),
                             rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW());
    cameraTransform_ = cameraTransform_inverse.inverse();

    std::cout << "\nTransform from " << sourceFrameID << " to " << targetFrameID << ":\n";
    std::cout << "Origin:\n"
              << "x = " << origin.getX() << "\n"
              << "y = " << origin.getY() << "\n"
              << "z = " << origin.getZ() << "\n\n";

    std::cout << "Rotation:\n"
              << "x = " << rotation.getX() << "\n"
              << "y = " << rotation.getY() << "\n"
              << "z = " << rotation.getZ() << "\n"
              << "w = " << rotation.getW() << "\n";
}

//########## MAIN ############################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "register_velodyne");

  ros::NodeHandle node_handle;
  RegisterVelodyne register_velodyne(node_handle);

  ROS_INFO("Node is spinning...");
  ros::spin();

  return 0;
}
