/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rawdata.h"

#include <pcl/filters/voxel_grid.h> 

#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <boost/thread.hpp>


#include <geometry_msgs/QuaternionStamped.h>
#include <pcl/registration/icp.h>

pcl::PointCloud<pcl::PointXYZI> combined_cloud;
std::vector<Eigen::Affine3f> tranz_;

pcl::PointCloud<pcl::PointXYZI>::Ptr dev1_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dev2_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dev3_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_points(new pcl::PointCloud<pcl::PointXYZI>);
namespace rslidar_pointcloud {
    class Convert {
    public:

        Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

    private:

        void callback(rslidar_pointcloud::CloudNodeConfig &config,
                      uint32_t level);

        void processScan1(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg);
        void processScan2(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg);
        void processScan3(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg);
        void processScan4(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg);
        void combined_pubber(const sensor_msgs::PointCloud2 &origin);
        ///Pointer to dynamic reconfigure service srv_
        boost::shared_ptr<dynamic_reconfigure::Server<rslidar_pointcloud::
        CloudNodeConfig> > srv_;

        boost::shared_ptr<rslidar_rawdata::RawData> data_;
        std::vector<ros::Subscriber> rslidar_scan_;
        ros::Publisher output_;

    };

}//namespace rslidar_pointcloud
#endif
