/*
 *  Copyright (C) 2018-2020 Robosense Authors
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rawdata_bp.h"


pcl::PointCloud<pcl::PointXYZI> combined_cloud_bp;
std::vector<Eigen::Affine3f> tranz_;

pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dev5_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dev6_points(new pcl::PointCloud<pcl::PointXYZI>);
namespace rslidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:
  void callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level);

  void processScan4(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg);
  void processScan5(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg);
  void processScan6(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg);
  void combined_pubber();
  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<rslidar_rawdata::RawData> data_;
  std::vector<ros::Subscriber> rslidar_scan_;
  ros::Publisher output_;
};

}  // namespace rslidar_pointcloud
#endif
