/*
 *  Copyright (C) 2018-2020 Robosense Authors
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#include "convert_bp.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));

  output_ = node.advertise<sensor_msgs::PointCloud2>("combined_bp", 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);


  ///trans
  std::string param_name = "/lidar0";
  for(int idx=4;idx < 7;idx++){
    Eigen::Affine3f tranz__ = Eigen::Affine3f::Identity();
    param_name[6] = (char)(idx+0x30);
    std::vector<double> tran;
    private_nh.getParam(param_name, tran);
    //for(int ind = 0;ind < tran.size();ind++)
      //std::cout << tran[ind] << std::endl;
    tranz__.translation() << tran[0],tran[1],tran[2];
    tranz__.rotate(Eigen::AngleAxisf(tran[3],Eigen::Vector3f::UnitZ()));
    tranz__.rotate(Eigen::AngleAxisf(tran[4],Eigen::Vector3f::UnitY()));
    tranz__.rotate(Eigen::AngleAxisf(tran[5],Eigen::Vector3f::UnitX()));
    tranz_.push_back(tranz__);
  }
  
  
  
/*
  tranz_4.translation() << 0,0,1;
  tranz_4.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitZ()));
  tranz_5.translation() << 0,0,1;
  tranz_5.rotate(Eigen::AngleAxisf(M_PI/2,Eigen::Vector3f::UnitZ()));
  tranz_6.translation() << 0,0,1;
  tranz_6.rotate(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitZ()));
  */
  
  // subscribe to rslidarScan packets
  rslidar_scan_.push_back(node.subscribe("dev4", 1, &Convert::processScan4, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev5", 1, &Convert::processScan5, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev6", 1, &Convert::processScan6, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true)));
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
//  ROS_INFO("[cloud][convert] Reconfigure Request");
  // config_.time_offset = config.time_offset;
}




void Convert::combined_pubber(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev5_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev6_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev4_points, *dev4_, tranz_[0]);
  pcl::transformPointCloud(*dev5_points, *dev5_, tranz_[1]);
  pcl::transformPointCloud(*dev6_points, *dev6_, tranz_[2]);
  pcl::PointCloud<pcl::PointXYZI> _added = *dev4_ + *dev5_ + *dev6_;
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(_added, outMsg);
  output_.publish(outMsg);
}




/** @brief Callback for raw scan messages. */
void Convert::processScan4(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  outPoints->height = 32;
  outPoints->width = 12 * (int)scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    data_->unpack(scanMsg->packets[i], outPoints);
  dev4_points = outPoints;
}

void Convert::processScan5(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  outPoints->height = 32;
  outPoints->width = 12 * (int)scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    data_->unpack(scanMsg->packets[i], outPoints);
  dev5_points = outPoints;
}


void Convert::processScan6(const rslidar_msgs::rslidarScan_bp::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  outPoints->height = 32;
  outPoints->width = 12 * (int)scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    data_->unpack(scanMsg->packets[i], outPoints);
  dev6_points = outPoints;
  combined_pubber();
}


}  // namespace rslidar_pointcloud
