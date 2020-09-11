#include "convert.h"
#include <time.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud {

Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
    : data_(new rslidar_rawdata::RawData()) {
  data_->loadConfigFile(private_nh); // load lidar parameters

  std::string model;
  private_nh.param("model", model, std::string("MEMS"));

  // advertise output point cloud (before subscribing to input data)
  output_ = node.advertise<sensor_msgs::PointCloud2>("combined", 10);

  srv_ = boost::make_shared<
      dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>>(
      private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType
      f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);
  
  tranz_1.translation() << 0,0,1;
  tranz_1.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitZ()));
  tranz_2.translation() << 0,0,1;
  tranz_2.rotate(Eigen::AngleAxisf(M_PI/2,Eigen::Vector3f::UnitZ()));
  tranz_3.translation() << 0,0,1;
  tranz_3.rotate(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitZ()));
  tranz_4.translation() << 0,0,1;
  tranz_4.rotate(Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f::UnitZ()));
  
  // 	subscribe to rslidarScan packets
  rslidar_scan_.push_back(node.subscribe("dev1", 1, &Convert::processScan1,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev2", 1, &Convert::processScan2,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev3", 1, &Convert::processScan3,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev4", 1, &Convert::processScan4,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
}



void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
}


void Convert::combined_pubber(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev1_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev2_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev3_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev1_points, *dev1_, tranz_1);
  pcl::transformPointCloud(*dev2_points, *dev2_, tranz_2);
  pcl::transformPointCloud(*dev3_points, *dev3_, tranz_3);
  pcl::transformPointCloud(*dev4_points, *dev4_, tranz_4);
  pcl::PointCloud<pcl::PointXYZI> _added = *dev1_ + *dev2_ + *dev3_ + *dev4_;
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(_added, outMsg);
  output_.publish(outMsg);
}


void Convert::processScan1(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->height = 6;
  outPoints->width = 25*scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  bool finish_packets_parse = false;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    if (i == (scanMsg->packets.size() - 1)) 
      finish_packets_parse = true;
    data_->unpack_MEMS(scanMsg->packets[i], outPoints, finish_packets_parse);
  }
  dev1_points = outPoints;
}


void Convert::processScan2(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->height = 6;
  outPoints->width = 25*scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  bool finish_packets_parse = false;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    if (i == (scanMsg->packets.size() - 1)) 
      finish_packets_parse = true;
    data_->unpack_MEMS(scanMsg->packets[i], outPoints, finish_packets_parse);
  }
  dev2_points = outPoints;
}


void Convert::processScan3(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->height = 6;
  outPoints->width = 25*scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  bool finish_packets_parse = false;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    if (i == (scanMsg->packets.size() - 1)) 
      finish_packets_parse = true;
    data_->unpack_MEMS(scanMsg->packets[i], outPoints, finish_packets_parse);
  }
  dev3_points = outPoints;
}


void Convert::processScan4(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->height = 6;
  outPoints->width = 25*scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  bool finish_packets_parse = false;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    if (i == (scanMsg->packets.size() - 1)) 
      finish_packets_parse = true;
    data_->unpack_MEMS(scanMsg->packets[i], outPoints, finish_packets_parse);
  }
  dev4_points = outPoints;
  clock_t pub_begin = clock();
  combined_pubber();
  //std::cout << (double)(clock() - pub_begin)/CLOCKS_PER_SEC * 1000 << std::endl;
}
} // namespace rslidar_pointcloud
