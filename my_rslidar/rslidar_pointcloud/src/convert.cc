#include "convert.h"
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
  
  // subscribe to rslidarScan packets
  rslidar_scan_.push_back(node.subscribe("dev0", 1, &Convert::processScan4,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev1", 1, &Convert::processScan1,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev2", 1, &Convert::processScan2,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev3", 1, &Convert::processScan3,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));

}



void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
}


void Convert::combined_pubber(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev1_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*dev1_points, *dev1_origin);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev1_transed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev1_origin, *dev1_transed, tranz_1);
  pcl::PointCloud<pcl::PointXYZI> dev1_ = *dev1_transed;
  pcl::PointCloud<pcl::PointXYZI> temp_dev1_added = dev1_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev2_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*dev2_points, *dev2_origin);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev2_transed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev2_origin, *dev2_transed, tranz_2);
  pcl::PointCloud<pcl::PointXYZI> dev2_ = *dev2_transed;
  pcl::PointCloud<pcl::PointXYZI> temp_dev2_added = temp_dev1_added + dev2_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev3_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*dev3_points, *dev3_origin);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev3_transed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev3_origin, *dev3_transed, tranz_3);
  pcl::PointCloud<pcl::PointXYZI> dev3_ = *dev3_transed;
  pcl::PointCloud<pcl::PointXYZI> temp_dev3_added = temp_dev2_added + dev3_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*dev4_points, *dev4_origin);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_transed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev4_origin, *dev4_transed, tranz_4);
  pcl::PointCloud<pcl::PointXYZI> dev4_ = *dev4_transed;
  pcl::PointCloud<pcl::PointXYZI> temp_dev4_added = temp_dev3_added + dev4_;
/*
  pcl::PointCloud<pcl::PointXYZI>::Ptr combined(new pcl::PointCloud<pcl::PointXYZI>);
  if(1){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    combined = temp_dev4_added.makeShared();
    vg.setInputCloud(combined);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(*cloud_filtered);
    pcl::copyPointCloud(*cloud_filtered, *combined);
    
  }
  */
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(temp_dev4_added, outMsg);
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
  combined_pubber();
}
} // namespace rslidar_pointcloud
