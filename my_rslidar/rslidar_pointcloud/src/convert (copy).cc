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
  
  tranz_90.translation() << 0,0,0;
  tranz_90.rotate(Eigen::AngleAxisf(M_PI/2,Eigen::Vector3f::UnitZ()));
  tranz_180.translation() << 0,0,0;
  tranz_180.rotate(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitZ()));
  tranz_270.translation() << 0,0,0;
  tranz_270.rotate(Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f::UnitZ()));
  
  // subscribe to rslidarScan packets
  rslidar_scan_.push_back(node.subscribe("dev1", 10, &Convert::processScan1,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev2", 10, &Convert::processScan2,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  //rslidar_scan_.push_back(node.subscribe("dev3", 10, &Convert::processScan3,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
}



void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
}


void Convert::combined_pubber(){
  pcl::PointCloud<pcl::PointXYZI> temp_cloud = *outPoints;
  pcl::PointCloud<pcl::PointXYZI> transed;
  switch(position){
  case 1:
    if(!flag1 && !flag2 && !flag3){
      combined_cloud = temp_cloud;flag1=1;
        std::cout << combined_cloud.points.size() << std::endl;      
      std::cout << "1" << std::endl;
    }
    break;
  case 2:
    if(flag1 && !flag2 && !flag3){
      pcl::transformPointCloud(temp_cloud, transed, tranz1);
      combined_cloud += transed;
        std::cout << combined_cloud.points.size() << std::endl;      
      flag2=1;
      std::cout << "2" << std::endl;      
    }
    break;
  case 3:
    if(flag1 && flag2 && !flag3){
      pcl::transformPointCloud(temp_cloud, transed, tranz2);
      combined_cloud += transed;
        std::cout << combined_cloud.points.size() << std::endl;      
      flag3=1;
      std::cout << "3" << std::endl;      
    }
    break;
  }


  if(position == 2 && flag1 && flag2){
    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    combined_cloud_ptr = combined_cloud.makeShared();
    if(0){
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(combined_cloud_ptr);
      vg.setLeafSize(0.01, 0.01, 0.01);
      vg.filter(*cloud_filtered);
      pcl::copyPointCloud(*cloud_filtered, *combined_cloud_ptr);
    }
    std::cout << "out put with : " << position << std::endl;    
    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*combined_cloud_ptr, outMsg);
    output_.publish(outMsg);
    flag1=0;flag2=0;flag3=0;
  }
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
  comb(3, outPoints);
}


} // namespace rslidar_pointcloud
