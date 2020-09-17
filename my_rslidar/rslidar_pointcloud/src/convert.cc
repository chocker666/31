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
  
  ///trans
  std::string param_name = "/lidar0";
  for(int idx=0;idx < 4;idx++){
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

  // subscribe to rslidarScan packets
  rslidar_scan_.push_back(node.subscribe("combined_bp", 1, &Convert::combined_pubber,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev0", 1, &Convert::processScan1,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev1", 1, &Convert::processScan2,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev2", 1, &Convert::processScan3,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));
  rslidar_scan_.push_back(node.subscribe("dev3", 1, &Convert::processScan4,(Convert *)this, ros::TransportHints().tcpNoDelay(true)));

}



void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
}


void Convert::combined_pubber(const sensor_msgs::PointCloud2 &bp_msgs){
  pcl::PointCloud<pcl::PointXYZI> _added;
  pcl::fromROSMsg(bp_msgs, _added);
    
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev1_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev2_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev3_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dev4_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*dev1_points, *dev1_, tranz_[0]);
  pcl::transformPointCloud(*dev2_points, *dev2_, tranz_[1]);
  pcl::transformPointCloud(*dev3_points, *dev3_, tranz_[2]);
  pcl::transformPointCloud(*dev4_points, *dev4_, tranz_[3]);
  _added = _added + *dev1_ + *dev2_ + *dev3_ + *dev4_;
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
}
} // namespace rslidar_pointcloud
