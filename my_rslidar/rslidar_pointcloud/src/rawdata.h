/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *	@author Tony Zhang
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>
//#include "filter4s4.h"

namespace rslidar_rawdata
{
static const int RAW_SCAN_SIZE = 8;  //每个通道的每个点的数据字节大小

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees 旋转角分辨率*/

static const float DISTANCE_MAX = 200.0f;       /**< meters */
static const float DISTANCE_MIN = 0.5f;         /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);

/** Special Defines for MEMS support **/
static const int MEMS_BLOCKS_PER_PACKET = 25;
static const int MEMS_SCANS_PER_FIRING = 6;
static const int MEMS_FIRINGS_PER_BLOCK = 1;
static const int MEMS_BLOCK_DATA_SIZE = 32;
static const int MEMS_FIRINGS_PER_FRAME = 19000;  // Equal to 100 beams

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
// union two_bytes {
//  uint16_t uint;
//  uint8_t bytes[2];
//};

static const int PACKET_SIZE = 1400;
static const int PACKET_STATUS_SIZE = 80;

/** \brief Raw Rsldar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */

// MEMS channel return
typedef struct raw_mems_channel
{
  uint16_t intensity_1;
  uint16_t distance_1;
  uint16_t intensity_2;
  uint16_t distance_2;
} raw_mems_channel_t;

// MEMS block
typedef struct raw_mems_block
{
  uint16_t pitch;
  uint16_t yaw;
  raw_mems_channel_t channel[MEMS_SCANS_PER_FIRING];
} raw_mems_block_t;

// MEMS timestamp
typedef struct raw_mems_timestamp
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
}raw_mems_timestamp_t;

// MEMS packet
typedef struct raw_mems_packet
{
  uint8_t sync[4];
  uint8_t cmd[4];
  uint8_t reserved[2];
  raw_mems_timestamp_t timestamp;
  raw_mems_block_t blocks[MEMS_BLOCKS_PER_PACKET];
  uint8_t status[PACKET_STATUS_SIZE];
} raw_mems_packet_t;


/** \brief RSLIDAR data conversion class */
class RawData
{
public:
  RawData();

  ~RawData()
  {
    this->cos_lookup_table_.clear();
    this->sin_lookup_table_.clear();
  }

  /*load the cablibrated files: angle, distance, intensity*/
  void loadConfigFile(ros::NodeHandle private_nh);

  /*unpack the MEMS UDP packet and opuput PCL PointXYZI type*/
  void unpack_MEMS(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                   bool finish_packets_parse);

  /*convert the deg */
  float pitchConvertDeg(int deg);
  float yawConvertDeg(int deg);
  
  /*calibrated the disctance for mems */
  float pixelToDistance_mems(int distance, int dsr);  // convert deg into 0 to 36000

private:
  int g_ChannelNum[MEMS_SCANS_PER_FIRING];
  int realSlow[27780];
  float g_pitchRate;
  float g_yawRate;
  float g_pitchOffset[MEMS_SCANS_PER_FIRING];
  float g_yawOffset[MEMS_SCANS_PER_FIRING];
  float g_yawLimitStaAngle[MEMS_SCANS_PER_FIRING];
  float g_yawLimitEndAngle[MEMS_SCANS_PER_FIRING];
  float distance_max_thd;
  float distance_min_thd;

  uint32_t point_idx;
  /* cos/sin lookup table */
  std::vector<float> cos_lookup_table_;
  std::vector<float> sin_lookup_table_;
};
}  // namespace rslidar_rawdata

#endif  // __RAWDATA_H
