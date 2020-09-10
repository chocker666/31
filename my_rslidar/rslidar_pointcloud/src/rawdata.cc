/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"

#define RS_Grabber_toRadians(x) ((x)*M_PI / 180.0)
#define RS_SWAP_HIGHLOW(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
// #define RS_SWAP_HIGHLOW(x) (((x) / 256) + (((x) % 256) * 256))

namespace rslidar_rawdata
{
RawData::RawData()
{
  point_idx = 0;
}

void RawData::loadConfigFile(ros::NodeHandle private_nh)
{
  std::string channelPath, limitPath, slowPath;

  private_nh.param("channel_path", channelPath, std::string(""));
  private_nh.param("limit_path", limitPath, std::string(""));
  private_nh.param("slow_path", slowPath, std::string(""));

  //=============================================================
  FILE* f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel)
  {
    ROS_ERROR_STREAM(channelPath << " channel path does not exist");
  }
  else
  {
    ROS_INFO_STREAM("Loading channelNum corrections file!");

    int loopm = 0;
    float tmpBuf[32];

    while (!feof(f_channel))
    {
      fscanf(f_channel, "%f%*[^\n]%*c\n", &tmpBuf[loopm]);
      loopm++;
      if (loopm >= 32)
      {
        break;
      }
    }

    for (int i = 0; i < MEMS_SCANS_PER_FIRING; i++)
    {
      g_ChannelNum[i] = (int)(tmpBuf[i]);
      g_pitchOffset[i] = tmpBuf[8 + i];
      g_yawOffset[i] = tmpBuf[14 + i];
      g_yawLimitStaAngle[i] = tmpBuf[20 + 2 * i];
      g_yawLimitEndAngle[i] = tmpBuf[21 + 2 * i];
    }
    g_pitchRate = tmpBuf[6];
    g_yawRate = tmpBuf[7];

    fclose(f_channel);
  }

  //=============================================================
  FILE* f_limit = fopen(limitPath.c_str(), "r");
  if (!f_limit)
  {
    ROS_ERROR_STREAM(f_limit << " limit path does not exist");
    distance_max_thd = 200.0;
    distance_min_thd = 0.0;
  }
  else
  {
    ROS_INFO_STREAM("Loading limit file!");
    float tmp_min;
    float tmp_max;

    fscanf(f_limit, "%f\n", &tmp_min);
    fscanf(f_limit, "%f\n", &tmp_max);

    fclose(f_limit);

    distance_max_thd = tmp_max / 100.0;
    distance_min_thd = tmp_min / 100.0;
  }

  //=============================================================
  FILE* f_slow = fopen(slowPath.c_str(), "r");
  if (!f_slow)
  {
    ROS_ERROR_STREAM(slowPath << " slow path does not exist");
  }
  else
  {
    ROS_INFO_STREAM("Loading slow corrections file!");
    int i, j;
    int t[20];
    for (i = 0; i < 1389; i++)
    {
      fscanf(f_slow, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n", &t[0], &t[1], &t[2], &t[3],
             &t[4], &t[5], &t[6], &t[7], &t[8], &t[9], &t[10], &t[11], &t[12], &t[13], &t[14], &t[15], &t[16], &t[17],
             &t[18], &t[19]);
      for (j = 0; j < 20; j++)
      {
        realSlow[i * 20 + j] = t[j];
      }
    }

    fclose(f_slow);
  }

  // lookup table init, -10 ~ 10 deg, 0.01 resolution
  this->cos_lookup_table_.resize(2000);
  this->sin_lookup_table_.resize(2000);
  for (int i = -1000; i < 1000; i++)
  {
    double rad = RS_Grabber_toRadians(i / 100.0f);

    this->cos_lookup_table_[i + 1000] = std::cos(rad);
    this->sin_lookup_table_[i + 1000] = std::sin(rad);
  }
}

//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */

float RawData::pitchConvertDeg(int deg)  // convert deg into  0 to 36000
{
  float result_f;
  float deg_f = deg;

  result_f = (deg_f * (1250.0f / 65534.0f) - 625.0f);
  // printf("%d,%f;%f\n",deg,deg_f,result_f);
  return result_f;
}

float RawData::yawConvertDeg(int deg)  // convert deg into  0 to 36000
{
  float result_f;
  float deg_f = deg;

  result_f = (deg_f * (1375.0f / 65534.0f) - 687.5f);
  // printf("%d,%f;%f\n",deg,deg_f,result_f);
  return result_f;
}

float RawData::pixelToDistance_mems(int distance, int dsr)
{
  float result;
  int cor = g_ChannelNum[dsr];

  if (distance <= cor)
  {
    result = 0.0;
  }
  else
  {
    result = distance - cor;
  }
  return result;
}

void RawData::unpack_MEMS(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                          bool finish_packets_parse)
{
  float pitch, yaw;  // 0.01 dgree
  int pitch_temp, index_temp;

  const raw_mems_packet_t* raw = (const raw_mems_packet_t*)&pkt.data[0];

  for (int block = 0; block < MEMS_BLOCKS_PER_PACKET; block++)  // 1 packet:25 data blocks
  {
    raw_mems_block_t* pBlockPkt = (raw_mems_block_t*)(raw->blocks + block);

    index_temp = RS_SWAP_HIGHLOW(pBlockPkt->pitch);

    // pitch angle mapping
    pitch_temp = (int)realSlow[index_temp];
    if (pitch_temp < -32767)
    {
      pitch_temp = -32767;
    }
    else if (pitch_temp > 32767)
    {
      pitch_temp = 32767;
    }

    pitch = pitchConvertDeg(pitch_temp + 32767) * ROTATION_RESOLUTION;
    yaw = yawConvertDeg(RS_SWAP_HIGHLOW(pBlockPkt->yaw)) * ROTATION_RESOLUTION;

    // unpack
    for (int dsr = 0; dsr < MEMS_SCANS_PER_FIRING; dsr++)  // 6 channels
    {
      float distance = pixelToDistance_mems(RS_SWAP_HIGHLOW(pBlockPkt->channel[dsr].distance_1), dsr);
      distance = distance * DISTANCE_RESOLUTION;

      float temp_a, temp_b, temp_c, sinax, cosax, sinay, cosay;
      int ax, ay;

      ax = -1 * (int)(100 * g_pitchRate * (pitch + g_pitchOffset[dsr]));  // ax: -1000 ~ 1000, 0.01deg
      ay = (int)(100 * g_yawRate * (yaw + g_yawOffset[dsr]));             // ay: -1000 ~ 1000, 0.01deg
      sinax = this->sin_lookup_table_[ax + 1000];                         // +1000, for index to 0 ~ 1999;
      cosax = this->cos_lookup_table_[ax + 1000];
      sinay = this->sin_lookup_table_[ay + 1000];
      cosay = this->cos_lookup_table_[ay + 1000];

      switch (dsr)
      {
        case 5:
          temp_a = sinay * (0.9659 * cosax - 0.2588 * sinax) *
                   (0.2588 * cosax + 0.9659 * sinax + 1.732 * cosay * (0.9659 * cosax - 0.2588 * sinax));
          temp_b = 0.5 - 1.0 * (0.2588 * cosax + 0.9659 * sinax) *
                             (0.2588 * cosax + 0.9659 * sinax + 1.732 * cosay * (0.9659 * cosax - 0.2588 * sinax));
          temp_c = cosay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.2588 * cosax + 0.9659 * sinax + 1.732 * cosay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.866;
          break;
        case 1:
          temp_a = sinay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                        1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.749;
          temp_b = 0.3313 - 1.0 * (0.2588 * cosax + 0.9659 * sinax) *
                                (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                                 1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax));
          temp_c = cosay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                        1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.5738;
          break;
        case 2:
          temp_a = sinay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                        0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.4147;
          temp_b = 0.455 - 1.0 * (0.2588 * cosax + 0.9659 * sinax) *
                               (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                                0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax));
          temp_c = cosay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) +
                        0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.788;
          break;
        case 3:
          temp_a = sinay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                        0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax)) +
                   0.4147;
          temp_b = 0.455 - 1.0 * (0.2588 * cosax + 0.9659 * sinax) *
                               (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                                0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax));
          temp_c = cosay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.2355 * cosax + 0.879 * sinax + 1.576 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                        0.8294 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.788;
          break;
        case 4:
          temp_a = sinay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                        1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax)) +
                   0.749;
          temp_b = 0.3313 - 1.0 * (0.2588 * cosax + 0.9659 * sinax) *
                                (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                                 1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax));
          temp_c = cosay * (0.9659 * cosax - 0.2588 * sinax) *
                       (0.1715 * cosax + 0.64 * sinax + 1.148 * cosay * (0.9659 * cosax - 0.2588 * sinax) -
                        1.498 * sinay * (0.9659 * cosax - 0.2588 * sinax)) -
                   0.5738;
          break;
        case 0:
          continue;
          /* temp_a = sinay*(0.9659*cosax - 0.2588*sinax)*(0.2588*cosax + 0.9659*sinax + 1.732*cosay*(0.9659*cosax -
          0.2588*sinax)); temp_b = 0.5 - 1.0*(0.2588*cosax + 0.9659*sinax)*(0.2588*cosax + 0.9659*sinax
          + 1.732*cosay*(0.9659*cosax - 0.2588*sinax)); temp_c = cosay*(0.9659*cosax - 0.2588*sinax)*(0.2588*cosax +
          0.9659*sinax + 1.732*cosay*(0.9659*cosax - 0.2588*sinax)) - 0.866; break;*/
        default:
          break;
      }

      pcl::PointXYZI point;
      if (distance > distance_max_thd || distance < distance_min_thd || yaw < g_yawLimitStaAngle[dsr] ||
          yaw > g_yawLimitEndAngle[dsr])  // invalid data
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
        pointcloud->at(point_idx, dsr) = point;
      }
      else
      {
        point.x = temp_c * distance;
        point.y = -temp_a * distance;
        point.z = temp_b * distance;
        point.intensity = RS_SWAP_HIGHLOW(pBlockPkt->channel[dsr].intensity_1);
        pointcloud->at(point_idx, dsr) = point;
      }
    }
    point_idx++;
  }

  if (finish_packets_parse)
  {
    point_idx = 0;
  }
}
}  // namespace rslidar_rawdata
