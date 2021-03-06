#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/radius_outlier_removal.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_space.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"    




//vx0.044-nr0.1
#define VX0 0.044f
#define NR 0.1

#define DIR "/home/u/Desktop/"
#define log_DIR "/home/u/log.txt"


#define IP "10.9.69.165"
#define sPORT 1130
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define BUFFER_SIZE IMG_WIDTH*IMG_HEIGHT*3/32  


using namespace std;
class stair_obj{
public:
  stair_obj(int W,int L, int D, int N, Eigen::Vector3f FLC);
  void show(void);

private:
  double width;
  double length;
  double depth;
  int num;
  Eigen::Vector3f firstLC;
};


stair_obj::stair_obj(int W,int L, int D, int N, Eigen::Vector3f FLC){
  width = (double)W/100;
  length = (double)L/100;
  depth = (double)D/100;
  num = N;
  firstLC = FLC;
  ROS_INFO("construct with:%lf,%lf,%lf,%d",width,length,depth,num);
}




int index_pic=120;
int W,L,D,N;
Eigen::Vector3f FLC;
Eigen::Vector3f W_pc, L_pc,D_pc;
std::vector<pcl::PointCloud<pcl::PointXYZ> > slices(240);
std::vector<std::vector<Eigen::Vector2f> > drawPs(240);
int len_compn = 0, dep_compn = 0, wid_compn = 0;
char STEP = 'K';//STEP A:dilution; B:boundary; C:macro seg; D:remove wall; E:stair sample; F:pca sample; G:pca stair; H:length line
clock_t read_start, start, cal_start, now;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
std::string cloudName="plane0";

bool compare_pca(std::vector <Eigen::Matrix3f> a, std::vector <Eigen::Matrix3f> b){return a.size()>b.size();}
bool compare_center(std::vector <Eigen::Vector4f> a, std::vector <Eigen::Vector4f> b){return a.size()>b.size();}
bool compare_main_center(Eigen::Vector4f a, Eigen::Vector4f b){return a(2) < b(2);}
bool border_rm(pcl::PointCloud<pcl::PointXYZ>::Ptr aim, pcl::PointCloud<pcl::PointXYZ>::Ptr result);
bool PCanalyze(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::Vector3f &pc, Eigen::Vector3f &O);


bool Depart(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointIndices> &outputs){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> prec;

  now = clock();
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(VX0, VX0, VX0);
  vg.filter(*cloud_filtered);
  pcl::copyPointCloud(*cloud_filtered, *cloud_ptr);
  if(STEP == 'A')ROS_INFO("A1/vg finished: %d left, in %f s",cloud_ptr->width,  (double)(clock() - now) / CLOCKS_PER_SEC);
  if(STEP == 'D'){
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, color, "cloud_ptr");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ptr");
  }

  now = clock();
  prec.setClusterTolerance (0.1);
  prec.setMinClusterSize (0.3 * cloud_ptr->width);
  prec.setMaxClusterSize (cloud_ptr->width);
  prec.setSearchMethod (tree);
  prec.setInputCloud (cloud_ptr);
  prec.extract (outputs);
  if(STEP == 'A')ROS_INFO("A2/pre-EC finished in %f s", (double)(clock() - now) / CLOCKS_PER_SEC);

  if(STEP == 'A')
    for (int j=0;j<outputs.size();j++){
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for(std::vector<int>::const_iterator t = outputs[j].indices.begin();t!=outputs[j].indices.end (); ++t)
        cloud_cluster->points.push_back (cloud_ptr->points[*t]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_cluster, 250-j*20, j*20, 125+j*10);
      viewer->addPointCloud(cloud_cluster, red, "cloud"+(char)j);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"+(char)j);
    }
  return 1;
}


bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointIndices &part_index){

  pcl::PointCloud<pcl::PointXYZ>::Ptr part(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr fined(new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<int>::const_iterator t = part_index.indices.begin();t!=part_index.indices.end (); ++t)
    part->points.push_back (cloud_ptr->points[*t]);
  part->width = part->points.size ();
  part->height = 1;
  part->is_dense = true;

  now = clock();
  border_rm(part, fined);
  if(STEP == 'B')ROS_INFO("B1/doundary remove finished in %f s", (double)(clock() - now) / CLOCKS_PER_SEC);
  if(STEP == 'B' || STEP == 'C'){
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(fined, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (fined, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  }


  now = clock();
  Eigen::Vector3f main_pc, dO;
  PCanalyze(fined,main_pc,dO);
  if(STEP == 'B')ROS_INFO("B2/PCA finished in %f s", (double)(clock() - now) / CLOCKS_PER_SEC);


    Eigen::Vector3f e2;e2(0)=-1*main_pc(2)*main_pc(0);e2(1)=-1*main_pc(2)*main_pc(1);e2(2)=1-main_pc(2)*main_pc(2);
    Eigen::Vector3f e1 = e2.cross(main_pc);
    e1.normalize();
    e2.normalize();
    std::vector<int> count_list_r(120),count_list_l(120);
    double DO = -1 * main_pc.dot(dO);//(main_pc(0) * dO(0) + main_pc(1) * dO(1) + main_pc(2) * dO(2));
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = cloud_ptr;
    for(int j=0;j<target->width;j++){
      double distance = (target->points.at(j).x * main_pc(0) + target->points.at(j).y * main_pc(1) + target->points.at(j).z * main_pc(2) + DO)/sqrt(main_pc(0)*main_pc(0) + main_pc(1) * main_pc(1) + main_pc(2) * main_pc(2));
      if(fabs(distance) < 1.2)
        if(distance > 0){
          count_list_r[(int)(distance/0.01)]++;
          slices[120+(int)(distance/0.01)].points.push_back(target->points[j]);
          Eigen::Vector3f P;P(0)=target->points.at(j).x;P(1)=target->points.at(j).y;P(2)=target->points.at(j).z;
          P = P-dO;
          Eigen::Vector2f xy;xy(0)=400+100*P.dot(e1);xy(1)=300-100*P.dot(e2);
          drawPs[120+(int)(distance/0.01)].push_back(xy);
        }else{
          count_list_l[(int)(-1*distance/0.01)]++;
          slices[120-(int)(-1*distance/0.01)].points.push_back(target->points[j]);
          Eigen::Vector3f P;P(0)=target->points.at(j).x;P(1)=target->points.at(j).y;P(2)=target->points.at(j).z;
          P = P-dO;
          Eigen::Vector2f xy;xy(0)=400+100*P.dot(e1);xy(1)=300-100*P.dot(e2);
          drawPs[120-(int)(-1*distance/0.01)].push_back(xy);
        }
    }
  return 1;
}


bool border_rm(pcl::PointCloud<pcl::PointXYZ>::Ptr aim, pcl::PointCloud<pcl::PointXYZ>::Ptr result){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
  normEst.setInputCloud(aim); 
  normEst.setRadiusSearch(0.1);
  normEst.setSearchMethod(tree);
  normEst.compute(*normals);
  boundEst.setInputCloud(aim);
  boundEst.setInputNormals(normals);
  boundEst.setRadiusSearch(0.09);
  boundEst.setAngleThreshold(M_PI/4);
  boundEst.setSearchMethod(tree);
  boundEst.compute(boundaries);
  for(int i = aim->points.size() - 1; i >= 0; i--)
    if(!(boundaries[i].boundary_point > 0))
      result->push_back(aim->points[i]);
return 1;
}



bool PCanalyze(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::Vector3f &pc, Eigen::Vector3f &O){
  int q=0;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (0.1 * cloud_ptr->width);
  ec.setMaxClusterSize (cloud_ptr->width);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_indices);

  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > candidates;
  std::vector< std::vector <Eigen::Vector4f> > center_list;
  std::vector< std::vector <Eigen::Matrix3f> > pca_list;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > origins;
  for (int j=0;j<cluster_indices.size();j++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = cluster_indices[j].indices.begin(); pit != cluster_indices[j].indices.end(); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    origins.resize(origins.size() + 1);
    origins[origins.size()-1] = *cloud_cluster;

    pcl::PointCloud<pcl::PointXYZ>::iterator index;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_cluster);
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Ssample = cloud_cluster;
    std::vector<int> pos_normal,neg_normal;
    double distance, ave_dis_p = 0, ave_dis_n = 0;
    for(int i=0;i<Ssample->width;){
      distance = (Ssample->points.at(i).x * coefficients->values[0] + Ssample->points.at(i).y * coefficients->values[1] + Ssample->points.at(i).z * coefficients->values[2] + coefficients->values[3])/sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1] * coefficients->values[1] + coefficients->values[2] * coefficients->values[2]);
      if(fabs(distance) < 0.2 || fabs(distance) > 1){Ssample->erase(Ssample->begin() + i);continue;
      }else if(distance>0){ave_dis_p += distance;pos_normal.push_back(i);
      }else if(distance<0){ave_dis_n += distance;neg_normal.push_back(i);
      }
      i++;
    }
    if(fabs(ave_dis_p/pos_normal.size())>fabs(ave_dis_n/neg_normal.size())) 
      for(;neg_normal.size()>0;){index = Ssample->begin() + neg_normal.back();neg_normal.pop_back();Ssample->erase(index);}
    else 
      for(;pos_normal.size()>0;){index = Ssample->begin() + pos_normal.back();pos_normal.pop_back();Ssample->erase(index);}

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(Ssample);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(8);
    ror.setNegative(false);
    ror.filter(*filtered);

    std::vector<pcl::PointIndices> clusters;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (filtered->width);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered);
    ec.extract (clusters);

    if((int)clusters.size() > 4){
      candidates.resize(candidates.size() + 1);
      center_list.resize(center_list.size() + 1);
      pca_list.resize(pca_list.size() + 1);
      for (int k=0;k<clusters.size();k++){
        candidates[q].resize(candidates[q].size() + 1);
        center_list[q].resize(center_list[q].size() + 1);
        pca_list[q].resize(pca_list[q].size() + 1);
        for (std::vector<int>::const_iterator pitt = clusters[k].indices.begin (); pitt != clusters[k].indices.end (); ++pitt)
          candidates[q][k].points.push_back (filtered->points[*pitt]);
        candidates[q][k].width = candidates[q][k].points.size ();
        candidates[q][k].height = 1;
        candidates[q][k].is_dense = true;

        if(STEP == 'B'){
          cloudName.resize(7);
          cloudName[5] = (char)q;
          cloudName[6] = (char)k;
          pcl::PointCloud<pcl::PointXYZ>::Ptr stair(new pcl::PointCloud<pcl::PointXYZ>);
          stair = candidates[q][k].makeShared();
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(stair, 250-j*50, j*50, 0);
          viewer->addPointCloud(stair, red, cloudName);
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        points = candidates[q][k].makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(points);
        pca.project(*points, *cloudPCAprojection);

        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*points, pcaCentroid);
        center_list[q][k] = pcaCentroid;
        Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
        Eigen::Vector3f eigenValuesPCA = pca.getEigenValues();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
        pca_list[q][k] = eigenVectorsPCA;
        if(STEP == 'B'){
          cloudName.resize(7);
          cloudName[5] = (char)q;
          cloudName[6] = (char)k;
          pcl::PointXYZ O,B,C,D;
          O.x=center_list[q][k](0);O.y=center_list[q][k](1);O.z=center_list[q][k](2);
          B.x=O.x + eigenVectorsPCA.col(0)(0);B.y=O.y + eigenVectorsPCA.col(0)(1);B.z=O.z + eigenVectorsPCA.col(0)(2);
          C.x=O.x + eigenVectorsPCA.col(1)(0);C.y=O.y + eigenVectorsPCA.col(1)(1);C.z=O.z + eigenVectorsPCA.col(1)(2);
          D.x=O.x + eigenVectorsPCA.col(2)(0);D.y=O.y + eigenVectorsPCA.col(2)(1);D.z=O.z + eigenVectorsPCA.col(2)(2);
          viewer->addArrow<pcl::PointXYZ>(B, O, 0, 255, 0, false, "pc1"+cloudName,0);
          viewer->addArrow<pcl::PointXYZ>(C, O, 0, 255, 0, false, "pc2"+cloudName,0);
          viewer->addArrow<pcl::PointXYZ>(D, O, 0, 255, 0, false, "pc3"+cloudName,0);
        }
      }
      q++;
    }else origins.pop_back();
  }
  for(int i=0;i<pca_list.size();i++){
    std::vector< std::vector <Eigen::Matrix3f> > pca_temp;
    std::vector< std::vector <Eigen::Vector4f> > center_temp;
    pca_temp.resize(pca_temp.size() + 1);
    pca_temp[0].push_back(pca_list[0][0]);
    center_temp.resize(center_temp.size() + 1);
    center_temp[0].push_back(center_list[0][0]);
    for(int j=1;j<pca_list[i].size();j++){
      bool push_flag = 0;
      for(int k=0;k<pca_temp.size();k++)
        if(atan2(pca_list[i][j].col(0).cross(pca_temp[k][0].col(0)).norm(), pca_list[i][j].col(0).transpose() * pca_temp[k][0].col(0)) < 3.14/16){
          pca_temp[k].push_back(pca_list[i][j]);
          center_temp[k].push_back(center_list[i][j]);
          push_flag = true;
        }
      if(push_flag == false){
        pca_temp.resize(pca_temp.size() + 1);
        pca_temp[pca_temp.size() - 1].push_back(pca_list[i][j]);
        center_temp.resize(center_temp.size() + 1);
        center_temp[center_temp.size() - 1].push_back(center_list[i][j]);
      }
    }
    sort(pca_temp.begin(),pca_temp.end(),compare_pca);
    sort(center_temp.begin(),center_temp.end(),compare_center);
 
    Eigen::Vector3f main_pc,scnd_pc,thrd_pc,temp_pc;
    for(int j=1;j<pca_temp[0].size();j++){
      main_pc += pca_temp[0][j].col(0);
      if(atan2(pca_temp[0][j].col(1).cross(pca_temp[0][0].col(1)).norm(), pca_temp[0][j].col(1).transpose() * pca_temp[0][0].col(1)) > 3.14/4)//45*
      pca_temp[0][j].col(1) = -1 * pca_temp[0][j].col(1);
      scnd_pc += pca_temp[0][j].col(1);
      if(atan2(pca_temp[0][j].col(2).cross(pca_temp[0][0].col(2)).norm(), pca_temp[0][j].col(2).transpose() * pca_temp[0][0].col(2)) > 3.14/4)//45*
      pca_temp[0][j].col(2) = -1 * pca_temp[0][j].col(2);
      thrd_pc += pca_temp[0][j].col(2);
    }
    sort(center_temp[0].begin(),center_temp[0].end(),compare_main_center);
    thrd_pc = main_pc.cross(scnd_pc);
    main_pc = scnd_pc.cross(thrd_pc);
    scnd_pc = thrd_pc.cross(main_pc);	
    main_pc.normalize();
    scnd_pc.normalize();
    thrd_pc.normalize();
    if(center_temp[0].size() < 4 || center_temp[0].size() < 0.8 * center_temp[0].size())continue;

    double length = 0, depth = 0;
    for(int j=1;j<center_temp[0].size();j++){
      Eigen::Vector4f arr = center_temp[0][j] - center_temp[0][j-1];
      Eigen::Vector3f ar;
      ar(0) = arr(0);ar(1) = arr(1); ar(2) = arr(2);
      depth += (double)ar.dot(scnd_pc);
      length += (double)ar.dot(thrd_pc);
    }
    length = fabs(length / center_temp[0].size());
    depth = fabs(depth / center_temp[0].size());
 
    Eigen::Vector3f dO;
    for(int j=0;j<center_temp[0].size();j++){dO(0) += center_temp[0][j](0);dO(1) += center_temp[0][j](1);dO(2) += center_temp[0][j](2);}
    dO /= center_temp[0].size();
    O = dO;
    pc = main_pc;
    L = length*100+3;
    D = depth*100+4;
    N = center_temp[0].size();
    FLC(0)=center_temp[0][0](0);FLC(1)=center_temp[0][0](1);FLC(2)=center_temp[0][0](2);
    W_pc=main_pc;D_pc=scnd_pc;L_pc=thrd_pc;
  }
return 1;
}


bool Coff(int index){
  cv::Mat img_sample(300, 400, CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat img_temp(600, 800, CV_8UC3, cv::Scalar(255,255,255));
  for(int i=0;i<drawPs[120].size();i++)
    cv::circle(img_sample, cv::Point(drawPs[120][i](0)-200, drawPs[120][i](1)-150), 3,cv::Scalar(255,0,0),-1);
  for(int i=0;i<drawPs[index].size();i++)
    cv::circle(img_temp, cv::Point(drawPs[index][i](0), drawPs[index][i](1)), 3,cv::Scalar(255,0,0),-1);

  cv::Mat result(img_temp.cols - img_sample.cols + 1, img_temp.rows - img_sample.rows + 1, CV_32FC1);
  cv::matchTemplate(img_temp, img_sample, result, CV_TM_CCOEFF);
  normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  double minValue, maxValue;
  cv::Point minLocation, maxLocation;
  minMaxLoc(result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
  double floating = sqrt((maxLocation.x-200)*(maxLocation.x-200) + (maxLocation.y-150)*(maxLocation.y-150))/60;

  if(floating < 1)return true;
  else return false;
}


void barCB(int, void*){
  cv::Mat show(600, 1600, CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat img_sample(300, 400, CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat img_temp(600, 800, CV_8UC3, cv::Scalar(255,255,255));

  cv::line(show, cv::Point(800, 0), cv::Point(800, 600), cv::Scalar(0, 0, 0), 2);
  cv::rectangle(show, cv::Rect(200, 150, 400, 300), cv::Scalar(0, 255, 255), 3);

  for(int i=0;i<drawPs[120].size();i++){
    cv::circle(show, cv::Point(drawPs[120][i](0), drawPs[120][i](1)), 3,cv::Scalar(0,0,255),-1);
    cv::circle(img_sample, cv::Point(drawPs[120][i](0)-200, drawPs[120][i](1)-150), 3,cv::Scalar(255,0,0),-1);
  }
  for(int i=0;i<drawPs[index_pic].size();i++){
    cv::circle(show, cv::Point(800+drawPs[index_pic][i](0), drawPs[index_pic][i](1)), 3,cv::Scalar(255,0,0),-1);
    cv::circle(img_temp, cv::Point(drawPs[index_pic][i](0), drawPs[index_pic][i](1)), 3,cv::Scalar(255,0,0),-1);
  }
  cv::Mat result(img_temp.cols - img_sample.cols + 1, img_temp.rows - img_sample.rows + 1, CV_32FC1);
  cv::matchTemplate(img_temp, img_sample, result, CV_TM_CCOEFF);

  normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
  double minValue, maxValue;
  cv::Point minLocation, maxLocation;
  minMaxLoc(result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());

  cv::Point matchLocation = maxLocation;
  matchLocation.x += 800;
  cv::rectangle(show, matchLocation, cv::Point(matchLocation.x + img_sample.cols, matchLocation.y + img_sample.rows), cv::Scalar(0, 0, 255), 2, 8, 0);

  std::string text = "";
  std::stringstream ss;
  double floating = sqrt((matchLocation.x-1000)*(matchLocation.x-1000) + (matchLocation.y-150)*(matchLocation.y-150))/60;
  ss << setprecision(3) << "float:" << floating*100 << "%"; 
  ss >> text;
  if(floating < 1)
    putText(show, text, cv::Point(0, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0), 1, 8, 0);
  else 
    putText(show, text, cv::Point(0, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1, 8, 0);
  cv::imshow("Detected",show);
}


void stair_obj::show(void){
  std::string Line_name = "Line0";
  ROS_INFO("is showing");
  for(int i=0; i<num;i++){
    Eigen::Vector3f LC = firstLC + i * length * L_pc + i * depth * D_pc;
    Eigen::Vector3f A,B,C,D,E,F;
    A = LC + 0.5 * width * W_pc + 0.5 * depth * D_pc;
    B = A - depth * D_pc;
    C = B - width * W_pc;
    D = C + depth * D_pc;
    E = A + length * L_pc;
    F = D + length * L_pc;
    Line_name[4]=(char)i;
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(A(0), A(1), A(2)), pcl::PointXYZ(B(0), B(1), B(2)),255,0,0, Line_name+"lineAB");
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(B(0), B(1), B(2)), pcl::PointXYZ(C(0), C(1), C(2)),255,0,0, Line_name+"lineBC");
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(C(0), C(1), C(2)), pcl::PointXYZ(D(0), D(1), D(2)),255,0,0, Line_name+"lineCD");
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(D(0), D(1), D(2)), pcl::PointXYZ(A(0), A(1), A(2)),255,0,0, Line_name+"lineDA");
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(A(0), A(1), A(2)), pcl::PointXYZ(E(0), E(1), E(2)),255,0,0, Line_name+"lineAE");
    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(D(0), D(1), D(2)), pcl::PointXYZ(F(0), F(1), F(2)),255,0,0, Line_name+"lineDF");
  }
}
