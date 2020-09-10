#include <detect.h>



int main (int argc, char **argv){
  if(argv[1] == NULL){ROS_INFO("\nneed target(.pcd) name. \n");return 1;}
  if(argv[2] != NULL)STEP = argv[2][0];
  //***construct a ROS Node
  ros::init(argc, argv, "pcl_create");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("louti", 1);
  //***read pcd in
  read_start = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(DIR+(std::string)argv[1]+".pcd", *origin);
  ROS_INFO("read( %d ) | spend %lf s",origin->width,  (double)(clock() - read_start) / CLOCKS_PER_SEC);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);


std::ofstream log(log_DIR, ios::app);

ROS_INFO("calculating...");
int i=0,j=0,k=0,q=0;


//**********************PREDEVIDE********************
std::vector<pcl::PointIndices> pre_indices;
cal_start = clock();
Depart(origin, pre_indices);
log << setprecision(3) << (double)(clock() - cal_start) / CLOCKS_PER_SEC << "	";


//**********************DETECTE**********************
cal_start = clock();
for(int idx = 0; idx < pre_indices.size(); idx++){
  Detect(origin, pre_indices[idx]);


}
log << setprecision(3) << (double)(clock() - cal_start) / CLOCKS_PER_SEC << "	";


/*
//**3**C**remove boundaries
cal_start = clock();
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = AP[0].makeShared();
now = clock();
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
normEst.setInputCloud(cloud_ptr); 
normEst.setRadiusSearch(NR);
normEst.setSearchMethod(tree);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
normEst.compute(*normals);
log << setprecision(3) << (double)(clock() - now) / CLOCKS_PER_SEC << "	";
if(STEP == 'C')
viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_ptr, normals, 20, 0.1, "normals");
if(STEP == 'C')
ROS_INFO("normals finished in %f s", (double)(clock() - now) / CLOCKS_PER_SEC);

now = clock();
pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
pcl::PointCloud<pcl::Boundary> boundaries;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); 
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
boundEst.setInputCloud(cloud_ptr);
boundEst.setInputNormals(normals);
boundEst.setRadiusSearch(0.09);
boundEst.setAngleThreshold(M_PI/4);
boundEst.setSearchMethod(tree);
boundEst.compute(boundaries);
for(int i = cloud_ptr->points.size() - 1; i >= 0; i--)
  if(boundaries[i].boundary_point > 0){
    cloud_boundary->push_back(cloud_ptr->points[i]);
    cloud_ptr->erase(cloud_ptr->begin() + i);
  }
log << setprecision(3) << (double)(clock() - now) / CLOCKS_PER_SEC << "	";
if(STEP == 'C')
ROS_INFO("boundary finished in %f s", (double)(clock() - now) / CLOCKS_PER_SEC);

if(STEP != 'K')ROS_INFO("boundary( %d ) | spend %lf s",cloud_ptr->width,  (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~
if(STEP == 'C'){
viewer->setBackgroundColor (0, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dbc(cloud_boundary, 255, 0, 0);
viewer->addPointCloud<pcl::PointXYZ> (cloud_boundary, dbc, "boundary");
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "boundary");

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, 0, 255, 0);
viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, single_color, "sample cloud");
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
}



//**4**D**EC to depart
cal_start = clock();

now = clock();
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
ec.setClusterTolerance (0.1);
ec.setMinClusterSize (0.1 * cloud_ptr->width);
ec.setMaxClusterSize (cloud_ptr->width);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_ptr);
ec.extract (cluster_indices);
log << setprecision(3) << (double)(clock() - now) / CLOCKS_PER_SEC << "	";
if(STEP == 'D')
ROS_INFO("euclideanC finished in %lf s", (double)(clock() - now) / CLOCKS_PER_SEC);

if(STEP != 'K')ROS_INFO("EC( %d c ) | spend %lf s", (int)cluster_indices.size(),  (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~
if(STEP == 'D')
for (j=0;j<cluster_indices.size();j++){
  std::cout << cluster_indices[j].indices.size() << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<int>::const_iterator t = cluster_indices[j].indices.begin();t!=cluster_indices[j].indices.end (); ++t)
    cloud_cluster->points.push_back (cloud_ptr->points[*t]);
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_cluster, 250-j*50, j*50, 125+j*25);
  viewer->addPointCloud(cloud_cluster, red, "cloud"+(char)j);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"+(char)j);
}



//**********************ANALYZES********************
std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > candidates;
std::vector< std::vector <Eigen::Vector4f> > center_list;
std::vector< std::vector <Eigen::Matrix3f> > pca_list;
std::vector<pcl::PointCloud<pcl::PointXYZ> > origins;

for (j=0;j<cluster_indices.size();j++){
//**5**E**RANSAC to remove the walls
  cal_start = clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = cluster_indices[j].indices.begin(); pit != cluster_indices[j].indices.end(); ++pit)
    cloud_cluster->points.push_back (cloud_ptr->points[*pit]);
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  origins.resize(origins.size() + 1);
  origins[origins.size()-1] = *cloud_cluster;
  if(STEP != 'K')std::cout << "cluster" << j << "(" << cloud_cluster->width << ") :";

  now = clock();
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
  if(STEP != 'K')std::cout << 100 * (double)(inliers->indices.size())/(double)(cloud_cluster->width) << "%(" << inliers->indices.size() << ") in wall" << endl;
  log << setprecision(3) << (double)(clock() - now) / CLOCKS_PER_SEC << "	";
  if(STEP == 'E')
    ROS_INFO("wall ransac finished in %lf s", (double)(clock() - now) / CLOCKS_PER_SEC);

  now = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr Ssample = cloud_cluster;
  std::vector<int> pos_normal,neg_normal;
  double distance, ave_dis_p = 0, ave_dis_n = 0;
  for(i=0;i<Ssample->width;){
    distance = (Ssample->points.at(i).x * coefficients->values[0] + Ssample->points.at(i).y * coefficients->values[1] + Ssample->points.at(i).z * coefficients->values[2] + coefficients->values[3])/sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1] * coefficients->values[1] + coefficients->values[2] * coefficients->values[2]);
    if(fabs(distance) < 0.1 || fabs(distance) > 0.9){Ssample->erase(Ssample->begin() + i);continue;
    }else if(distance>0){ave_dis_p += distance;pos_normal.push_back(i);
    }else if(distance<0){ave_dis_n += distance;neg_normal.push_back(i);
    }
    i++;
  }
  if(fabs(ave_dis_p/pos_normal.size())>fabs(ave_dis_n/neg_normal.size())) 
    for(;neg_normal.size()>0;){index = Ssample->begin() + neg_normal.back();neg_normal.pop_back();Ssample->erase(index);}
  else 
    for(;pos_normal.size()>0;){index = Ssample->begin() + pos_normal.back();pos_normal.pop_back();Ssample->erase(index);}
  log << setprecision(3) << (double)(clock() - now) / CLOCKS_PER_SEC << "	";
  if(STEP == 'E')
    ROS_INFO("wall remove finished in %lf s", (double)(clock() - now) / CLOCKS_PER_SEC);

  if(STEP != 'K')ROS_INFO("ranSAC | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~
  if(STEP == 'E'){
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(Ssample, 250-j*50, j*50, 0);
    viewer->addPointCloud(Ssample, red, "cluster"+(char)j);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cluster"+(char)j);
  }



//**6**F**ec stair clusters
  cal_start = clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  ror.setInputCloud(Ssample);
  ror.setRadiusSearch(0.1);
  ror.setMinNeighborsInRadius(8);
  ror.setNegative(false);
  ror.filter(*filtered);
  pcl::copyPointCloud(*filtered, *Ssample);

  std::vector<pcl::PointIndices> clusters;
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (Ssample->width);
  ec.setSearchMethod (tree);
  ec.setInputCloud (Ssample);
  ec.extract (clusters);
  if(STEP != 'K')ROS_INFO("ec stairs | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~



//**7**G**PCA stairs
  cal_start = clock();
  if((int)clusters.size() > 4){
  candidates.resize(candidates.size() + 1);
  center_list.resize(center_list.size() + 1);
  pca_list.resize(pca_list.size() + 1);
  for (k=0;k<clusters.size();k++){
    candidates[q].resize(candidates[q].size() + 1);
    center_list[q].resize(center_list[q].size() + 1);
    pca_list[q].resize(pca_list[q].size() + 1);
    for (std::vector<int>::const_iterator pitt = clusters[k].indices.begin (); pitt != clusters[k].indices.end (); ++pitt)
      candidates[q][k].points.push_back (Ssample->points[*pitt]);
    candidates[q][k].width = candidates[q][k].points.size ();
    candidates[q][k].height = 1;
    candidates[q][k].is_dense = true;

    if(STEP == 'F' || STEP == 'G' || STEP == 'H' || STEP == 'I'){
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

    if(STEP == 'G'){
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
  if(STEP != 'K')ROS_INFO("PCA | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~
  }else{origins.pop_back();if(STEP != 'K')ROS_INFO("PCA | spend 0 s");}
}




//**8**H**analyze PCA result
for(i=0;i<pca_list.size();i++){
  cal_start = clock();

  std::vector< std::vector <Eigen::Matrix3f> > pca_temp;
  std::vector< std::vector <Eigen::Vector4f> > center_temp;
  pca_temp.resize(pca_temp.size() + 1);
  pca_temp[0].push_back(pca_list[0][0]);
  center_temp.resize(center_temp.size() + 1);
  center_temp[0].push_back(center_list[0][0]);
  for(j=1;j<pca_list[i].size();j++){
    bool push_flag = 0;
    for(k=0;k<pca_temp.size();k++)
      if(atan2(pca_list[i][j].col(0).cross(pca_temp[k][0].col(0)).norm(), pca_list[i][j].col(0).transpose() * pca_temp[k][0].col(0)) < 3.14/16){//11.25*
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
  if(STEP == 'H')
    for(j=0;j<pca_temp[0].size();j++){
      cloudName.resize(7);
      cloudName[5] = (char)i;
      cloudName[6] = (char)j;
      pcl::PointXYZ O,B,C,D;
      O.x=center_temp[0][j](0);O.y=center_temp[0][j](1);O.z=center_temp[0][j](2);
      B.x=O.x + pca_temp[0][j].col(0)(0);B.y=O.y + pca_temp[0][j].col(0)(1);B.z=O.z + pca_temp[0][j].col(0)(2);
      C.x=O.x + pca_temp[0][j].col(1)(0);C.y=O.y + pca_temp[0][j].col(1)(1);C.z=O.z + pca_temp[0][j].col(1)(2);
      D.x=O.x + pca_temp[0][j].col(2)(0);D.y=O.y + pca_temp[0][j].col(2)(1);D.z=O.z + pca_temp[0][j].col(2)(2);
      viewer->addArrow<pcl::PointXYZ>(B, O, 0, 255, 0, false, cloudName + "pc1",0);
      viewer->addArrow<pcl::PointXYZ>(C, O, 0, 255, 0, false, cloudName + "pc2",0);
      viewer->addArrow<pcl::PointXYZ>(D, O, 0, 255, 0, false, cloudName + "pc3",0);
    }

  Eigen::Vector3f main_pc,scnd_pc,thrd_pc,temp_pc;
  for(j=1;j<pca_temp[0].size();j++){
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
  if(STEP != 'K')ROS_INFO("analyze PCs | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~



  if(center_temp[0].size() < 4 || center_temp[0].size() < 0.8 * center_temp[0].size())continue;
//**9**I**length
  cal_start = clock();

  double length = 0, depth = 0;
  for(j=1;j<center_temp[0].size();j++){
    Eigen::Vector4f arr = center_temp[0][j] - center_temp[0][j-1];
    Eigen::Vector3f ar;
    ar(0) = arr(0);ar(1) = arr(1); ar(2) = arr(2);
    depth += (double)ar.dot(scnd_pc);
    length += (double)ar.dot(thrd_pc);
  }
  length = fabs(length / center_temp[0].size());
  depth = fabs(depth / center_temp[0].size());
  if(STEP != 'K')ROS_INFO("length & depth | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~

  if(STEP == 'I'){
    cloudName.resize(7);
    cloudName[5] = (char)i;
    pcl::PointXYZ O,B,C,D,C1,C2,C3;
    for(j=1;j<center_temp[0].size();j++){
      cloudName[6] = (char)j;
      C1.x = center_temp[0][j](0);C1.y = center_temp[0][j](1);C1.z = center_temp[0][j](2);
      C2.x = center_temp[0][j-1](0);C2.y = center_temp[0][j-1](1);C2.z = center_temp[0][j-1](2);
      C3 = C1; C3.x += length * thrd_pc(0); C3.y += length * thrd_pc(1); C3.z += length * thrd_pc(2);
      viewer->addLine<pcl::PointXYZ>(C1, C2, 0, 0, 255, cloudName+"center_line");
      viewer->addLine<pcl::PointXYZ>(C3, C1, 0, 255, 0, cloudName+"length");
    }
    O.x=center_temp[0][0](0);O.y=center_temp[0][0](1);O.z=center_temp[0][0](2);
    B.x=O.x + main_pc(0);B.y=O.y + main_pc(1);B.z=O.z + main_pc(2);
    C.x=O.x + scnd_pc(0);C.y=O.y + scnd_pc(1);C.z=O.z + scnd_pc(2);
    D.x=O.x + thrd_pc(0);D.y=O.y + thrd_pc(1);D.z=O.z + thrd_pc(2);
    viewer->addArrow<pcl::PointXYZ>(B, O, 0, 255, 0, false, cloudName+"pc1",0);
    viewer->addArrow<pcl::PointXYZ>(C, O, 0, 255, 0, false, cloudName+"pc2",0);
    viewer->addArrow<pcl::PointXYZ>(D, O, 0, 255, 0, false, cloudName+"pc3",0);
  }



//**10**J**width
  cal_start = clock();

  Eigen::Vector3f dO;
  for(j=0;j<center_temp[0].size();j++){dO(0) += center_temp[0][j](0);dO(1) += center_temp[0][j](1);dO(2) += center_temp[0][j](2);}
  dO /= center_temp[0].size();
  std::vector<int> count_list_r(120),count_list_l(120);
  double DO = -1 * (main_pc(0) * dO(0) + main_pc(1) * dO(1) + main_pc(2) * dO(2));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
  target = origins[i].makeShared();
  for(j=0;j<target->width;j++){
    double distance = (target->points.at(j).x * main_pc(0) + target->points.at(j).y * main_pc(1) + target->points.at(j).z * main_pc(2) + DO)/sqrt(main_pc(0)*main_pc(0) + main_pc(1) * main_pc(1) + main_pc(2) * main_pc(2));
    if(fabs(distance) < 1.2)
      if(distance > 0)
        count_list_r[(int)distance/0.01]++;
      else
        count_list_l[(int)(-1*distance/0.01)]++;
  }
  for(j=count_list_r.size();j>0;j--)  count_list_r[j] = fabs(count_list_r[j] - count_list_r[j-1]);
  count_list_r.erase(count_list_r.begin());
  for(j=1;j<count_list_r.size()-1;j++)  count_list_r[j] = (count_list_r[j-1] + count_list_r[j] + count_list_r[j+1])/3;
  for(j=count_list_l.size();j>0;j--)  count_list_l[j] = fabs(count_list_l[j] - count_list_l[j-1]);
  count_list_l.erase(count_list_l.begin());
  for(j=1;j<count_list_l.size()-1;j++)  count_list_l[j] = (count_list_l[j-1] + count_list_l[j] + count_list_l[j+1])/3;
  double width = (int)(max_element(count_list_r.begin(),count_list_r.end()) - count_list_r.begin()) + (max_element(count_list_l.begin(),count_list_l.end())  - count_list_l.begin());
  if(STEP != 'K')ROS_INFO("width | spend %lf s", (double)(clock() - cal_start) / CLOCKS_PER_SEC);//end~~~~~~~~~~~~~~~~~~~
  if(STEP == 'J' || STEP == 'K'){
  pcl::PointXYZ OC;
  OC.x = center_temp[0][0](0);OC.y = center_temp[0][0](1);OC.z = center_temp[0][0](2);
  cloudName[5] = char(i);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(origin, 0, 255, 0);
  viewer->addSphere(OC, 0.1, cloudName+"sphere", 0);
  viewer->addPointCloud<pcl::PointXYZ> (origin, single_color, cloudName);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName);
  }
  std::cout << endl << "****parameters*************" << endl << "*" << endl;
  std::cout << setprecision(2) << "* location:   ";
  std::cout << "(" << center_temp[0][0](0) << "," << center_temp[0][0](1) << "," << center_temp[0][0](2) << ")" << endl;
  std::cout << setprecision(2) << "* num     :   " << center_temp[0].size() << endl;
  std::cout << setprecision(2) << "* length  :   " << length *100 + len_compn<< "   cm" << endl;
  std::cout << setprecision(2) << "* depth   :   " << depth * 100 + dep_compn<< "   cm" << endl;
  std::cout << setprecision(2) << "* width   :   " << width + wid_compn<< "   cm" << endl;
  std::cout << "*" << endl << "**************************" << endl << endl;
}



  ROS_INFO("done, total spends %lf s", (double)(clock() - start) / CLOCKS_PER_SEC);
*/
  log << endl;
  log.close();
  ROS_INFO("loged");
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

//waitting
  ros::Rate loop_rate(10);
  while (ros::ok()){
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}




    



