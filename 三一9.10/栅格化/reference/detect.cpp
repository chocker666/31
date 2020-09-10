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

  std::ofstream log(log_DIR, ios::app);
  ROS_INFO("calculating...");
//**********************PREDEVIDE********************
  std::vector<pcl::PointIndices> pre_indices;
  cal_start = clock();
  Depart(origin, pre_indices);
  log << setprecision(3) << (double)(clock() - cal_start) / CLOCKS_PER_SEC << "	";


//**********************DETECTE**********************
  cal_start = clock();
  for(int idx = 0; idx < pre_indices.size(); idx++)
    Detect(origin, pre_indices[idx]);
  log << setprecision(3) << (double)(clock() - cal_start) / CLOCKS_PER_SEC << "	";


//**********************Cofficient*******************
  cal_start = clock();
  int countL=0,countR=0;
  for(int i=80;i>70;i--)
    if(!Coff(i))
      countL++;
  for(int i=160;i<170;i++)
    if(!Coff(i))
      countR++;
  int width = 80 + countL + countR;
  log << setprecision(3) << (double)(clock() - cal_start) / CLOCKS_PER_SEC << "	";
  W = width;

  if(STEP == 'C'){
  cv::namedWindow("Detected");
  cv::createTrackbar("num: ","Detected",&index_pic,230,barCB);
  barCB(0,0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr slice = slices[120].makeShared();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(slice, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (slice, color, "slice");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slice");
  }


//**********************Modelling****************
  stair_obj Stair(W,L,D,N,FLC);
  if(STEP == 'D'){
    Stair.show();
    std::cout << W << endl << L << endl << D << endl;
    std::cout << "(" << FLC(0) << "," << FLC(1) << "," << FLC(2) << ")" << endl;
  }

  ROS_INFO("done, total spends %lf s", (double)(clock() - start) / CLOCKS_PER_SEC);
  log << endl;
  log.close();
  ROS_INFO("loged");
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

//waitting
  ros::Rate loop_rate(50);
  while (ros::ok()){
    if(STEP == 'C'){
      cv::waitKey(1);
      viewer->removePointCloud("slice");
      pcl::PointCloud<pcl::PointXYZ>::Ptr slice = slices[index_pic].makeShared();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(slice, 0, 0, 255);
      viewer->addPointCloud<pcl::PointXYZ> (slice, color, "slice");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slice");
    } 
    viewer->spinOnce(1);
    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}




    



