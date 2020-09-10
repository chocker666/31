#include "mygridtool.h"

Grided::Grided(){
	double DOI = 10;//兴趣范围/m
	double resolution = 0.05;//栅格分辨率/m
	double h_step = 0.01//单格精度/m
	double start_thresh = 0.1//运行阈值比/1


}




int Grided::setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr){
	//用于检测输入点云是否符合标准
	//进行简单处理，从而确定后续处理中的一些基本参数
	//***********1,栅格范围
	double xM = cloud_ptr->points[0].x;double xm = cloud_ptr->points[0].x;
	double yM = cloud_ptr->points[0].y;double ym = cloud_ptr->points[0].y;
	for(int cloud_itr = 1; cloud_itr < cloud_ptr->points.size(); cloud_itr++){
		if(cloud_ptr->points[cloud_itr].x < xm)xm = cloud_ptr->points[cloud_itr].x;
		else if(cloud_ptr->points[cloud_itr].x > xM)xM = cloud_ptr->points[cloud_itr].x;
		if(cloud_ptr->points[cloud_itr].y < ym)ym = cloud_ptr->points[cloud_itr].y;
		else if(cloud_ptr->points[cloud_itr].y > yM)yM = cloud_ptr->points[cloud_itr].y;
	}
	if((xM-xm) < DOI*start_thresh || (yM-ym) < DOI*start_thresh){
		return 100;
		ROS_WARN("too small");
	}
	M = ((xM-xm) > DOI)?(DOI/resolution):((xM-xm)/resolution);
	N = ((yM-ym) > DOI)?(DOI/resolution):((yM-ym)/resolution);
	return 101;
	ROS_INFO("scene ready,%d,%d",M,N);

}
