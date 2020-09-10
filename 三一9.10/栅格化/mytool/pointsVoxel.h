#include<iostream>
#include <vector>

#define resolution 0.05//栅格分辨率/m
#define h_step 0.01//单格精度/m
#define start_thresh 0.1//运行阈值比/1
#define DOI 10//兴趣范围/m

//构建一个用于存储单个栅格点信息的结构体pos
//定义一个栅格化工具类Grided，模仿官方vox工具，定义初始化函数
//使用时实例化grid，给定输入输出点云
//???可视化工具??使用rviz插件实现
//四叉树节点结构体
struct pos{
	vector<int> h_data;
	bool isGround;//是否为地面栅格，筛选确定地面用
	int cost;//通过代价
};

class Grided{
public:
	Grided();
private:
	int M;//栅格长
	int N;//栅格宽
	int setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
	void setGridSize(double reso_x, double reso_y, double step);
	void setStartThreshold(double s_thresh);
	void setDistanceOfInterst(double dis);
	void generateGridMulTree();//四叉树实现
	void generateGridVector();//二维容器实现
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
	ROS_INFO("scene ready");
}

Grided::Grided(){
	//1，定义二维vector栅格Grids
	//通过原始点云最远点距离算得栅格的size，(int)m, n
	//由于兴趣范围DOI为一定距离之内(~/m)，若上一步范围超过DOI
	//则将手动给定栅格size为DOI，同时抛弃越界点（遍历时跳过）
	//从而将原始点云数据转换为长宽在[M,N]内的pos栅格数据
	//若栅格任意方向小于运行阈值长度，则返回错误100：原始数据过小
	
	
	//初始化参数





}






bool isGround;//是否为地面栅格，筛选确定地面用
double height[3];//高度范围, [0]最低, [1]最高, [3]点云散布情况
