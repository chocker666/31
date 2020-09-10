#include "mygridtool.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "GridNode");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("unGrided", 1000, chatterCallback);



return 0;
}
