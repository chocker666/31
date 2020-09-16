#include "convert.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "Pre-Exc");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    rslidar_pointcloud::Convert conv(node, priv_nh);
    //ros::AsyncSpinner s(2);
    //s.start();
    //ros::waitForShutdown();

    ros::spin();
    return 0;
}
