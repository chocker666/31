/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */
#include <ros/ros.h>
#include "rsdriver.h"
#include "std_msgs/String.h"

using namespace rs_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rsdriver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, my_handler);

    // start the driver
    rs_driver::rslidarDriver dvr(node, private_nh);
    // loop until shut down or end of file
    
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        dvr.poll();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
