//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "wolf_gps_node");

    // Trilateration node
    WolfGPSNode wgNode;

    ros::spin();
    return 0;
}