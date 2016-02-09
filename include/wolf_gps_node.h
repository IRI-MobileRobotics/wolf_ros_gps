//
// Created by ptirindelli on 8/02/16.
//

#ifndef WOLF_ROS_WOLF_GPS_NODE_H
#define WOLF_ROS_WOLF_GPS_NODE_H


/**************************
 *      WOLF includes     *
 **************************/
#include "wolf/processor_gps.h"
#include "wolf/sensor_gps.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>

class WolfGPSNode
{
public:
    WolfGPSNode();
    ~WolfGPSNode();

protected:
    // ROS node handle
    ros::NodeHandle nh;

};


#endif //WOLF_ROS_WOLF_GPS_NODE_H
