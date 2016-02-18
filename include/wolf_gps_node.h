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
#include "wolf/sensor_odom_2D.h"
#include "wolf/wolf.h"
#include "wolf/wolf_problem.h"
#include "wolf/capture_motion.h"
#include "wolf/capture_odom_2D.h"
#include <wolf/capture_fix.h>

/**************************
 *     CERES includes     *
 **************************/
#include "wolf/ceres_wrapper/ceres_manager.h"

/**************************
 *      raw_gps_utils     *
 **************************/
#include "raw_gps_utils/satellites_obs.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
#include <nav_msgs/Odometry.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <queue>

class WolfGPSNode
{
public:
    WolfGPSNode();
    virtual ~WolfGPSNode();


protected:

    /*
     * ROS stuff
     */
    // Subscribers
    ros::Subscriber gps_sub_; // obs (measurements) subscriber
    ros::Subscriber odom_sub_; // obs (measurements) subscriber

    // ROS node handle
    ros::NodeHandle nh_;

    //Odometry callback
    void gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

};


#endif //WOLF_ROS_WOLF_GPS_NODE_H
