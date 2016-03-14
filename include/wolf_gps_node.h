//
// Created by ptirindelli on 8/02/16.
//

#ifndef WOLF_ROS_WOLF_GPS_NODE_H
#define WOLF_ROS_WOLF_GPS_NODE_H

/**************************
 *      raw_gps_utils     *
 **************************/
#include "raw_gps_utils/satellites_obs.h"

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
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

class WolfGPSNode
{
public:
    WolfGPSNode(const Eigen::VectorXs& _prior,
                const unsigned int& _trajectory_size,
                const WolfScalar& _new_frame_elapsed_time,
                const Eigen::Vector3s& _gps_sensor_p,
                Eigen::Vector4s& _map_pose,
                Eigen::Vector2s& _odom_std);
    virtual ~WolfGPSNode();

    bool hasDataToProcess();
    void process();

    //wolf manager methods
    void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);
    void createFrame(const TimeStamp& _time_stamp);
    void manageWindow();

    ros::Time getTimeLastProcess();

    void publishTrajectory(bool verbose = true);

protected:
    //sets the problem
    WolfProblem* problem_;
    FrameStructure frame_structure_;

    //pointer to a sensor providing predictions
    SensorOdom2D* sensor_prior_;
    SensorGPS* gps_sensor_ptr_;

    //auxiliar/temporary iterators, frames and captures
    FrameBaseIter first_window_frame_;
    FrameBase* current_frame_;
    FrameBase* last_key_frame_;
    CaptureMotion* last_capture_relative_;
    CaptureMotion* second_last_capture_relative_;

    //Manager parameters
    unsigned int trajectory_size_;
    WolfScalar new_frame_elapsed_time_;

    void addCapture(CaptureBase* _capture);
    bool checkNewFrame(CaptureBase* new_capture);
    Eigen::VectorXs getVehiclePose(const TimeStamp& _now = 0);

    //transforms
    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;
    //TODO are this 3 things useful?
//    tf::Transform T_base_map_;  //wolf output
//    tf::Transform T_base_odom_; //published by odom source
    tf::Transform T_odom_map_;  //to be broadcasted by this node
    const std::string base_frame_name_ = "teo_base_footprint";
    const std::string gps_frame_name_ = "gps_asterx1";
    const std::string world_frame_name_ = "world";
    const std::string map_frame_name_ = "map";
    const std::string odom_frame_name_ = "teo_odom";
    void broadcastTfWorldMap(Eigen::Vector3s _map_p, Eigen::Vector1s _map_o, Eigen::Vector2s _vehicle_p, Eigen::Vector1s _vehicle_o, Eigen::Vector3s _sensor_p);
    void broadcastTfMapOdom(Eigen::Vector2s _vehicle_p, Eigen::Vector1s _vehicle_o);

    //ceres
    int max_iterations_;
    bool use_auto_diff_wrapper_, apply_loss_function_;
    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;

    //Odometry callback
    ros::Time last_odom_stamp_;
    ros::Subscriber odom_sub_; // odometry subscriber
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // GPS callback
    ros::Subscriber gps_sub_; // gps subscriber
    void gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg);
    int gps_data_arrived_;

    // Publisher (markers)
    ros::Publisher marker_pub_;

    // GPS fix callback
    ros::Subscriber fix_ecef_sub_;
    void fixEcefCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);
    int fix_arrived_;

    ros::Time time_last_process_;

    // ROS node handle
    ros::NodeHandle nh_;
};

#endif //WOLF_ROS_WOLF_GPS_NODE_H