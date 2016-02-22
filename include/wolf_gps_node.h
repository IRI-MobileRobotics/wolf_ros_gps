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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
#include <nav_msgs/Odometry.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

class WolfGPSNode
{
public:
    WolfGPSNode(SensorGPS* _gps_sensor_ptr,
                const FrameStructure _frame_structure,
                SensorBase* _sensor_prior_ptr,
                const Eigen::VectorXs& _prior,
                const Eigen::MatrixXs& _prior_cov,
                const unsigned int& _trajectory_size,
                const WolfScalar& _new_frame_elapsed_time);
    virtual ~WolfGPSNode();

    bool hasDataToProcess();
    void process();

    //wolf manager methods
    void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);
    void createFrame(const TimeStamp& _time_stamp);
    void manageWindow();
    void addSensor(SensorBase* _sensor_ptr);

protected:
    //-------------- WOLF manager stuff--------------
    //sets the problem
    WolfProblem* problem_;
    FrameStructure frame_structure_;

    //pointer to a sensor providing predictions
    SensorBase* sensor_prior_;

    //auxiliar/temporary iterators, frames and captures
    FrameBaseIter first_window_frame_;
    FrameBase* current_frame_;
    FrameBase* last_key_frame_;
    CaptureMotion* last_capture_relative_;
    CaptureMotion* second_last_capture_relative_;
    std::queue<CaptureBase*> new_captures_;

    //Manager parameters
    unsigned int trajectory_size_;
    WolfScalar new_frame_elapsed_time_;

    void addCapture(CaptureBase* _capture);
    void update();
    bool checkNewFrame(CaptureBase* new_capture);
    Eigen::VectorXs getVehiclePose(const TimeStamp& _now = 0);

    //---------------WOLF NODE STUFF-----------------
    //transforms
    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;
    tf::Transform Tf_map2base_; //wolf output
    tf::Transform Tf_odom2base_; //published by odom source
    tf::Transform Tf_map2odom_; //to be broadcasted by this node
    std::string base_frame_name_;
    std::string gps_frame_name_;

    std::string ecef_frame_name_;
    std::string map_frame_name_;
    std::string odom_frame_name_;

//    void loadGPSTf();
//    bool gps_tf_loaded_;

    //ceres
    int max_iterations_;
    bool use_auto_diff_wrapper_, apply_loss_function_;
    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;
    //-----------------------------------------------


//    E' il sensor prior!
//    //Wolf: odom sensor
//    SensorOdom2D* odom_sensor_ptr_;

    //Wolf: gps sensor
    SensorGPS* gps_sensor_ptr_;

    //Odometry callback
    ros::Time last_odom_stamp_;
    ros::Subscriber odom_sub_; // odometry subscriber
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // GPS callback
    ros::Subscriber gps_sub_; // gps subscriber
    void gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg);
    int gps_data_arrived_;

    // ROS node handle
    ros::NodeHandle nh_;

};


#endif //WOLF_ROS_WOLF_GPS_NODE_H
