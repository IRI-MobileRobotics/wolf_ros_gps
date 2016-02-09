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
#include "wolf/wolf.h"
#include "wolf/wolf_problem.h"

/**************************
 *     CERES includes     *
 **************************/
#include "wolf/ceres_wrapper/ceres_manager.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <queue>

class WolfGPSNode
{
public:
    WolfGPSNode(StateBlock* _sensor_p,
                StateBlock* _sensor_o,
                StateBlock* _sensor_bias,
                StateBlock* _vehicle_init_p,
                StateBlock* _vehicle_init_o,
                SensorBase* _sensor_prior_ptr,
                const Eigen::VectorXs& _prior,
                const Eigen::MatrixXs& _prior_cov,
                const unsigned int& _trajectory_size = 10,
                const WolfScalar& _new_frame_elapsed_time = 0.1);
    virtual ~WolfGPSNode();

    void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);
    void createFrame(const TimeStamp& _time_stamp);
    void manageWindow();

    void obsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg);

protected:
    void initCeresManager();

protected:
    /*
     * WOLF stuff
     */
    // Parameters, to be optimized
    StateBlock* sensor_p_;          // gps sensor position
    StateBlock* sensor_o_;          // gps sensor orientation
    StateBlock* sensor_bias_;       // gps sensor bias
    StateBlock* vehicle_init_p_;    // vehicle init position
    StateBlock* vehicle_init_o_;    // vehicle init orientation

    // GPS sensor
    SensorGPS* gps_sensor_ptr_;

    // Wolf Problem
    WolfProblem* problem_;
    FrameStructure frame_structure_; // always = PO_3D TODO maybe is not needed

    //pointer to a sensor providing predictions
    SensorBase* sensor_prior_;

    //TODO del prossimo blocco, vedi cosa serve veramente
    //auxiliar/temporary iterators, frames and captures
    FrameBaseIter first_window_frame_;
    FrameBase* current_frame_;
    FrameBase* last_key_frame_;
    CaptureMotion* last_capture_relative_;
    CaptureMotion* second_last_capture_relative_;
    std::queue<CaptureBase*> new_captures_;

    //TODO del prossimo blocco, vedi cosa serve veramente
    //Manager parameters
    unsigned int trajectory_size_;
    WolfScalar new_frame_elapsed_time_;

    CeresManager* ceres_manager_;

    /*
     * ROS stuff
     */
    // Subscribers
    ros::Subscriber obs_sub_; // obs (measurements) subscriber
    // ROS node handle
    ros::NodeHandle nh_;

};


#endif //WOLF_ROS_WOLF_GPS_NODE_H
