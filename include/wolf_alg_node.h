// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _wolf_alg_node_h_
#define _wolf_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "wolf_alg.h"
#include <tf/transform_broadcaster.h>

// [publisher subscriber headers]
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WolfAlgNode : public algorithm_base::IriBaseAlgorithm<WolfAlgorithm>
{
  private:

    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;
    SensorOdom2D* odom_sensor_;
    SensorLaser2D* laser_1_sensor_;
    SensorLaser2D* laser_2_sensor_;
    WolfManager* wolf_manager_;
    bool laser_1_params_setted_, laser_2_params_setted_;

    // [publisher attributes]
    ros::Publisher corners_publisher_;
    visualization_msgs::MarkerArray corners_MarkerArray_msg_;


    // [subscriber attributes]
    ros::Subscriber relative_odometry_subscriber_;
    void relative_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t relative_odometry_mutex_;
    void relative_odometry_mutex_enter(void);
    void relative_odometry_mutex_exit(void);

    ros::Subscriber laser_back_subscriber_;
    void laser_back_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_back_mutex_;
    void laser_back_mutex_enter(void);
    void laser_back_mutex_exit(void);

    ros::Subscriber laser_front_subscriber_;
    void laser_front_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_front_mutex_;
    void laser_front_mutex_enter(void);
    void laser_front_mutex_exit(void);


    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    WolfAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~WolfAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
