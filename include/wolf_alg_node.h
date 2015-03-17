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

// [publisher subscriber headers]
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WolfAlgNode : public algorithm_base::IriBaseAlgorithm<WolfAlgorithm>
{
  private:
    bool draw_lines_;
    int window_length_;
    double new_frame_elapsed_time_;
    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;
    SensorOdom2D* odom_sensor_;
    std::vector<SensorLaser2D*> laser_sensor_ptrs_;
    WolfManager* wolf_manager_;
    std::vector<bool> laser_params_setted_;
    std::vector<std_msgs::ColorRGBA> lines_colors_;

    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;

    tf::Transform T_odom_;
    tf::Transform T_localization_;
    tf::Transform T_map_base_;

    // [publisher attributes]
    ros::Publisher lines_publisher_;
    visualization_msgs::MarkerArray lines_MarkerArray_msg_;

    ros::Publisher constraints_publisher_;
    visualization_msgs::Marker constraints_Marker_msg_;

    ros::Publisher corners_publisher_;
    visualization_msgs::MarkerArray corners_MarkerArray_msg_;

    ros::Publisher vehicle_publisher_;
    visualization_msgs::MarkerArray vehicle_MarkerArray_msg_;


    // [subscriber attributes]
    ros::Subscriber laser_1_subscriber_;
    void laser_1_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_1_mutex_;
    void laser_1_mutex_enter(void);
    void laser_1_mutex_exit(void);

    ros::Subscriber laser_2_subscriber_;
    void laser_2_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_2_mutex_;
    void laser_2_mutex_enter(void);
    void laser_2_mutex_exit(void);

    ros::Subscriber laser_3_subscriber_;
    void laser_3_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_3_mutex_;
    void laser_3_mutex_enter(void);
    void laser_3_mutex_exit(void);

    ros::Subscriber laser_4_subscriber_;
    void laser_4_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_4_mutex_;
    void laser_4_mutex_enter(void);
    void laser_4_mutex_exit(void);

    ros::Subscriber laser_5_subscriber_;
    void laser_5_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_5_mutex_;
    void laser_5_mutex_enter(void);
    void laser_5_mutex_exit(void);

    ros::Subscriber laser_6_subscriber_;
    void laser_6_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t laser_6_mutex_;
    void laser_6_mutex_enter(void);
    void laser_6_mutex_exit(void);

    ros::Subscriber relative_odometry_subscriber_;
    void relative_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t relative_odometry_mutex_;
    void relative_odometry_mutex_enter(void);
    void relative_odometry_mutex_exit(void);


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

    void computeLaserScan(CaptureLaser2D* new_capture, const std_msgs::Header & header, const unsigned int laser_idx);

    void updateLaserParams(const unsigned int laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif
