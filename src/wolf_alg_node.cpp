#include "wolf_alg_node.h"

WolfAlgNode::WolfAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<WolfAlgorithm>(),
    //odom_sensor_pose_(3),
    odom_sensor_point_(odom_sensor_pose_.data()),
    odom_sensor_theta_(&odom_sensor_pose_(3)),
    laser_sensor_pose_(6),
    laser_sensor_point_(6),
    //laser_sensor_theta_(6),
    laser_sensor_orientation_(6),
    laser_sensor_ptr_(6),
    laser_params_set_({false, false, false, false, false, false}),
    line_colors_(6),
    draw_lines_(false), 
    laser_subscribers_(6), 
    laser_mutexes_(6)
{
    //init class attributes if necessary
    WolfScalar odom_std[2];
    int state_initial_length;
    public_node_handle_.param<int>("window_length", window_length_, 35);
    public_node_handle_.param<double>("odometry_translational_std", odom_std[0], 0.1);
    public_node_handle_.param<double>("odometry_rotational_std", odom_std[1], 0.2);
    public_node_handle_.param<int>("state_initial_length", state_initial_length, 1e6);
    this->loop_rate_ = 10;//in [Hz] ToDo: should be an input parameter from cfg
 
    // init ceres
    ceres_options_.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION;
    ceres_options_.max_line_search_step_contraction = 1e-3;
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_manager_ = new CeresManager(problem_options_);

    // init wolf odom sensor 
    odom_sensor_pose_ = Eigen::Vector4s::Zero(); //odom sensor coniciding with vehicle base link: xyz + theta
    odom_sensor_ptr_ = new SensorOdom2D(&odom_sensor_point_, &odom_sensor_theta_, odom_std[0], odom_std[1]);//both arguments initialized on top

    // init wolf laser sensors:
//     laser_sensor_pose_[0] << 3.5,0,0,0,0,0;
//     laser_sensor_pose_[1] << 3.1,-0.78,0,0,0,-1.65806;//3.1 -0.78 0.5 yaw:-1.65806
//     laser_sensor_pose_[2] << -0.5,-0.8,0,0,0,-1.46608;//-0.5 -0.8  0.5  yaw:-1.46608
//     laser_sensor_pose_[3] << -0.9,0,0,0,0,-3.12414;//-0.9  0,   0.45 yaw:-3.12414
//     laser_sensor_pose_[4] << -0.5,0.8,0,0,0,1.48353;//-0.5 -0.8  0.5  yaw:1.48353
//     laser_sensor_pose_[5] << 3.1,0.8,0,0,0,1.64388;//3.1  0.8  0.55 yaw:1.64388
    std::stringstream lidar_frame_name_ii; 
    tf::StampedTransform base_2_lidar_ii; 
    for (uint ii = 0; ii<6; ii++)
    {
        //build name
        lidar_frame_name_ii.str("");
        lidar_frame_name_ii << "agv_lidar" << ii;
        
        //look up for transform from base to ibeo
        //std::cout << "waiting for transform: " << lidar_frame_name_ii.str() << std::endl;
        if ( tfl_.waitForTransform("agv_base_link", lidar_frame_name_ii.str(), ros::Time(0), ros::Duration(1.)) )
        {
            //look up for transform at TF
            tfl_.lookupTransform("agv_base_link", lidar_frame_name_ii.str(), ros::Time(0), base_2_lidar_ii);

            //Set mounting frame. Fill translation part
            laser_sensor_pose_[ii].head(3) << base_2_lidar_ii.getOrigin().x(),base_2_lidar_ii.getOrigin().y(),base_2_lidar_ii.getOrigin().z();
            
            //Set mounting frame. Fill quaternion representing rotation part.
            //laser_sensor_pose_[ii].tail(3) << 0,0,base_2_lidar_ii.getRotation().getAngle();//assumes rotation only in Z (2D)
            laser_sensor_pose_[ii].tail(4) << base_2_lidar_ii.getRotation().getX(),
                                              base_2_lidar_ii.getRotation().getY(),
                                              base_2_lidar_ii.getRotation().getZ(),
                                              base_2_lidar_ii.getRotation().getW();
            
            //DEBUG: prints 2D lidar pose
            //std::cout << "LIDAR " << ii << ": " << lidar_frame_name_ii.str() << ": " << laser_sensor_pose_[ii].transpose() << std::endl;
            
            //set wolf states and sensors
            laser_sensor_point_[ii] = new StatePoint3D(laser_sensor_pose_[ii].data());
            //laser_sensor_theta_[ii] = new StateTheta(&laser_sensor_pose_[ii](5));
            laser_sensor_orientation_[ii] = new StateQuaternion(&laser_sensor_pose_[ii](3));
            //laser_sensor_ptr_[ii] = new SensorLaser2D(laser_sensor_point_[ii], laser_sensor_theta_[ii]);
            laser_sensor_ptr_[ii] = new SensorLaser2D(laser_sensor_point_[ii], laser_sensor_orientation_[ii]);
        }
        else
        {
            std::cout << "WARNING: No TF found from agv_base_link to " << lidar_frame_name_ii.str() << std::endl << std::endl;
        }
    }

    //create the manager
    wolf_manager_ = new WolfManager(state_initial_length, odom_sensor_ptr_,Eigen::Vector3s::Zero(), 
                                    Eigen::Matrix3s::Identity()*0.01, window_length_, new_frame_elapsed_time_, false);

    // [init publishers]
    this->lines_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("lines", 1);
    this->constraints_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("constraints", 1);
    this->corners_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("corners", 1);
    this->vehicle_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vehicle", 1);
    
    // Broadcast 0 transform to align frames initially
    T_map2base_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    tfb_.sendTransform( tf::StampedTransform(T_map2base_, ros::Time::now(), "map", "odom"));

    // [init subscribers]
    
    //init odometry subscriber and odometry mutex
    this->relative_odometry_subscriber_ = this->public_node_handle_.subscribe("relative_odometry", 10,
                                                                              &WolfAlgNode::relative_odometry_callback, this);
    pthread_mutex_init(&this->relative_odometry_mutex_,nullptr);
    
    //init lidar subscribers and mutexes
    std::stringstream lidar_topic_name_ii; 
    for (unsigned int ii = 0; ii<6; ii++)
    {
        //build topic name
        lidar_topic_name_ii.str("");
        lidar_topic_name_ii << "laser_" << ii;
        
        //init the subsrciber
        this->laser_subscribers_[ii] = this->public_node_handle_.subscribe(lidar_topic_name_ii.str(),20,&WolfAlgNode::laser_callback,this);
        
        //init the mutex
        pthread_mutex_init(&this->laser_mutexes_[ii],nullptr);
    }
    
    // [init services]
    
    // [init clients]
    
    // [init action servers]
    
    // [init action clients]

    // init MARKERS
    
    // init constraint markers message
    constraints_Marker_msg_.type = visualization_msgs::Marker::LINE_LIST;
    constraints_Marker_msg_.header.frame_id = "map";
    constraints_Marker_msg_.scale.x = 0.1;
    constraints_Marker_msg_.color.r = 1; //yellow
    constraints_Marker_msg_.color.g = 1; //yellow
    constraints_Marker_msg_.color.b = 0; //yellow
    constraints_Marker_msg_.color.a = 1;
    constraints_Marker_msg_.ns = "/constraints";
    constraints_Marker_msg_.id = 1;
    
    //init lidar lines marker array
    line_colors_[0].r = 0; line_colors_[0].g = 0; line_colors_[0].b = 1; line_colors_[0].a = 1;
    line_colors_[1].r = 1; line_colors_[1].g = 0; line_colors_[1].b = 1; line_colors_[1].a = 1;
    line_colors_[2].r = 1; line_colors_[2].g = 0; line_colors_[2].b = 0; line_colors_[2].a = 1;
    line_colors_[3].r = 1; line_colors_[3].g = 1; line_colors_[3].b = 0; line_colors_[3].a = 1;
    line_colors_[4].r = 0; line_colors_[4].g = 1; line_colors_[4].b = 0; line_colors_[4].a = 1;
    line_colors_[5].r = 0; line_colors_[5].g = 1; line_colors_[5].b = 1; line_colors_[5].a = 1;
    visualization_msgs::Marker line_list_Marker_msg;
    for (unsigned int i=0; i<6; i++)
    {
        line_list_Marker_msg.header.stamp = ros::Time::now();
        line_list_Marker_msg.header.frame_id = "agv_base_link";
        line_list_Marker_msg.type = visualization_msgs::Marker::LINE_LIST;
        line_list_Marker_msg.scale.x = 0.1;
        line_list_Marker_msg.color = line_colors_[i];
        line_list_Marker_msg.ns = "/lines";
        line_list_Marker_msg.id = i;
        lines_MarkerArray_msg_.markers.push_back(line_list_Marker_msg);
    }
    
    // Init vehicle_MarkerArray_msg with a first RED CUBE marker representing the vehicle at agv_base_link.
    visualization_msgs::Marker vehicle_head_marker;
    vehicle_head_marker.header.stamp = ros::Time::now();
    vehicle_head_marker.header.frame_id = "agv_base_link";
    vehicle_head_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_head_marker.scale.x = 4.4;
    vehicle_head_marker.scale.y = 1.6;
    vehicle_head_marker.scale.z = 1;
    vehicle_head_marker.color.r = 1; //red
    vehicle_head_marker.color.g = 0;
    vehicle_head_marker.color.b = 0;
    vehicle_head_marker.color.a = 1;
    vehicle_head_marker.pose.position.x = 1.3;
    vehicle_head_marker.pose.position.y = 0.0;
    vehicle_head_marker.pose.position.z = 0.5;
    vehicle_head_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    vehicle_head_marker.id = 0;
    vehicle_MarkerArray_msg_.markers.push_back(vehicle_head_marker);

    // Init the rest of the vehicle_MarkerArray_msg with YELLOW CUBE markers, with neither pose nor stamp
    visualization_msgs::Marker vehicle_trajectory_marker;
    for (unsigned int ii = 0; ii < window_length_; ii++)
    {
        //vehicle_trajectory_marker.header.stamp = ros::Time::now();
        //vehicle_trajectory_marker.header.frame_id = (i==1 ? "agv_base_link" : "map");
        vehicle_trajectory_marker.header.frame_id = "map";
        vehicle_trajectory_marker.type = visualization_msgs::Marker::CUBE;
        vehicle_trajectory_marker.scale.x = 4.4;
        vehicle_trajectory_marker.scale.y = 1.6;
        vehicle_trajectory_marker.scale.z = 1;
        vehicle_trajectory_marker.color.r = 1;//yellow
        vehicle_trajectory_marker.color.g = 1;//yellow
        vehicle_trajectory_marker.color.b = 0;
        //vehicle_trajectory_marker.color.a = (i==1 ? 1 : 0.0);
        vehicle_trajectory_marker.color.a = 0.0; //no show at while no true frame
        vehicle_trajectory_marker.pose.position.x = 1.3;
        vehicle_trajectory_marker.pose.position.y = 0.0;
        vehicle_trajectory_marker.pose.position.z = 0.5;
//         vehicle_trajectory_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        vehicle_trajectory_marker.id = ii+1; //0 already taken by the current vehicle
        vehicle_MarkerArray_msg_.markers.push_back(vehicle_trajectory_marker);
    }

// std::cout << "End of constructor: " << __LINE__ << std::endl;
    ROS_INFO("STARTING IRI WOLF...");
}

WolfAlgNode::~WolfAlgNode(void)
{
    //delete allocated memory in reverse order of dependencies
    delete wolf_manager_;
    delete odom_sensor_ptr_;
    for (uint ii = 0; ii<6; ii++)
    {
        delete laser_sensor_ptr_[ii];
        delete laser_sensor_point_[ii];
        //delete laser_sensor_theta_[ii];
        delete laser_sensor_orientation_[ii];
    }
        
    // [free dynamic memory]
    pthread_mutex_destroy(&this->relative_odometry_mutex_);
    for (unsigned int ii=0; ii<6; ii++) pthread_mutex_destroy(&laser_mutexes_[ii]);
}

void WolfAlgNode::mainNodeThread(void)
{
    //std::cout << "mainNodeThread(): " << __LINE__ << std::endl;
    
    //lock everyone
    relative_odometry_mutex_enter();
    for (unsigned int ii=0; ii<6; ii++) laser_mutex_enter(ii);

    //solve problem
    wolf_manager_->update();
    ceres_manager_->update(wolf_manager_->getProblemPtr());
    ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);

    //unlock everyone
    relative_odometry_mutex_exit();    
    for (unsigned int ii=0; ii<6; ii++) laser_mutex_exit(ii);

    // Sets localization timestamp & Gets wolf localization estimate
    ros::Time loc_stamp = ros::Time::now();
    Eigen::VectorXs vehicle_pose  = wolf_manager_->getVehiclePose();

    // Broadcast transform ---------------------------------------------------------------------------
    //Get map2base from Wolf result, and builds base2map pose
    tf::Pose map2base;
    map2base.setOrigin( tf::Vector3(vehicle_pose(0), vehicle_pose(1), 0) ); 
    map2base.setRotation( tf::createQuaternionFromYaw(vehicle_pose(2)) );
    
    std::cout << "Loc: (" << vehicle_pose(0) << "," << vehicle_pose(1) << "," << vehicle_pose(2) << ")" << std::endl;
    
    //base2map: invert map2base to get base2map (map wrt base), and stamp it
    tf::Stamped<tf::Pose> base2map(map2base.inverse(), loc_stamp, "agv_base_link");
    
    //gets odom2map (map wrt odom), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::Stamped<tf::Pose> odom2map; 
    if ( tfl_.waitForTransform("odom", "agv_base_link", loc_stamp, ros::Duration(0.1)) )
    {
        //gets odom2map
        tfl_.transformPose("odom", base2map, odom2map);
    
        //invert odom2map to get map2odom, and stamp it
        //tf::Stamped<tf::Pose> map2odom(odom2map.inverse(), loc_stamp, "map"); 
    
        //broadcast map2odom = odom2map.inverse()
        //tfb_.sendTransform( tf::StampedTransform(map2odom, loc_stamp, "map", "odom") );
        tfb_.sendTransform( tf::StampedTransform(odom2map.inverse(), loc_stamp, "map", "odom") );
        
    }
    //End Broadcast transform -----------------------------------------------------------------------------
    
    // [fill msg structures]

    // MARKERS VEHICLE & CONSTRAINTS
    ConstraintBaseList ctr_list;
    constraints_Marker_msg_.points.clear();
    constraints_Marker_msg_.header.stamp = loc_stamp;
    unsigned int ii = 1; //start to 1 to do not modify vehicle_MarkerArray_msg_.marker[0], since it is already at agv_base_link
    geometry_msgs::Point point1, point2;
    for (auto fr_it = wolf_manager_->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->rbegin(); 
         fr_it != wolf_manager_->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->rend() && ii <= window_length_+1; 
         fr_it++, ii++) //runs the list of frames in reverse order, from the head (current) to the tail (former)
    {
        // VEHICLES
        vehicle_MarkerArray_msg_.markers[ii].action = visualization_msgs::Marker::MODIFY;
        vehicle_MarkerArray_msg_.markers[ii].header.stamp = loc_stamp;
        vehicle_MarkerArray_msg_.markers[ii].header.frame_id = "map";
        vehicle_MarkerArray_msg_.markers[ii].pose.position.x = *((*fr_it)->getPPtr()->getPtr())+1.3*cos( *((*fr_it)->getOPtr()->getPtr()) );
        vehicle_MarkerArray_msg_.markers[ii].pose.position.y = *((*fr_it)->getPPtr()->getPtr()+1)+1.3*sin( *((*fr_it)->getOPtr()->getPtr()) );
        vehicle_MarkerArray_msg_.markers[ii].pose.orientation = tf::createQuaternionMsgFromYaw( *((*fr_it)->getOPtr()->getPtr()) );
        vehicle_MarkerArray_msg_.markers[ii].color.a = 0.5; //Show with little transparency
        
        // CONSTRAINTS (odometry)
        point1.x = *(*fr_it)->getPPtr()->getPtr();
        point1.y = *((*fr_it)->getPPtr()->getPtr()+1);
        point1.z = 0.25;
        if (ii == 1)
        {
            point2.x = map2base.getOrigin().x();
            point2.y = map2base.getOrigin().y();
            point2.z = 0.25;
        }
        else
        {
            auto next_frame = fr_it; //inverse iterator
            next_frame--;
            point2.x = *(*next_frame)->getPPtr()->getPtr();
            point2.y = *((*next_frame)->getPPtr()->getPtr()+1);
            point2.z = 0.25;
        }
        constraints_Marker_msg_.points.push_back(point1);
        constraints_Marker_msg_.points.push_back(point2);

        // CONSTRAINTS (landmarks)
        ctr_list.clear();
        (*fr_it)->getConstraintList(ctr_list);
        for (auto c_it = ctr_list.begin(); c_it != ctr_list.end(); c_it++)
        {
            if ((*c_it)->getConstraintType() == CTR_CORNER_2D_THETA)
            {
                point1.x = *(*fr_it)->getPPtr()->getPtr();
                point1.y = *((*fr_it)->getPPtr()->getPtr()+1);
                point1.z = 0.25;
                point2.x = *((ConstraintCorner2DTheta*)(*c_it))->getLandmarkPtr()->getPPtr()->getPtr();
                point2.y = *(((ConstraintCorner2DTheta*)(*c_it))->getLandmarkPtr()->getPPtr()->getPtr()+1);
                point2.z = 1.5;
                constraints_Marker_msg_.points.push_back(point1);
                constraints_Marker_msg_.points.push_back(point2);
            }
        }
    }
    
    // MARKERS LANDMARKS
    ii = 0;
    corners_MarkerArray_msg_.markers.clear();
    visualization_msgs::Marker new_corner;
    for (auto l_it = wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); 
         l_it != wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); 
         l_it++, ii++)
    {
        new_corner.header.stamp = loc_stamp;
        new_corner.header.frame_id = "map";
        new_corner.type = visualization_msgs::Marker::CUBE;
        new_corner.color.r = (double)(*l_it)->getHits()/10;
        new_corner.color.g = 0;
        new_corner.color.b = 1 - (double)(*l_it)->getHits()/10;
        new_corner.color.a = 1;//0.3 + 0.7*((double)(*l_it)->getHits()/10);
        new_corner.ns = "/corners";
        new_corner.id = ii;

        new_corner.pose.position.x = *(*l_it)->getPPtr()->getPtr();
        new_corner.pose.position.y = *((*l_it)->getPPtr()->getPtr()+1);
        new_corner.pose.position.z = 1.5;
        new_corner.pose.orientation = tf::createQuaternionMsgFromYaw(*(*l_it)->getOPtr()->getPtr());

        new_corner.scale.x = 0.5;
        new_corner.scale.y = 0.5;
        new_corner.scale.z = 3;
        corners_MarkerArray_msg_.markers.push_back(new_corner);
    }
    //ROS_INFO("WolfAlgNode: %i Landmarks ", corners_MarkerArray_msg_.markers.size());
  
    // [fill srv structure and make request to the server]
    
    // [fill action structure and make request to the action server]

    // [publish messages]
    vehicle_publisher_.publish(this->vehicle_MarkerArray_msg_);
    corners_publisher_.publish(this->corners_MarkerArray_msg_);
    lines_publisher_.publish(this->lines_MarkerArray_msg_);
    constraints_publisher_.publish(this->constraints_Marker_msg_);

}

/*  [subscriber callbacks] */

void WolfAlgNode::relative_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::relative_odometry_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->relative_odometry_mutex_enter();
  wolf_manager_->addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                              odom_sensor_ptr_,
                                              Eigen::Vector3s(msg->pose.pose.position.x,0.0,tf::getYaw(msg->pose.pose.orientation))));

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->relative_odometry_mutex_exit();
}

void WolfAlgNode::relative_odometry_mutex_enter(void)
{
  pthread_mutex_lock(&this->relative_odometry_mutex_);
}

void WolfAlgNode::relative_odometry_mutex_exit(void)
{
  pthread_mutex_unlock(&this->relative_odometry_mutex_);
}

void WolfAlgNode::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::laser_front_left_callback: New Message Received");
    
    //get the id from the message header 
    const char lidar_id_c = msg->header.frame_id.back(); 
    unsigned int lidar_id = atoi(&lidar_id_c); 
    //std::cout << "laser_callback() starts. lidar_id: " << msg->header.frame_id << ", " << lidar_id_c << ", " << lidar_id << std::endl;
    
    if ( laser_sensor_ptr_[lidar_id] != nullptr ) //checks that the sensor exists
    {
        //updates laser scan params in case they are not set yet
        updateLaserParams(lidar_id, msg);

        //lock appropiate mutex to shared variables if necessary
        laser_mutex_enter(lidar_id);
        
        //create a new capture in the Wolf environment.
        CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                        laser_sensor_ptr_[lidar_id], msg->ranges); 
        wolf_manager_->addCapture(new_capture);
        
        //just compute lines for viuslization purposes. TODO: To be removed from here and get lines from visualization somewhere from wolf objects
        //computeLaserScan(new_capture, msg->header, lidar_id);

        //std::cout << msg->data << std::endl;
        //unlock previously blocked shared variables
        //this->alg_.unlock();
        this->laser_mutex_exit(lidar_id);
    }
}

void WolfAlgNode::laser_mutex_enter(unsigned int _id)
{
    pthread_mutex_lock(&this->laser_mutexes_[_id]);
}

void WolfAlgNode::laser_mutex_exit(unsigned int _id)
{
    pthread_mutex_unlock(&this->laser_mutexes_[_id]);
}

/*
void WolfAlgNode::laser_1_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::laser_front_callback: New Message Received");
    updateLaserParams(0, msg);

    //use appropiate mutex to shared variables if necessary
    //this->alg_.lock();
    //this->laser_1_mutex_enter();
    //  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
    //                                                   laser_sensor_ptr_[0],
    //                                                   msg->ranges);
    //  wolf_manager_->addCapture(new_capture);
    //  computeLaserScan(new_capture, msg->header, 0);

    //unlock previously blocked shared variables
    //this->alg_.unlock();
    //this->laser_1_mutex_exit();
}

void WolfAlgNode::laser_1_mutex_enter(void)
{
    pthread_mutex_lock(&this->laser_1_mutex_);
}

void WolfAlgNode::laser_1_mutex_exit(void)
{
    pthread_mutex_unlock(&this->laser_1_mutex_);
}

void WolfAlgNode::laser_2_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::laser_front_left_callback: New Message Received");
    updateLaserParams(1, msg);

    //use appropiate mutex to shared variables if necessary
    //this->alg_.lock();
    this->laser_2_mutex_enter();
    CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                     laser_sensor_ptr_[1], 
                                                     msg->ranges);
    wolf_manager_->addCapture(new_capture);
    computeLaserScan(new_capture, msg->header, 1);

    //std::cout << msg->data << std::endl;
    //unlock previously blocked shared variables
    //this->alg_.unlock();
    this->laser_2_mutex_exit();
}

void WolfAlgNode::laser_2_mutex_enter(void)
{
    pthread_mutex_lock(&this->laser_2_mutex_);
}

void WolfAlgNode::laser_2_mutex_exit(void)
{
    pthread_mutex_unlock(&this->laser_2_mutex_);
}

void WolfAlgNode::laser_3_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_back_left_callback: New Message Received");
  updateLaserParams(2, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->laser_3_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                   laser_sensor_ptr_[2],
                                                   msg->ranges);
  wolf_manager_->addCapture(new_capture);
  computeLaserScan(new_capture, msg->header, 2);

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->laser_3_mutex_exit();
}

void WolfAlgNode::laser_3_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_3_mutex_);
}

void WolfAlgNode::laser_3_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_3_mutex_);
}

void WolfAlgNode::laser_4_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_back_callback: New Message Received");
  updateLaserParams(3, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->laser_4_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                   laser_sensor_ptr_[3],
                                                   msg->ranges);
  wolf_manager_->addCapture(new_capture);
  computeLaserScan(new_capture, msg->header, 3);

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->laser_4_mutex_exit();
}

void WolfAlgNode::laser_4_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_4_mutex_);
}

void WolfAlgNode::laser_4_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_4_mutex_);
}

void WolfAlgNode::laser_5_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_back_right_callback: New Message Received");
  updateLaserParams(4, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->laser_5_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                   laser_sensor_ptr_[4],
                                                   msg->ranges);
  wolf_manager_->addCapture(new_capture);
  computeLaserScan(new_capture, msg->header, 4);

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->laser_5_mutex_exit();
}

void WolfAlgNode::laser_5_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_5_mutex_);
}

void WolfAlgNode::laser_5_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_5_mutex_);
}

void WolfAlgNode::laser_6_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_front_right_callback: New Message Received");
  updateLaserParams(5, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->laser_6_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                   laser_sensor_ptr_[5],
                                                   msg->ranges);
  wolf_manager_->addCapture(new_capture);
  computeLaserScan(new_capture, msg->header, 5);

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->laser_6_mutex_exit();
}

void WolfAlgNode::laser_6_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_6_mutex_);
}

void WolfAlgNode::laser_6_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_6_mutex_);
}
*/


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WolfAlgNode::node_config_update(Config &config, uint32_t level)
{
    this->alg_.lock();
    for (unsigned int ii = 0; ii < 6; ii++)
    {
        if ( laser_sensor_ptr_[ii] != nullptr ) //checks that the sensor exists
        {
            laserscanutils::ExtractCornerParams corners_alg_params = laser_sensor_ptr_[ii]->getCornerAlgParams();
            corners_alg_params.theta_min_ = config.theta_min;
            corners_alg_params.max_distance_ = config.max_distance;
            corners_alg_params.line_params_.jump_dist_ut_ = config.max_distance; //TODO: To be added as a new param at .cfg file
            corners_alg_params.line_params_.window_sz_ = config.segment_window_size; //TODO: change name at .cfg file
            corners_alg_params.line_params_.k_sigmas_ut_ = config.k_sigmas;//TODO: change name at .cfg file
            corners_alg_params.line_params_.concatenate_ii_ut_ = config.max_beam_distance;//TODO: change name at .cfg file
            corners_alg_params.line_params_.concatenate_angle_ut_ = config.theta_max_parallel;//TODO: change name at .cfg file
            laser_sensor_ptr_[ii]->setCornerAlgParams(corners_alg_params);//TODO: change name at .cfg file
        }
    }
    draw_lines_ = config.draw_lines;
    new_frame_elapsed_time_ = config.new_frame_elapsed_time;
    wolf_manager_->setNewFrameElapsedTime(new_frame_elapsed_time_);
    this->alg_.unlock();
}

void WolfAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<WolfAlgNode>(argc, argv, "wolf_alg_node");
}

void WolfAlgNode::updateLaserParams(const unsigned int _laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!laser_params_set_[_laser_idx])
  {
    // laser intrinsic parameters
    laserscanutils::ScanParams params = laser_sensor_ptr_[_laser_idx]->getScanParams();
    params.angle_min_ = msg->angle_min;
    params.angle_max_ = msg->angle_max;
    params.angle_step_ = msg->angle_increment;
    params.scan_time_ = msg->time_increment;
    params.range_min_ = 0.5;//msg->range_min;
    params.range_max_ = msg->range_max;
    params.range_std_dev_ = 0.05;
    laser_sensor_ptr_[_laser_idx]->setScanParams(params);
    laser_params_set_[_laser_idx] = true;
  }
}

void WolfAlgNode::computeLaserScan(CaptureLaser2D* new_capture, const std_msgs::Header & header, const unsigned int laser_idx)
{
  if (draw_lines_)
  {
    std::list<laserscanutils::Line> line_list;
    new_capture->extractLines(line_list);

    lines_MarkerArray_msg_.markers[laser_idx].points.clear();
    lines_MarkerArray_msg_.markers[laser_idx].header.stamp = header.stamp;
    lines_MarkerArray_msg_.markers[laser_idx].header.frame_id = header.frame_id;

    for (auto line_it = line_list.begin(); line_it != line_list.end(); line_it++ )
    {
      geometry_msgs::Point point1, point2;
      point1.x = line_it->point_first_(0);
      point1.y = line_it->point_first_(1);
      point1.z = 1.5;
      point2.x = line_it->point_last_(0);
      point2.y = line_it->point_last_(1);
      point2.z = 1.5;
      lines_MarkerArray_msg_.markers[laser_idx].points.push_back(point1);
      lines_MarkerArray_msg_.markers[laser_idx].points.push_back(point2);
    }
  }
}


