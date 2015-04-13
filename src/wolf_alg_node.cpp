#include "wolf_alg_node.h"

WolfAlgNode::WolfAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<WolfAlgorithm>(),
    odom_sensor_pose_(3),
    odom_sensor_point_(odom_sensor_pose_.data()),
    odom_sensor_theta_(&odom_sensor_pose_(2)),
    laser_params_setted_({false, false, false, false, false, false}),
    laser_sensor_ptrs_(6),
    lines_colors_(6),
    draw_lines_(false)
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
  odom_sensor_pose_ = Eigen::Vector3s::Zero(); //odom sensor coniciding with vehicle base link  
  odom_sensor_ = new SensorOdom2D(&odom_sensor_point_, &odom_sensor_theta_, odom_std[0], odom_std[1]);
  
  // init wolf laser sensor
  //std::vector<Eigen::Vector6s> laser_poses(6);
  laser_poses[0] << 3.5,0,0,0,0,0;
  laser_poses[1] << 3.1,-0.78,0,0,0,-1.65806;//3.1 -0.78 0.5 yaw:-1.65806
  laser_poses[2] << -0.5,-0.8,0,0,0,-1.46608;//-0.5 -0.8  0.5  yaw:-1.46608
  laser_poses[3] << -0.9,0,0,0,0,-3.12414;//-0.9  0,   0.45 yaw:-3.12414
  laser_poses[4] << -0.5,0.8,0,0,0,1.48353;//-0.5 -0.8  0.5  yaw:1.48353
  laser_poses[5] << 3.1,0.8,0,0,0,1.64388;//3.1  0.8  0.55 yaw:1.64388
  for (uint i = 0; i<6; i++)
    laser_sensor_ptrs_[i] = new SensorLaser2D(laser_poses[i]);
  wolf_manager_ = new WolfManager(odom_sensor_, false, state_initial_length, Eigen::Vector3s::Zero(), new_frame_elapsed_time_, window_length_);

  // [init publishers]
  this->lines_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("lines", 1);
  this->constraints_publisher_ = this->public_node_handle_.advertise<visualization_msgs::Marker>("constraints", 1);
  this->corners_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("corners", 1);
  this->vehicle_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("vehicle", 1);
  
  // TF
  T_localization_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0, 0.0));
  tfb_.sendTransform( tf::StampedTransform(T_localization_, ros::Time::now(), "odom", "base"));

  // [init subscribers]
  this->laser_1_subscriber_ = this->public_node_handle_.subscribe("laser_1", 1, &WolfAlgNode::laser_1_callback, this);
  pthread_mutex_init(&this->laser_1_mutex_,NULL);

  this->laser_2_subscriber_ = this->public_node_handle_.subscribe("laser_2", 1, &WolfAlgNode::laser_2_callback, this);
  pthread_mutex_init(&this->laser_2_mutex_,NULL);

  this->laser_3_subscriber_ = this->public_node_handle_.subscribe("laser_3", 1, &WolfAlgNode::laser_3_callback, this);
  pthread_mutex_init(&this->laser_3_mutex_,NULL);

  this->laser_4_subscriber_ = this->public_node_handle_.subscribe("laser_4", 1, &WolfAlgNode::laser_4_callback, this);
  pthread_mutex_init(&this->laser_4_mutex_,NULL);

  this->laser_5_subscriber_ = this->public_node_handle_.subscribe("laser_5", 1, &WolfAlgNode::laser_5_callback, this);
  pthread_mutex_init(&this->laser_5_mutex_,NULL);

  this->laser_6_subscriber_ = this->public_node_handle_.subscribe("laser_6", 1, &WolfAlgNode::laser_6_callback, this);
  pthread_mutex_init(&this->laser_6_mutex_,NULL);

  this->relative_odometry_subscriber_ = this->public_node_handle_.subscribe("relative_odometry", 10, &WolfAlgNode::relative_odometry_callback, this);
  pthread_mutex_init(&this->relative_odometry_mutex_,NULL);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]

  // init lines marker array
  lines_colors_[0].r = 0;
  lines_colors_[0].g = 0;
  lines_colors_[0].b = 1;
  lines_colors_[0].a = 1;
  lines_colors_[1].r = 1;
  lines_colors_[1].g = 0;
  lines_colors_[1].b = 1;
  lines_colors_[1].a = 1;
  lines_colors_[2].r = 1;
  lines_colors_[2].g = 0;
  lines_colors_[2].b = 0;
  lines_colors_[2].a = 1;
  lines_colors_[3].r = 1;
  lines_colors_[3].g = 1;
  lines_colors_[3].b = 0;
  lines_colors_[3].a = 1;
  lines_colors_[4].r = 0;
  lines_colors_[4].g = 1;
  lines_colors_[4].b = 0;
  lines_colors_[4].a = 1;
  lines_colors_[5].r = 0;
  lines_colors_[5].g = 1;
  lines_colors_[5].b = 1;
  lines_colors_[5].a = 1;
  for (unsigned int i=0; i<6; i++)
  {
    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color = lines_colors_[i];
    line_list.ns = "/lines";
    line_list.id = i;

    lines_MarkerArray_msg_.markers.push_back(line_list);
  }
  // odom vehicle
  visualization_msgs::Marker odom_vehicle_Marker_msg;
  odom_vehicle_Marker_msg.header.stamp = ros::Time::now();
  odom_vehicle_Marker_msg.header.frame_id = "/odom";
  odom_vehicle_Marker_msg.type = visualization_msgs::Marker::CUBE;
  odom_vehicle_Marker_msg.scale.x = 4.4;
  odom_vehicle_Marker_msg.scale.y = 1.6;
  odom_vehicle_Marker_msg.scale.z = 1;
  odom_vehicle_Marker_msg.color.r = 1;
  odom_vehicle_Marker_msg.color.g = 0;
  odom_vehicle_Marker_msg.color.b = 0;
  odom_vehicle_Marker_msg.color.a = 0;
  odom_vehicle_Marker_msg.pose.position.x = 1.3;
  odom_vehicle_Marker_msg.pose.position.y = 0.0;
  odom_vehicle_Marker_msg.pose.position.z = 0.5;
  odom_vehicle_Marker_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  odom_vehicle_Marker_msg.id = 0;
  vehicle_MarkerArray_msg_.markers.push_back(odom_vehicle_Marker_msg);

  for (unsigned int i = 1; i <= window_length_+1; i++)
  {
    visualization_msgs::Marker window_frame;
    window_frame.header.stamp = ros::Time::now();
    window_frame.header.frame_id = (i==1 ? "/base" : "/map");
    window_frame.type = visualization_msgs::Marker::CUBE;
    window_frame.scale.x = 4.4;
    window_frame.scale.y = 1.6;
    window_frame.scale.z = 1;
    window_frame.color.r = 1;
    window_frame.color.g = 1;
    window_frame.color.b = 0;
    window_frame.color.a = (i==1 ? 1 : 0.0);
    window_frame.pose.position.x = 1.3;
    window_frame.pose.position.y = 0.0;
    window_frame.pose.position.z = 0.5;
    window_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    window_frame.id = i;
    vehicle_MarkerArray_msg_.markers.push_back(window_frame);
  }

  // constraints
  constraints_Marker_msg_.type = visualization_msgs::Marker::LINE_LIST;
  constraints_Marker_msg_.header.stamp = ros::Time::now();
  constraints_Marker_msg_.header.frame_id = "/map";
  constraints_Marker_msg_.scale.x = 0.1;
  constraints_Marker_msg_.color.r = 1;
  constraints_Marker_msg_.color.g = 1;
  constraints_Marker_msg_.color.b = 0;
  constraints_Marker_msg_.color.a = 1;
  constraints_Marker_msg_.ns = "/constraints";

  ROS_INFO("START...");
}

WolfAlgNode::~WolfAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->laser_3_mutex_);
  pthread_mutex_destroy(&this->laser_5_mutex_);
  pthread_mutex_destroy(&this->laser_6_mutex_);
  pthread_mutex_destroy(&this->laser_2_mutex_);
  pthread_mutex_destroy(&this->relative_odometry_mutex_);
  pthread_mutex_destroy(&this->laser_4_mutex_);
  pthread_mutex_destroy(&this->laser_1_mutex_);
}

void WolfAlgNode::mainNodeThread(void)
{
  wolf_manager_->update();
  ceres_manager_->update(wolf_manager_->getProblemPtr());
  ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);

  // tf
  Eigen::VectorXs vehicle_pose  = wolf_manager_->getVehiclePose();
//  std::cout << "Get vehicle pose: "<< vehicle_pose.transpose() << std::endl;
  geometry_msgs::Pose current_pose;
  current_pose.position.x = vehicle_pose(0);
  current_pose.position.y = vehicle_pose(1);
  current_pose.position.z = 0;
  current_pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_pose(2));
  tf::poseMsgToTF(current_pose, T_localization_);

  // [fill msg structures]

  // WINDOW OF FRAMES
  ConstraintBaseList ctr_list;
  constraints_Marker_msg_.points.clear();
  unsigned int i = 2;
  for (auto fr_it = wolf_manager_->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->rbegin(); fr_it != wolf_manager_->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()->rend(); fr_it++)
  {
    // FRAME
    vehicle_MarkerArray_msg_.markers[i].action = visualization_msgs::Marker::MODIFY;
    vehicle_MarkerArray_msg_.markers[i].pose.position.x = *(*fr_it)->getPPtr()->getPtr()+1.3*cos(*(*fr_it)->getOPtr()->getPtr());
    vehicle_MarkerArray_msg_.markers[i].pose.position.y = *((*fr_it)->getPPtr()->getPtr()+1)+1.3*sin(*(*fr_it)->getOPtr()->getPtr());
    vehicle_MarkerArray_msg_.markers[i].pose.orientation = tf::createQuaternionMsgFromYaw(*(*fr_it)->getOPtr()->getPtr());
    vehicle_MarkerArray_msg_.markers[i].color.a = 0.5;

    // CONSTRAINTS (odometry)
    geometry_msgs::Point point1, point2;
    point1.x = *(*fr_it)->getPPtr()->getPtr();
    point1.y = *((*fr_it)->getPPtr()->getPtr()+1);
    point1.z = 0.25;
    if (i == 2)
    {
      point2.x = current_pose.position.x;
      point2.y = current_pose.position.y;
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
        point2.z = 0.25;
        point2.x = *((ConstraintCorner2DTheta*)(*c_it))->getLandmarkPtr()->getPPtr()->getPtr();
        point2.y = *(((ConstraintCorner2DTheta*)(*c_it))->getLandmarkPtr()->getPPtr()->getPtr()+1);
        point2.z = 1.5;
        constraints_Marker_msg_.points.push_back(point1);
        constraints_Marker_msg_.points.push_back(point2);
      }
    }
    i++;
    if (i > window_length_+1)
      break;
  }
  // LANDMARKS
  i = 0;
  corners_MarkerArray_msg_.markers.clear();
  for (auto l_it = wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); l_it != wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); l_it++)
  {
    visualization_msgs::Marker new_corner;
    new_corner.header.stamp = ros::Time::now();
    new_corner.header.frame_id = "/map";
    new_corner.type = visualization_msgs::Marker::CUBE;
    new_corner.color.r = (double)(*l_it)->getHits()/10;
    new_corner.color.g = 0;
    new_corner.color.b = 1 - (double)(*l_it)->getHits()/10;
    new_corner.color.a = 1;//0.3 + 0.7*((double)(*l_it)->getHits()/10);
    new_corner.ns = "/corners";
    new_corner.id = i;

    new_corner.pose.position.x = *(*l_it)->getPPtr()->getPtr();
    new_corner.pose.position.y = *((*l_it)->getPPtr()->getPtr()+1);
    new_corner.pose.position.z = 1.5;
    new_corner.pose.orientation = tf::createQuaternionMsgFromYaw(*(*l_it)->getOPtr()->getPtr());

    new_corner.scale.x = 0.5;
    new_corner.scale.y = 0.5;
    new_corner.scale.z = 3;
    corners_MarkerArray_msg_.markers.push_back(new_corner);

    i++;
  }
  //ROS_INFO("WolfAlgNode: %i Landmarks ", corners_MarkerArray_msg_.markers.size());
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  tfb_.sendTransform( tf::StampedTransform(T_localization_, ros::Time::now(), "map", "base") );
  vehicle_publisher_.publish(this->vehicle_MarkerArray_msg_);
  corners_publisher_.publish(this->corners_MarkerArray_msg_);
  lines_publisher_.publish(this->lines_MarkerArray_msg_);
  constraints_publisher_.publish(this->constraints_Marker_msg_);

}

void WolfAlgNode::laser_1_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_front_callback: New Message Received");
  updateLaserParams(0, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->laser_1_mutex_enter();
//  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
//                                                   laser_sensor_ptrs_[0],
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
                                                   laser_sensor_ptrs_[1],
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

/*  [subscriber callbacks] */
void WolfAlgNode::laser_3_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_back_left_callback: New Message Received");
  updateLaserParams(2, msg);

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->laser_3_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                   laser_sensor_ptrs_[2],
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
                                                   laser_sensor_ptrs_[3],
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
                                                   laser_sensor_ptrs_[4],
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
                                                   laser_sensor_ptrs_[5],
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

void WolfAlgNode::relative_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::relative_odometry_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->relative_odometry_mutex_enter();
  wolf_manager_->addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                              odom_sensor_,
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


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WolfAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  for (unsigned int laser_idx = 0; laser_idx < 6;laser_idx++)
  {
    laserscanutils::ExtractCornerParams corners_alg_params = laser_sensor_ptrs_[laser_idx]->getCornerAlgParams();
    corners_alg_params.segment_window_size = config.segment_window_size;
    corners_alg_params.theta_min = config.theta_min;
    corners_alg_params.theta_max_parallel = config.theta_max_parallel;
    corners_alg_params.k_sigmas = config.k_sigmas;
    corners_alg_params.max_beam_distance = config.max_beam_distance;
    corners_alg_params.max_distance = config.max_distance;
    laser_sensor_ptrs_[laser_idx]->setCornerAlgParams(corners_alg_params);
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

void WolfAlgNode::updateLaserParams(const unsigned int laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!laser_params_setted_[laser_idx])
  {
    // laser intrinsic parameters
    laserscanutils::ScanParams params = laser_sensor_ptrs_[laser_idx]->getScanParams();
    params.angle_min_ = msg->angle_min;
    params.angle_max_ = msg->angle_max;
    params.angle_step_ = msg->angle_increment;
    params.scan_time_ = msg->time_increment;
    params.range_min_ = 0.5;//msg->range_min;
    params.range_max_ = msg->range_max;
    params.range_std_dev_ = 0.05;
    laser_sensor_ptrs_[laser_idx]->setScanParams(params);
    laser_params_setted_[laser_idx] = true;
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

