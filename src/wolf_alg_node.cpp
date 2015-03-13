#include "wolf_alg_node.h"

WolfAlgNode::WolfAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<WolfAlgorithm>(),
  laser_front_params_setted_(false),
  laser_back_params_setted_(false)
{
  //init class attributes if necessary
  WolfScalar odom_std[2], new_frame_elapsed_time;
  int window_length, state_initial_length;
  public_node_handle_.param<double>("odometry_tranlational_std", odom_std[0], 0.01);
  public_node_handle_.param<double>("odometry_rotational_std", odom_std[1], 0.01);
  public_node_handle_.param<int>("window_lenght", window_length, 10);
  public_node_handle_.param<int>("state_initial_lenght", state_initial_length, 1e9);
  public_node_handle_.param<double>("new_frame_elapsed_time", new_frame_elapsed_time, 0.1);

  this->loop_rate_ = 10;//in [Hz]
  // init ceres
  ceres_options_.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION;
  ceres_options_.max_line_search_step_contraction = 1e-3;
  problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_manager_ = new CeresManager(problem_options_);

  // init wolf
  odom_sensor_ = new SensorOdom2D(Eigen::Vector6s::Zero(), odom_std[0], odom_std[1]);
  Eigen::Vector6s laser_front_pose, laser_back_pose;
  laser_front_pose << 3.5,0,0,0,0,0;
  laser_back_pose << -0.9,0,0,0,0,-3.12414;
  laser_front_sensor_ = new SensorLaser2D(laser_front_pose);
  laser_back_sensor_ = new SensorLaser2D(laser_back_pose);
  wolf_manager_ = new WolfManager(odom_sensor_, false, state_initial_length, Eigen::Vector3s::Zero(), new_frame_elapsed_time, window_length);

  // [init publishers]
  this->corners_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("corners", 1);
  
  // [init subscribers]
  this->relative_odometry_subscriber_ = this->public_node_handle_.subscribe("relative_odometry", 10, &WolfAlgNode::relative_odometry_callback, this);
  pthread_mutex_init(&this->relative_odometry_mutex_,NULL);

  this->laser_back_subscriber_ = this->public_node_handle_.subscribe("laser_back", 10, &WolfAlgNode::laser_back_callback, this);
  pthread_mutex_init(&this->laser_back_mutex_,NULL);

  this->laser_front_subscriber_ = this->public_node_handle_.subscribe("laser_front", 10, &WolfAlgNode::laser_front_callback, this);
  pthread_mutex_init(&this->laser_front_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

WolfAlgNode::~WolfAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->relative_odometry_mutex_);
  pthread_mutex_destroy(&this->laser_back_mutex_);
  pthread_mutex_destroy(&this->laser_front_mutex_);
}

void WolfAlgNode::mainNodeThread(void)
{
  wolf_manager_->update();
  ceres_manager_->update(wolf_manager_->getProblemPtr());
  ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);

  // [fill msg structures]

  unsigned int i = 0;
  for (auto l_it = wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->begin(); l_it != wolf_manager_->getProblemPtr()->getMapPtr()->getLandmarkListPtr()->end(); l_it++)
  {
    if (corners_MarkerArray_msg_.markers.size() <= i)
    {
      visualization_msgs::Marker new_corner;
      new_corner.header.stamp = ros::Time::now();
      new_corner.header.frame_id = "/map";
      new_corner.type = visualization_msgs::Marker::CUBE;
      new_corner.color.r = 1;
      new_corner.color.g = 0;
      new_corner.color.b = 0;
      new_corner.color.a = 1;
      new_corner.ns = "/corners";
      new_corner.id = corners_MarkerArray_msg_.markers.back().id+1;

      new_corner.pose.position.x = *(*l_it)->getPPtr()->getPtr();
      new_corner.pose.position.y = *((*l_it)->getPPtr()->getPtr()+1);
      new_corner.pose.position.z = 1.5;
      new_corner.pose.orientation = tf::createQuaternionMsgFromYaw(*(*l_it)->getOPtr()->getPtr());

      new_corner.scale.x = 1;
      new_corner.scale.y = 1;
      new_corner.scale.z = 3;
      corners_MarkerArray_msg_.markers.push_back(new_corner);
      ROS_INFO("WolfAlgNode: New landmark!");

    }
    else
    {
      corners_MarkerArray_msg_.markers[i].action = visualization_msgs::Marker::MODIFY;
      corners_MarkerArray_msg_.markers[i].pose.position.x = *(*l_it)->getPPtr()->getPtr();
      corners_MarkerArray_msg_.markers[i].pose.position.y = *((*l_it)->getPPtr()->getPtr()+1);
      corners_MarkerArray_msg_.markers[i].pose.position.z = 1.5;
      corners_MarkerArray_msg_.markers[i].pose.orientation = tf::createQuaternionMsgFromYaw(*(*l_it)->getOPtr()->getPtr());
    }
    i++;
  }
  ROS_INFO("WolfAlgNode: %i Landmarks ", corners_MarkerArray_msg_.markers.size());
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  corners_publisher_.publish(this->corners_MarkerArray_msg_);

}

/*  [subscriber callbacks] */
void WolfAlgNode::relative_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::relative_odometry_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->relative_odometry_mutex_enter();
  wolf_manager_->addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                              odom_sensor_,
                                              Eigen::Vector2s(msg->pose.pose.position.x,tf::getYaw(msg->pose.pose.orientation))));
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->relative_odometry_mutex_exit();
}

void WolfAlgNode::relative_odometry_mutex_enter(void)
{
  pthread_mutex_lock(&this->relative_odometry_mutex_);
}

void WolfAlgNode::relative_odometry_mutex_exit(void)
{
  pthread_mutex_unlock(&this->relative_odometry_mutex_);
}

void WolfAlgNode::laser_back_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_back_callback: New Message Received");

  if (!laser_back_params_setted_)
  {
    laserscanutils::ScanParams params = laser_back_sensor_->getScanParams();
    params.angle_min_ = msg->angle_min;
    params.angle_max_ = msg->angle_max;
    params.angle_step_ = msg->angle_increment;
    params.scan_time_ = msg->time_increment;
    params.range_min_ = msg->range_min;
    params.range_max_ = msg->range_max;
    laser_back_sensor_->setScanParams(params);
  }

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->laser_back_mutex_enter();
  CaptureLaser2D* new_capture = new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                              laser_back_sensor_,
                                                              msg->ranges);
  wolf_manager_->addCapture(new_capture);

  std::list<laserscanutils::Line> line_list;
  new_capture->extractLines(line_list);

  visualization_msgs::Marker new_line_;
  new_line_.header.stamp = ros::Time::now();
  new_line_.header.frame_id = "/map";
  new_line_.type = visualization_msgs::Marker::LINE_LIST;
  new_line_.color.r = 1;
  new_line_.color.g = 0;
  new_line_.color.b = 0;
  new_line_.color.a = 1;
  new_line_.ns = "/lines";
  new_line_.id = corners_MarkerArray_msg_.markers.back().id+1;
  for (auto line_it = line_list.begin(); line_it != line_list.end(); line_it++ )
  {
    geometry_msgs::Point point1, point2;
    point1.x = line_it->point_first_(0);
    point1.y = line_it->point_first_(1);
    point1.z = 1.5;
    point2.x = line_it->point_last_(0);
    point2.y = line_it->point_last_(1);
    point2.z = 1.5;

    new_line_.points.push_back(point1);
    new_line_.points.push_back(point2);
    ROS_INFO("WolfAlgNode: New landmark!");
  }
  if (line_list.size() > 1)
    corners_MarkerArray_msg_.markers.push_back(new_line_);

  ROS_INFO("WolfAlgNode: %i Lines ", line_list.size());

    // [fill srv structure and make request to the server]

    // [fill action structure and make request to the action server]

    // [publish messages]
    corners_publisher_.publish(this->corners_MarkerArray_msg_);


//  ROS_INFO("Ranges:");
//  std::copy(msg->ranges.begin(), msg->ranges.end(), std::ostream_iterator<float>(std::cout, " "));
//  ROS_INFO("Double ranges:");
//  std::copy(double_ranges.begin(), double_ranges.end(), std::ostream_iterator<double>(std::cout, " "));
//  ROS_INFO("Scan:");
//  std::cout << "scan:" << scan_reading.transpose() << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->laser_back_mutex_exit();
}

void WolfAlgNode::laser_back_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_back_mutex_);
}

void WolfAlgNode::laser_back_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_back_mutex_);
}

void WolfAlgNode::laser_front_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("WolfAlgNode::laser_front_callback: New Message Received");

  if (!laser_front_params_setted_)
  {
    laserscanutils::ScanParams params = laser_front_sensor_->getScanParams();
    params.angle_min_ = msg->angle_min;
    params.angle_max_ = msg->angle_max;
    params.angle_step_ = msg->angle_increment;
    params.scan_time_ = msg->time_increment;
    params.range_min_ = msg->range_min;
    params.range_max_ = msg->range_max;
    laser_front_sensor_->setScanParams(params);
  }
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->laser_front_mutex_enter();
  wolf_manager_->addCapture(new CaptureLaser2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                               laser_front_sensor_,
                                               msg->ranges));
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->laser_front_mutex_exit();
}

void WolfAlgNode::laser_front_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_front_mutex_);
}

void WolfAlgNode::laser_front_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_front_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WolfAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

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


