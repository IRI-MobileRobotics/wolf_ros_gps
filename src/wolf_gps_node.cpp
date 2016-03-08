//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode(const Eigen::VectorXs& _prior,
                         const unsigned int& _trajectory_size,
                         const WolfScalar& _new_frame_elapsed_time,
                         const Eigen::Vector3s& _gps_sensor_p,
                         Eigen::Vector4s& _init_vehicle_pose,
                         Eigen::Vector2s& _odom_std) :
        nh_(ros::this_node::getName()),
        last_odom_stamp_(0),
        problem_(new WolfProblem(PO_2D)),
        current_frame_(nullptr),
        last_key_frame_(nullptr),
        last_capture_relative_(nullptr),
        trajectory_size_(_trajectory_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
{
    //std::cout << "WolfGPSNode::WolfGPSNode(...) -- constructor\n";
    assert( _prior.size() == 3 && "Wrong init_frame state vector");

    // GPS sensor
    gps_sensor_ptr_ = new SensorGPS(new StateBlock(_gps_sensor_p, true), //gps sensor position. for now is fixed,
                                    new StateBlock(Eigen::Vector4s::Zero(), true),   //gps sensor orientation. is fixed
                                    new StateBlock(Eigen::Vector1s::Zero()),      //gps sensor bias
                                    new StateBlock(_init_vehicle_pose.head(3)),   //vehicle initial position
                                    new StateBlock(_init_vehicle_pose.tail(1)));  // vehicle initial orientation
    problem_->getHardwarePtr()->addSensor(gps_sensor_ptr_);

    gps_sensor_ptr_->addProcessor(new ProcessorGPS());

    // Odometry sensor
    sensor_prior_ = new SensorOdom2D(new StateBlock(Eigen::Vector2s::Zero()), new StateBlock(Eigen::Vector1s::Zero()), _odom_std[0], _odom_std[1]);//both arguments initialized on top
    problem_->getHardwarePtr()->addSensor(sensor_prior_);

    // Initial frame
    createFrame(_prior, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
    //std::cout << " first_window_frame_" << std::endl;

    // Current robot frame
    createFrame(_prior, TimeStamp(0));



    // [init publishers]
    // Broadcast 0 transform to align frames initially

    T_map2odom_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0));
    tfb_.sendTransform( tf::StampedTransform(T_map2odom_, ros::Time::now(), map_frame_name_, odom_frame_name_));


    // [init subscribers]
    odom_sub_ = nh_.subscribe("/teo/odomfused", 10, &WolfGPSNode::odometryCallback, this);
    gps_sub_ = nh_.subscribe("/sat_pseudoranges", 1000, &WolfGPSNode::gpsCallback, this);
    gps_data_arrived_ = 0;

    max_iterations_ = 1e4;

    use_auto_diff_wrapper_ = false;
    apply_loss_function_ = false;

    // init ceres
    ceres_options_.minimizer_type = ceres::TRUST_REGION;
    ceres_options_.max_line_search_step_contraction = 1e-3;
    ceres_options_.max_num_iterations = max_iterations_;
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_manager_ = new CeresManager(problem_, problem_options_);

    // init publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    ROS_INFO("STARTING IRI WOLF...");
}

WolfGPSNode::~WolfGPSNode()
{
    std::cout << std::endl << " ========= WolfGPSNode DESTRUCTOR (should not crash) =============" << std::endl;
    problem_->destruct();

    //TODO check if is correct to put this here
    std::cout << std::endl << " ========= destroying ceres manager (now seg fault) =============" << std::endl;
    delete ceres_manager_;
}


void WolfGPSNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::relative_odometry_callback: New Message Received");
    if (last_odom_stamp_ != ros::Time(0))
    {
        float dt = (msg->header.stamp - last_odom_stamp_).toSec();
        addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                     TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                     sensor_prior_,
                                     Eigen::Vector3s(msg->twist.twist.linear.x*dt, 0. ,msg->twist.twist.angular.z*dt)));
        Eigen::Vector3s(msg->pose.pose.position.x, 0. ,tf::getYaw(msg->pose.pose.orientation));

    }
    last_odom_stamp_ = msg->header.stamp;
}


void WolfGPSNode::gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    gps_data_arrived_++;

    if(gps_sensor_ptr_ != nullptr)
    {
        //std::cout << "------MSG: found " << msg->measurements.size() << " sats\n";

        // Create CaptureGPS
        rawgpsutils::SatellitesObs obs;
        obs.time_ros_sec_ = msg->time_ros.sec;
        obs.time_ros_nsec_ = msg->time_ros.nsec;
        obs.time_gps_wnc_ = msg->time_gps_wnc;
        obs.time_gps_tow_ = msg->time_gps_tow;
        for (int i = 0; i < msg->measurements.size(); ++i)
        {
            rawgpsutils::PrMeasurement sat(msg->measurements[i].sat_id,
                                           msg->measurements[i].pseudorange,
                                           msg->measurements[i].x,
                                           msg->measurements[i].y,
                                           msg->measurements[i].z,
                                           msg->measurements[i].v_x,
                                           msg->measurements[i].v_y,
                                           msg->measurements[i].v_z);

            obs.measurements_.push_back(sat);
        }
        //std::cout << "------OBS: found " << obs.measurements_.size() << " sats\n";

        TimeStamp time_stamp(obs.time_ros_sec_, obs.time_ros_nsec_);

        addCapture(new CaptureGPS(time_stamp, gps_sensor_ptr_, obs));
        //std::cout << "added CaptureGPS with " << msg->measurements.size() << " measurements\n";

    }
}

void WolfGPSNode::process()
{
    ROS_INFO("PROCESSING.\t\t(%i gps obs discarded)", gps_data_arrived_);
    gps_data_arrived_ = 0;
    time_last_process_ = ros::Time::now();

    //solve problem
    //std::cout << "wolf updated" << std::endl;
    ceres_manager_->update(use_auto_diff_wrapper_, apply_loss_function_);
    //std::cout << "ceres updated" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);
    //std::cout << "------------------------- SOLVED -------------------------" << std::endl;
    //std::cout << (use_auto_diff_wrapper_ ? "AUTO DIFF WRAPPER" : "CERES AUTO DIFF") << std::endl;
    //std::cout << summary.FullReport() << std::endl;
    //std::cout << summary.BriefReport() << std::endl;
    //ceres_manager_->computeCovariances();
    //std::cout << "covariances computed" << std::endl;

    // Sets localization timestamp & Gets wolf localization estimate
    ros::Time loc_stamp = time_last_process_;//ros::Time::now();
    Eigen::VectorXs vehicle_pose  = getVehiclePose();

    // Broadcast transforms ---------------------------------------------------------------------------
    //Get map2base from Wolf result, and builds base2map pose
    tf::Pose map2base;
    map2base.setOrigin( tf::Vector3(vehicle_pose(0), vehicle_pose(1), 0) );
    map2base.setRotation( tf::createQuaternionFromYaw(vehicle_pose(2)) );


    std::cout << "Loc: (" << vehicle_pose(0) << "," << vehicle_pose(1) << "," << vehicle_pose(2) << ")" << std::endl;

    //base2map: invert map2base to get base2map (map wrt base), and stamp it
    tf::Stamped<tf::Pose> base2map(map2base.inverse(), ros::Time::now(), base_frame_name_);

    //gets odom2map (map wrt odom), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::Stamped<tf::Pose> odom2map;
    if ( tfl_.waitForTransform(odom_frame_name_, base_frame_name_, ros::Time::now(), ros::Duration(1)) )
    {
        //gets odom2map
        tfl_.transformPose(odom_frame_name_, base2map, odom2map);

        //invert odom2map to get map2odom, and stamp it
        tf::Stamped<tf::Pose> map2odom(odom2map.inverse(), ros::Time::now(), map_frame_name_);

        //broadcast map2odom = odom2map.inverse()
        //tfb_.sendTransform( tf::StampedTransform(map2odom, loc_stamp, map_frame_name_, odom_frame_name_) );
        tfb_.sendTransform( tf::StampedTransform(odom2map.inverse(), ros::Time::now(), map_frame_name_, odom_frame_name_) );
    }
    else
        ROS_WARN_STREAM("No odom_to_base frame received: "<< odom_frame_name_<<" " << base_frame_name_);

    //quaternione che fa ruotare l'asse x in modo che combaci con il vettore passato
    Eigen::Quaterniond rot;
    rot.setFromTwoVectors(Eigen::Vector3d::UnitZ(), gps_sensor_ptr_->getInitVehiclePPtr()->getVector());

    /*
     * TODO
     * find geographic north and align x-axis to north+initVehicleO
     */

    tf::Transform T_map2ecef;
    T_map2ecef.setOrigin(tf::Vector3(gps_sensor_ptr_->getInitVehiclePPtr()->getVector()[0], gps_sensor_ptr_->getInitVehiclePPtr()->getVector()[1], gps_sensor_ptr_->getInitVehiclePPtr()->getVector()[2]));
    //I think that with this multiplication between quaternion i obtain the desired rotation: z-axis aligned normal to the ground, x-axis aligned alpha degree from north
    T_map2ecef.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()) * tf::createQuaternionFromYaw(gps_sensor_ptr_->getInitVehicleOPtr()->getVector()[0]));
    tfb_.sendTransform(tf::StampedTransform(T_map2ecef, loc_stamp, world_frame_name_, map_frame_name_));

    /*
     * questa non credo sia da inviare!
     * devo inviare map to odom
     */
//    tf::Transform T_base2map;
//    T_base2map.setOrigin( tf::Vector3(vehicle_pose(0), vehicle_pose(1), 0) );
//    T_base2map.setRotation( tf::createQuaternionFromYaw(vehicle_pose(2)) );
//    tfb_.sendTransform(tf::StampedTransform(T_base2map, loc_stamp, map_frame_name_, base_frame_name_));


    //End Broadcast transform -----------------------------------------------------------------------------
    // [fill msg structures]

    // MARKERS VEHICLE & CONSTRAINTS
    // [...]
    if(debug_mode)
    {
        operatorDebug(getVehiclePose().head(2), getVehiclePose().tail(1),
                      gps_sensor_ptr_->getPPtr()->getVector(), gps_sensor_ptr_->getIntrinsicPtr()->getVector(),
                      gps_sensor_ptr_->getInitVehiclePPtr()->getVector(),
                      gps_sensor_ptr_->getInitVehicleOPtr()->getVector());
    }
    std::cout << std::setprecision(12);
    std::cout << "\n~~~~ RESULTS ~~~~\n";
    std::cout << "|\tinitial P: " << gps_sensor_ptr_->getInitVehiclePPtr()->getVector().transpose() << std::endl;// initial vehicle position (ecef)
    std::cout << "|\tinitial O: " << gps_sensor_ptr_->getInitVehicleOPtr()->getVector().transpose() << std::endl;// initial vehicle orientation (ecef)
    std::cout << "|\tVehicle Poses: " << getVehiclePose().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
    //To print all the previous frame
    for (auto it : *(problem_->getTrajectoryPtr()->getFrameListPtr()))
    {
        std::cout << "|\tVehicle P: " << it->getPPtr()->getVector().transpose() << "\t| Vehicle O: " << it->getOPtr()->getVector().transpose() << std::endl;
    }
    std::cout << "|\tsensor P: " << gps_sensor_ptr_->getPPtr()->getVector().transpose() << std::endl;// position of the sensor with respect to the vehicle's frame
    //        std::cout << "|\tsensor O (not needed):" << gps_sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;// orientation of antenna is not needed, because omnidirectional
    std::cout << "|\tbias: " << gps_sensor_ptr_->getIntrinsicPtr()->getVector().transpose() << std::endl;//intrinsic parameter  = receiver time bias
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";


}


void WolfGPSNode::operatorDebug(Eigen::Vector2s _vehicle_p, Eigen::Vector1s _vehicle_o,
                                Eigen::Vector3s _sensor_p, Eigen::Vector1s _bias,
                                Eigen::Vector3s _init_vehicle_p, Eigen::Vector1s _init_vehicle_o)
{
    Eigen::Vector1s residual;

//    std::cout << std::setprecision(12);
//    std::cout << "\n~~~~ RESULTS INSIDE ~~~~\n";
//    std::cout << "|\tinitial P: " << _init_vehicle_p.transpose() << std::endl;// initial vehicle position (ecef)
//    std::cout << "|\tinitial O: " << _init_vehicle_o.transpose() << std::endl;// initial vehicle orientation (ecef)
//    std::cout << "|\tVehicle P: " << _vehicle_p.transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
//    std::cout << "|\tVehicle O: " << _vehicle_o.transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
//
//    std::cout << "|\tsensor P: " << _sensor_p.transpose() << std::endl;// position of the sensor with respect to the vehicle's frame
//    std::cout << "|\tbias: " << _bias << std::endl;//intrinsic parameter  = receiver time bias
//    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";

    int marker_duration_ms = 5000;

    visualization_msgs::Marker m;
    m.header.frame_id = base_frame_name_;
    m.header.stamp = ros::Time::now();
    m.ns = "sensor_base";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;//SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    // Pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = _sensor_p[0];
    m.pose.position.y = _sensor_p[1];
    m.pose.position.z = _sensor_p[2];
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x =  m.scale.y = m.scale.z = 0.5;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.5;
    m.lifetime = ros::Duration(marker_duration_ms); //after tot seconds satellites are deleted
    marker_pub_.publish(m);

    Eigen::Matrix<WolfScalar , 3, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2]); //sensor position with respect to the base (the vehicle)
    Eigen::Matrix<WolfScalar, 3, 1> vehicle_p_map(_vehicle_p[0], _vehicle_p[1], 0);
    Eigen::Matrix<WolfScalar, 3, 1> init_vehicle_p(_init_vehicle_p[0], _init_vehicle_p[1], _init_vehicle_p[2]);
    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<WolfScalar, 3, 3> T_base2map = Eigen::Matrix<WolfScalar, 3, 3>::Identity();
    T_base2map(0, 0) = cos(_vehicle_o[0]);
    T_base2map(0, 1) = sin(_vehicle_o[0]);
    T_base2map(1, 0) = -sin(_vehicle_o[0]);
    T_base2map(1, 1) = cos(_vehicle_o[0]);
    Eigen::Matrix<WolfScalar, 3, 1> sensor_p_map; // sensor position with respect to map frame (initial frame of the experiment)
    sensor_p_map = T_base2map * sensor_p_base + vehicle_p_map;

    std::cout << "1st trasform:  ";
    std::cout << "sensor_p_map: " << sensor_p_map[0] << ", " << sensor_p_map[1] << ", " << sensor_p_map[2] << std::endl;


    m.header.frame_id = map_frame_name_;
    m.header.stamp = ros::Time::now();
    m.ns = "sensor_map";
    m.id = 1;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    // Pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = sensor_p_map[0];
    m.pose.position.y = sensor_p_map[1];
    m.pose.position.z = sensor_p_map[2];
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x =  m.scale.y = m.scale.z = 0.5;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.5;
    m.lifetime = ros::Duration(marker_duration_ms); //after tot seconds satellites are deleted
    marker_pub_.publish(m);


//
//
//    /*
//     * _init_vehicle_p from ecef to lla
//     */
//    // WGS84 ellipsoid constants
//    WolfScalar a = WolfScalar(6378137); // earth's radius
//    WolfScalar e = WolfScalar(8.1819190842622e-2); // eccentricity
//    WolfScalar asq = a * a;
//    WolfScalar esq = e * e;
//    WolfScalar b = WolfScalar(sqrt(asq * (WolfScalar(1) - esq)));
//    WolfScalar bsq = WolfScalar(b * b);
//    WolfScalar ep = WolfScalar(sqrt((asq - bsq) / bsq));
//    WolfScalar p = WolfScalar(sqrt(_init_vehicle_p[0] * _init_vehicle_p[0] + _init_vehicle_p[1] * _init_vehicle_p[1]));
//    WolfScalar th = WolfScalar(atan2(a * _init_vehicle_p[2], b * p));
//    WolfScalar lon = WolfScalar(atan2(_init_vehicle_p[1], _init_vehicle_p[0]));
//    WolfScalar lat = WolfScalar(atan2((_init_vehicle_p[2] + ep * ep * b * pow(sin(th), 3)), (p - esq * a * pow(cos(th), 3))));
//    //        WolfScalar N = WolfScalar(a/( sqrt(WolfScalar(1)-esq*pow(sin(lat),2)) ));
//    //        WolfScalar alt = WolfScalar(p / cos(lat) - N);
//    // mod lat to 0-2pi
//    while (lon < WolfScalar(0))
//        lon += WolfScalar(2 * M_PI);
//    while (lon >= 2 * M_PI)
//        lon -= WolfScalar(2 * M_PI);
//    // correction for altitude near poles left out.
//    if (verbose)
//    {
//        std::cout << "_init_vehicle_p: " << _init_vehicle_p[0] << ", " << _init_vehicle_p[1] << ", "
//        << _init_vehicle_p[2] << std::endl;
//        //        std::cout << "_init_vehicle_p LLA: " << lat << ", " << lon /*<< ", " << alt*/ << std::endl;
//        //        std::cout << "_init_vehicle_p LLA degrees: " << lat * WolfScalar(180 / M_PI) << ", " << lon * WolfScalar(180 / M_PI) /*<< ", " << alt*/ << std::endl;
//    }
//    /*
//     * map-to-ECEF transform matrix
//     */
//    Eigen::Matrix<WolfScalar, 3, 3> R1 = Eigen::Matrix<WolfScalar, 3, 3>::Identity();
//    R1(0, 0) = WolfScalar(cos(lon));
//    R1(0, 1) = WolfScalar(sin(lon));
//    R1(1, 0) = WolfScalar(-sin(lon));
//    R1(1, 1) = WolfScalar(cos(lon));
//    Eigen::Matrix<WolfScalar, 3, 3> R2 = Eigen::Matrix<WolfScalar, 3, 3>::Identity();
//    R2(0, 0) = WolfScalar(cos(lat));
//    R2(0, 2) = WolfScalar(sin(lat));
//    R2(2, 0) = WolfScalar(-sin(lat));
//    R2(2, 2) = WolfScalar(cos(lat));
//    Eigen::Matrix<WolfScalar, 3, 3> R3 = Eigen::Matrix<WolfScalar, 3, 3>::Zero();
//    R3(0, 1) = R3(1, 2) = R3(2, 0) = WolfScalar(1);
//    Eigen::Matrix<WolfScalar, 3, 3> R4 = Eigen::Matrix<WolfScalar, 3, 3>::Identity();
//    R4(0, 0) = WolfScalar(cos(_init_vehicle_o[0]));
//    R4(0, 1) = WolfScalar(sin(_init_vehicle_o[0]));
//    R4(1, 0) = WolfScalar(-sin(_init_vehicle_o[0]));
//    R4(1, 1) = WolfScalar(cos(_init_vehicle_o[0]));
//    Eigen::Matrix<WolfScalar, 3, 3> T_map2ecef = (R4 * R3 * R2 * R1).inverse();
//    /*
//     * result I want to find: sensor position with respect to ecef
//     */
//    Eigen::Matrix<WolfScalar, 3, 1> sensor_p_ecef; //sensor position with respect to ecef coordinate system
//    sensor_p_ecef = T_map2ecef * sensor_p_map + init_vehicle_p;
//    if (verbose)
//        std::cout << "!!! sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2]
//        << std::endl;
//
//    //il codice qui sotto è quello vecchio, adattato in modo da usare la posizione del sensore rispetto a ecef, calcolata qui sopra
//    WolfScalar square_sum = WolfScalar(0);
//    for (int i = 0; i < 3; ++i)
//    {
//        square_sum += pow(sensor_p_ecef[i] - WolfScalar(sat_position_[i]), 2);
//    }
//    WolfScalar distance = (square_sum != WolfScalar(0)) ? sqrt(square_sum) : WolfScalar(0);
//
//    //     error = (expected measurement)       - (actual measurement)
//    _residual[0] = (distance + _bias[0] * WolfScalar(LIGHT_SPEED)) - (pseudorange_);
//
//    if (verbose)
//    {
//        std::cout << "Residual: " << _residual[0] << "\n";
//    }
//    // normalizing by the covariance
//    _residual[0] = _residual[0] / WolfScalar(getMeasurementCovariance()(0, 0));//WolfScalar(sqrt(getMeasurementCovariance()(0, 0)));
//
//
//
//    if (verbose)
//    {
//        //        std::cout << "Expected: " << (distance + _bias[0]*WolfScalar(LIGHT_SPEED)) << "\nreceived = " << pseudorange_ << "\n";
//        std::cout << "Residual norm: " << _residual[0] << "\n";
//
//        std::cout << "------ END OPERATOR()------\n";
//    }


}


Eigen::VectorXs WolfGPSNode::getVehiclePose(const TimeStamp& _now)
{
    if (last_capture_relative_ == nullptr)
        return Eigen::Map<Eigen::Vector3s>(current_frame_->getPPtr()->getPtr());
    else
        return last_capture_relative_->computeFramePose(_now);
}



bool WolfGPSNode::checkNewFrame(CaptureBase* new_capture)
{
    //std::cout << "checking if new frame..." << std::endl;
    // TODO: not only time, depending on the sensor...
    //std::cout << new_capture->getTimeStamp().get() - last_frame_->getTimeStamp().get() << std::endl;
    return new_capture->getTimeStamp().get() - last_key_frame_->getTimeStamp().get() > new_frame_elapsed_time_;
}


void WolfGPSNode::createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
    std::cout << "creating new frame..." << _frame_state.transpose() << std::endl;

    // current frame -> KEYFRAME
    last_key_frame_ = current_frame_;

    // ---------------------- CREATE NEW FRAME ---------------------
    problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                           new StateBlock(_frame_state.head(2)),
                                           new StateBlock(_frame_state.tail(1))));

    //std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
    //std::cout << "current_frame_" << std::endl;

    // Zero odometry (to be integrated)
    if (last_key_frame_ != nullptr)
    {
        //TODO leave commented after adding odometry?
        CaptureMotion* empty_odom = new CaptureOdom2D(_time_stamp, _time_stamp, sensor_prior_, Eigen::Vector3s::Zero());
        current_frame_->addCapture(empty_odom);
        empty_odom->process();
        last_capture_relative_ = empty_odom;
    }
    //std::cout << "last_key_frame_" << std::endl;

    // ---------------------- KEY FRAME ---------------------
    //last_key_frame_->setKey();
    if (last_key_frame_ != nullptr)
    {
        //std::cout << "Processing last frame non-odometry captures " << current_frame_->getCaptureListPtr()->size() << std::endl;
        for (auto capture_it = last_key_frame_->getCaptureListPtr()->begin(); capture_it != last_key_frame_->getCaptureListPtr()->end(); capture_it++)
            if ((*capture_it)->getSensorPtr() != sensor_prior_)
            {
                //std::cout << "processing capture " << (*capture_it)->nodeId() << std::endl;
                (*capture_it)->process();
            }


    }
    //std::cout << "Last key frame non-odometry captures processed" << std::endl;

    // ---------------------- MANAGE WINDOW OF POSES ---------------------
    manageWindow();
    //std::cout << "new frame created" << std::endl;
}


void WolfGPSNode::createFrame(const TimeStamp& _time_stamp)
{
    if (last_capture_relative_ != nullptr)
    {
        std::cout << "creating new frame from prior..." << std::endl;
        createFrame(last_capture_relative_->computeFramePose(_time_stamp), _time_stamp);
    }
    else if (last_key_frame_ != nullptr)
    {
        std::cout << "creating new frame from last keyframe..." << std::endl;
        createFrame(last_key_frame_->getState(), _time_stamp);
    }
    else
    {
        std::cout << "creating new frame from zero..." << std::endl;
        createFrame(problem_->getTrajectoryPtr()->getLastFramePtr()->getState(), _time_stamp);
    }
}


void WolfGPSNode::addCapture(CaptureBase* new_capture)
{
    // OVERWRITE CURRENT STAMP
    current_frame_->setTimeStamp(new_capture->getTimeStamp());

    // INITIALIZE FIRST FRAME STAMP
    if (last_key_frame_->getTimeStamp().get() == 0)
        last_key_frame_->setTimeStamp(new_capture->getTimeStamp());

    // NEW KEY FRAME ?
    if (checkNewFrame(new_capture))
        createFrame(new_capture->getTimeStamp());

    // ODOMETRY SENSOR
    if (new_capture->getSensorPtr() == sensor_prior_)
    {
//        std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;

        // ADD ODOM CAPTURE TO THE CURRENT FRAME (or integrate to the previous capture)
        //std::cout << "searching repeated capture..." << new_capture->nodeId() << std::endl;
        CaptureBaseIter repeated_capture_it = current_frame_->hasCaptureOf(sensor_prior_);

        if (repeated_capture_it != current_frame_->getCaptureListPtr()->end()) // repeated capture
        {
//            std::cout << "existing odom capture, integrating new capture" << new_capture->nodeId() << std::endl;
            last_capture_relative_->integrateCapture((CaptureMotion*) (new_capture));
            current_frame_->setState(last_capture_relative_->computeFramePose(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
            delete new_capture;
        }
        else
        {
//            std::cout << "not repeated, adding capture..." << new_capture->nodeId() << std::endl;
            current_frame_->addCapture(new_capture);
        }
    }
    else
    {
        //std::cout << "adding not odometry capture..." << new_capture->nodeId() << std::endl;

        // ADD CAPTURE TO THE CURRENT FRAME (or substitute the same sensor previous capture)
        //std::cout << "searching repeated capture..." << new_capture->nodeId() << std::endl;
        CaptureBaseIter repeated_capture_it = current_frame_->hasCaptureOf(new_capture->getSensorPtr());

        if (repeated_capture_it != current_frame_->getCaptureListPtr()->end()) // repeated capture
        {
            //std::cout << "repeated capture, keeping new capture" << new_capture->nodeId() << std::endl;
            current_frame_->removeCapture(repeated_capture_it);
            current_frame_->addCapture(new_capture);
        }
        else
        {
            //std::cout << "not repeated, adding capture..." << new_capture->nodeId() << std::endl;
            current_frame_->addCapture(new_capture);
        }
    }
}


void WolfGPSNode::manageWindow()
{
    //std::cout << "managing window..." << std::endl;
    // WINDOW of FRAMES (remove or fix old frames)
    if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > trajectory_size_+1)
    {
        //std::cout << "first_window_frame_ " << (*first_window_frame_)->nodeId() << std::endl;
        //problem_->getTrajectoryPtr()->removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
        (*first_window_frame_)->fix();
        first_window_frame_++;
    }
    //std::cout << "window managed" << std::endl;
}

bool WolfGPSNode::hasDataToProcess()
{
    return (gps_data_arrived_ > 0);
}

ros::Time WolfGPSNode::getTimeLastProcess()
{
    return time_last_process_;
}

void WolfGPSNode::publish()
{
    //TODO
}