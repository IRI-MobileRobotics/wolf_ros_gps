//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode(const Eigen::VectorXs& _prior,
                         const unsigned int& _trajectory_size,
                         const WolfScalar& _new_frame_elapsed_time,
                         const Eigen::Vector3s& _gps_sensor_p,
                         Eigen::Vector4s& _map_pose,
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
                                    new StateBlock(_map_pose.head(3)),   //map position
                                    new StateBlock(_map_pose.tail(1)));  // map orientation
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
    fix_ecef_sub_ = nh_.subscribe("/teo/sensors/gps/gps_ecef", 1000, &WolfGPSNode::fixEcefCallback, this);
    fix_arrived_ = 0;

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

    //TODO check if it is correct to put this here
    std::cout << std::endl << " ========= destroying ceres manager (now seg fault) =============" << std::endl;
    delete ceres_manager_;
}


void WolfGPSNode::process()
{
    ROS_INFO("================ PROCESSING.\t(%i gps obs discarded)", gps_data_arrived_);
    gps_data_arrived_ = 0;
    ros::Time local_stamp = ros::Time::now();

    //solve problem
    //std::cout << "wolf updated" << std::endl;
    ceres_manager_->update(use_auto_diff_wrapper_, apply_loss_function_);
    //std::cout << "ceres updated" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);
    std::cout << "------------------------- SOLVED -------------------------" << std::endl;
    //std::cout << (use_auto_diff_wrapper_ ? "AUTO DIFF WRAPPER" : "CERES AUTO DIFF") << std::endl;
//    std::cout << summary.FullReport() << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    //ceres_manager_->computeCovariances(); //TODO if uncommented, crash. tell Joan
    //std::cout << "covariances computed" << std::endl;

    // Sets localization timestamp & Gets wolf localization estimate
    time_last_process_ = ros::Time::now();
    Eigen::VectorXs vehicle_pose  = getVehiclePose();

    /*
     * TODO check this part!!!
     * TODO check this part!!!
     */
    // Broadcast transforms ---------------------------------------------------------------------------
    publishWorld2MapTF(gps_sensor_ptr_->getMapPPtr()->getVector(), gps_sensor_ptr_->getMapOPtr()->getVector(),
                       getVehiclePose().head(2), getVehiclePose().tail(1),
                       gps_sensor_ptr_->getPPtr()->getVector());

    //Get map2base from Wolf result, and builds base2map pose
    tf::Pose map2base;
    map2base.setOrigin( tf::Vector3(vehicle_pose(0), vehicle_pose(1), 0) );
    map2base.setRotation( tf::createQuaternionFromYaw(vehicle_pose(2)) );

    //base2map: invert map2base to get base2map (map wrt base), and stamp it
    tf::Stamped<tf::Pose> base2map(map2base.inverse(), ros::Time::now(), base_frame_name_);

    /*
     * TODO check this part!!!
     * TODO check this part!!!
     * TODO check this part!!!
     * TODO check this part!!!
     * TODO check this part!!!
     */
    //gets odom2map (map wrt odom), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::Stamped<tf::Pose> odom2map;
    if ( tfl_.waitForTransform(odom_frame_name_, base_frame_name_, ros::Time::now(), ros::Duration(1)) )
    {
        //gets odom2map
        tfl_.transformPose(odom_frame_name_, base2map, odom2map);

        //broadcast map2odom = odom2map.inverse()
        tfb_.sendTransform( tf::StampedTransform(odom2map.inverse(), ros::Time::now(), map_frame_name_, odom_frame_name_) );
    }
    else
        ROS_WARN_STREAM("No odom_to_base frame received: "<< odom_frame_name_<<" " << base_frame_name_);


    //End Broadcast transform -----------------------------------------------------------------------------
    // [fill msg structures]

    // MARKERS VEHICLE & CONSTRAINTS
    // [...]


    std::cout << std::setprecision(12);
    std::cout << "\n~~~~ RESULTS ~~~~\n";
    std::cout << "|\tmap P: " << gps_sensor_ptr_->getMapPPtr()->getVector().transpose() << std::endl;// map position wrt (ecef)
    std::cout << "|\tmap O: " << gps_sensor_ptr_->getMapOPtr()->getVector().transpose() << std::endl;// map orientation wrt (ecef)
    std::cout << "|\tVehicle Pose: " << getVehiclePose().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
    //publishTrajectory(false); (now is in main)
    std::cout << "|\tsensor P: " << gps_sensor_ptr_->getPPtr()->getVector().transpose() << std::endl;// position of the sensor with respect to the vehicle's frame
    //        std::cout << "|\tsensor O (not needed):" << gps_sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;// orientation of antenna is not needed, because omnidirectional
    std::cout << "|\tbias: " << gps_sensor_ptr_->getIntrinsicPtr()->getVector().transpose() << std::endl;//intrinsic parameter  = receiver time bias
    //std::cout << "|\tLoc: (" << vehicle_pose(0) << "," << vehicle_pose(1) << "," << vehicle_pose(2) << ")" << std::endl;// position of the vehicle's frame with respect to the map frame
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";


}
/*
 * calculate translation of map frame
 */
void WolfGPSNode::publishWorld2MapTF(Eigen::Vector3s _map_p, Eigen::Vector1s _map_o, Eigen::Vector2s _vehicle_p, Eigen::Vector1s _vehicle_o, Eigen::Vector3s _sensor_p)
{
    int verbose_level_ = 0;
    bool tf_gps_wrt_other_frames = true; //true only for debug

    std::cout << std::setprecision(12);
    if (verbose_level_ >= 1)
    {
        std::cout << "============================================ " << std::endl;
        std::cout << "_sensor_p(_base): " << _sensor_p.transpose() << std::endl;
        std::cout << "_vehicle_p(_map): " << _vehicle_p.transpose() << std::endl;
        std::cout << "_vehicle_o(_map): " << _vehicle_o[0] << std::endl;
        std::cout << "_map_p: " << _map_p.transpose() << "\t|  " << "_map_o: " << _map_o[0] << std::endl;
    }

    // broadcast TF of gps wrt base
    tfb_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(_sensor_p[0], _sensor_p[1], _sensor_p[2])),ros::Time::now(),base_frame_name_,gps_frame_name_));

    Eigen::Matrix<WolfScalar, 4, 1> sensor_p_base(_sensor_p[0], _sensor_p[1], _sensor_p[2], WolfScalar(1)); //sensor position wrt base (the vehicle)
    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<WolfScalar, 4, 4> T_map_base = Eigen::Matrix<WolfScalar, 4, 4>::Identity();
    T_map_base(0, 0) = cos(_vehicle_o[0]);
    T_map_base(0, 1) = -sin(_vehicle_o[0]);
    T_map_base(1, 0) = sin(_vehicle_o[0]);
    T_map_base(1, 1) = cos(_vehicle_o[0]);
    T_map_base(0, 3) = _vehicle_p[0];
    T_map_base(1, 3) = _vehicle_p[1];

    // sensor position with respect to map frame
    Eigen::Matrix<WolfScalar, 4, 1> sensor_p_map = T_map_base * sensor_p_base;

    if (verbose_level_ >= 1)
        std::cout << "!!! sensor_p_map: " << sensor_p_map.transpose() << std::endl;


    // broadcast TF of gps wrt map
    if(tf_gps_wrt_other_frames)
        tfb_.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(sensor_p_map[0], sensor_p_map[1], sensor_p_map[2])), ros::Time::now(), map_frame_name_, "gps_map"));


    /*
     * _map_p from ECEF to LLA (math from https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf )
     */
    // WGS84 ellipsoid constants
    WolfScalar a = 6378137.0; // earth's radius
    WolfScalar e = 8.1819190842622e-2; // eccentricity
    WolfScalar asq = a * a;
    WolfScalar esq = e * e;
    WolfScalar b = sqrt(asq * (1) - esq);
    WolfScalar bsq = b * b;
    WolfScalar ep = sqrt((asq - bsq) / bsq);
    WolfScalar p = sqrt(_map_p[0] * _map_p[0] + _map_p[1] * _map_p[1]);
    WolfScalar th = atan2(a * _map_p[2], b * p);
    WolfScalar lon = atan2(_map_p[1], _map_p[0]);
    WolfScalar lat = atan2((_map_p[2] + ep * ep * b * pow(sin(th), 3)), (p - esq * a * pow(cos(th), 3)));

    if(verbose_level_>=2)
    {
        std::cout << "_map_p LLA: " << lat << ", " << lon << std::endl;
        std::cout << "_map_p LLA degrees: " << (lat * 180 / M_PI) << ", " << (lon * 180 / M_PI) << std::endl;
    }

    /*
     * map-to-ECEF transform matrix
     * made by the product of the next 4 matrixes
     */
    Eigen::Matrix<WolfScalar, 4, 4> T_ecef_aux = Eigen::Matrix<WolfScalar, 4, 4>::Identity();
    T_ecef_aux(0, 3) = WolfScalar(_map_p[0]);
    T_ecef_aux(1, 3) = WolfScalar(_map_p[1]);
    T_ecef_aux(2, 3) = WolfScalar(_map_p[2]);
    if (verbose_level_ >= 2)
        std::cout << "T_ecef_aux\n" << T_ecef_aux << std::endl << std::endl;

    Eigen::Matrix<WolfScalar, 4, 4> T_aux_lon = Eigen::Matrix<WolfScalar, 4, 4>::Identity();
    T_aux_lon(0, 0) = cos(lon);
    T_aux_lon(0, 1) = -sin(lon);
    T_aux_lon(1, 0) = sin(lon);
    T_aux_lon(1, 1) = cos(lon);
    if (verbose_level_ >= 2)
        std::cout << "T_aux_lon\n" << T_aux_lon << std::endl << std::endl;

    Eigen::Matrix<WolfScalar, 4, 4> T_lon_lat = Eigen::Matrix<WolfScalar, 4, 4>::Identity();
    T_lon_lat(0, 0) = cos(lat);
    T_lon_lat(0, 2) = -sin(lat);
    T_lon_lat(2, 0) = sin(lat);
    T_lon_lat(2, 2) = cos(lat);
    if (verbose_level_ >= 2)
        std::cout << "T_lon_lat\n" << T_lon_lat << std::endl << std::endl;

    Eigen::Matrix<WolfScalar, 4, 4> T_lat_enu = Eigen::Matrix<WolfScalar, 4, 4>::Zero();
    T_lat_enu(0, 2) = T_lat_enu(1, 0) = T_lat_enu(2, 1) = T_lat_enu(3, 3) = 1;
    if (verbose_level_ >= 2)
        std::cout << "T_lat_enu\n" << T_lat_enu << std::endl << std::endl;

    Eigen::Matrix<WolfScalar, 4, 4> T_enu_map = Eigen::Matrix<WolfScalar, 4, 4>::Identity();
    T_enu_map(0, 0) = WolfScalar(cos(_map_o[0]));
    T_enu_map(0, 1) = WolfScalar(-sin(_map_o[0]));
    T_enu_map(1, 0) = WolfScalar(sin(_map_o[0]));
    T_enu_map(1, 1) = WolfScalar(cos(_map_o[0]));
    if (verbose_level_ >= 2)
        std::cout << "T_enu_map\n" << T_enu_map << std::endl << std::endl;


    Eigen::Matrix<WolfScalar, 4, 4> T_ecef_map = T_ecef_aux *  T_aux_lon * T_lon_lat * T_lat_enu * T_enu_map;
    if (verbose_level_ >= 2)
        std::cout << "---------T_ecef_map\n" << T_ecef_map << std::endl << std::endl;


    //sensor position with respect to ecef coordinate system
    Eigen::Matrix<WolfScalar, 4, 1> sensor_p_ecef =  T_ecef_map * sensor_p_map;
    if (verbose_level_ >= 1)
        std::cout << "!!! sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2] << std::endl;


    //broadcast map wrt world TF
    Eigen::Matrix<WolfScalar, 3, 3> submatrix = T_ecef_map.block(0, 0, 3, 3);//create quaternion from the submatrix from 0, 0 big 3, 3
    Eigen::Quaternions quat_ecef_map(submatrix);
    tf::Quaternion q;
    q.setX(quat_ecef_map.x());
    q.setY(quat_ecef_map.y());
    q.setZ(quat_ecef_map.z());
    q.setW(quat_ecef_map.w());
    if(verbose_level_ >= 1)
        std::cout << "QUATERNION:" << q.x() << ", "<< q.y() << ", " << q.z() << ", " << q.w() << std::endl << std::endl;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_map_p[0], _map_p[1], _map_p[2]));
    transform.setRotation(q);
    tfb_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame_name_, map_frame_name_));



    // broadcast TF of gps wrt world
    if(tf_gps_wrt_other_frames)
        tfb_.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(sensor_p_ecef[0], sensor_p_ecef[1], sensor_p_ecef[2])), ros::Time::now(), world_frame_name_, "gps_world"));

}


void WolfGPSNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::odometry_callback: New Message Received");
    if (last_odom_stamp_ != ros::Time(0))
    {
        float dt = (msg->header.stamp - last_odom_stamp_).toSec();

        addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                     TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                     sensor_prior_,
                                     Eigen::Vector3s(msg->twist.twist.linear.x*dt, 0. ,msg->twist.twist.angular.z*dt)));
    }
    last_odom_stamp_ = msg->header.stamp;
}


void WolfGPSNode::gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::GPS_callback: New Message Received");
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

Eigen::VectorXs WolfGPSNode::getVehiclePose(const TimeStamp& _now)
{
    if (last_capture_relative_ == nullptr && last_key_frame_ != nullptr)
        return Eigen::Map<Eigen::Vector3s>(last_key_frame_->getPPtr()->getPtr());
    else if (last_capture_relative_ == nullptr)
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
    //std::cout << "creating new frame..." << _frame_state.transpose() << std::endl;

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

//    // Zero odometry (to be integrated)
//    if (last_key_frame_ != nullptr)
//    {
//        //TODO leave commented after adding odometry?
//        CaptureMotion* empty_odom = new CaptureOdom2D(_time_stamp, _time_stamp, sensor_prior_, Eigen::Vector3s::Zero());
//        current_frame_->addCapture(empty_odom);
//        empty_odom->process();
//        last_capture_relative_ = empty_odom;
//    }
    //std::cout << "last_key_frame_" << std::endl;

    // ---------------------- KEY FRAME ---------------------
    if (last_key_frame_ != nullptr)
    {
        last_key_frame_->setKey();
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
        //std::cout << "creating new frame from prior..." << std::endl;
        createFrame(last_capture_relative_->computeFramePose(_time_stamp), _time_stamp);
    }
    else if (last_key_frame_ != nullptr)
    {
        //std::cout << "creating new frame from last keyframe..." << std::endl;
        createFrame(last_key_frame_->getState(), _time_stamp);
    }
    else
    {
        //std::cout << "creating new frame from zero..." << std::endl;
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
            last_capture_relative_ = (CaptureMotion*) new_capture;
            last_capture_relative_->process();
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

void WolfGPSNode::publishTrajectory(bool verbose)
{
    std::cout << "Trajectory:\n";
    //To print all the previous frame
    int i=0;
    for (auto it : *(problem_->getTrajectoryPtr()->getFrameListPtr()))
    {
        if(verbose)
            std::cout << "|\tFrame" << (it->isKey() ? "K" : "NK") << ": " << it->nodeId() << " Vehicle P: " << it->getPPtr()->getVector().transpose() << "\t| Vehicle O: " << it->getOPtr()->getVector().transpose() << std::endl;

        tf::Quaternion orientation = tf::createQuaternionFromYaw(it->getOPtr()->getVector()[0]);

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_name_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = i++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = it->getPPtr()->getVector()[0];
        marker.pose.position.y = it->getPPtr()->getVector()[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = orientation.getX();
        marker.pose.orientation.y = orientation.getY();
        marker.pose.orientation.z = orientation.getZ();
        marker.pose.orientation.w = orientation.getW();
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub_.publish( marker );

    }
}

void WolfGPSNode::fixEcefCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg)
{
    fix_arrived_++;

//    if(fix_arrived_ % 5 == 0)     std::cout << "%%%%  FIX:  = (" << std::setprecision(12) << msg->x << ", " << msg->y << ", " << msg->z << ")\n";

    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_name_;
    m.header.stamp = ros::Time::now();

    m.ns = "gps_fix";
    m.id = 1;//fix_arrived_;

    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    // Pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = msg->x;
    m.pose.position.y = msg->y;
    m.pose.position.z = msg->z;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x =  m.scale.y = m.scale.z = 5;

    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 0.4;
    marker_pub_.publish(m);

    // broadcast TF of last fix received
    tfb_.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(msg->x, msg->y, msg->z)),ros::Time::now(),world_frame_name_,"gps_fix"));

}