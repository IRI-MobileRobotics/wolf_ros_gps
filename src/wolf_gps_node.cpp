
//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode(SensorGPS* _gps_sensor_ptr,
                         const FrameStructure _frame_structure,
                         SensorBase* _sensor_prior_ptr,
                         const Eigen::VectorXs& _prior,
                         const Eigen::MatrixXs& _prior_cov,
                         const unsigned int& _trajectory_size,
                         const WolfScalar& _new_frame_elapsed_time) :
        nh_(ros::this_node::getName()),
        last_odom_stamp_(0),

        gps_sensor_ptr_(_gps_sensor_ptr),

        problem_(new WolfProblem()),
        frame_structure_(_frame_structure),
        sensor_prior_(_sensor_prior_ptr),
        current_frame_(nullptr),
        last_key_frame_(nullptr),
        last_capture_relative_(nullptr),
        trajectory_size_(_trajectory_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
{
    std::cout << "WolfGPSNode::WolfGPSNode(...) -- constructor\n";

    //******** inizio costrutt wolf manager
    if (_frame_structure == PO_2D)
        assert( _prior.size() == 3 &&
                _prior_cov.cols() == 3 &&
                _prior_cov.rows() == 3 &&
                "Wrong init_frame state vector or covariance matrix size");
    else
        assert( _prior.size() == 7 &&
                _prior_cov.cols() == 7 &&
                _prior_cov.rows() == 7 &&
                "Wrong init_frame state vector or covariance matrix size");


    // Initial frame
    createFrame(_prior, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
    //std::cout << " first_window_frame_" << std::endl;

    // Initial covariance
    SensorBase* prior_sensor = new SensorBase(ABSOLUTE_POSE, nullptr, nullptr, nullptr, 0);
    problem_->getHardwarePtr()->addSensor(prior_sensor);
    CaptureFix* initial_covariance = new CaptureFix(TimeStamp(0), prior_sensor, _prior, _prior_cov);
    //std::cout << " initial_covariance" << std::endl;
    current_frame_->addCapture(initial_covariance);
    //std::cout << " addCapture" << std::endl;

    // Current robot frame
    createFrame(_prior, TimeStamp(0));

//    gps_tf_loaded_ = false;

    //std::cout << " wolfmanager initialized" << std::endl;
    //************ fine costrutt wolf manager


    base_frame_name_ = "base";
    gps_frame_name_ = "gps";
    ecef_frame_name_ = "ecef";
    map_frame_name_ = "map";
    odom_frame_name_ = "odom";

    // Odometry sensor
    addSensor(sensor_prior_);

    // GPS sensor
    addSensor(gps_sensor_ptr_);
    gps_sensor_ptr_->addProcessor(new ProcessorGPS());

    // [init publishers]
    //TODO ask joan: why map2base is between map and odom?
    // Broadcast 0 transform to align frames initially
    Tf_map2base_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    tfb_.sendTransform( tf::StampedTransform(Tf_map2base_, ros::Time::now(), map_frame_name_, odom_frame_name_));

    // [init subscribers]
    odom_sub_ = nh_.subscribe("odometry", 10, &WolfGPSNode::odometryCallback, this);
    gps_sub_ = nh_.subscribe("/sat_pseudoranges", 1000, &WolfGPSNode::gpsCallback, this);
    gps_data_arrived_ = 0;

    max_iterations_ = 1;

    use_auto_diff_wrapper_ = false;
    apply_loss_function_ = false;

    // init ceres
    ceres_options_.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION;
    ceres_options_.max_line_search_step_contraction = 1e-3;
    ceres_options_.max_num_iterations = max_iterations_;
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_manager_ = new CeresManager(problem_, problem_options_);

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


void WolfGPSNode::gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    gps_data_arrived_++;

    if(gps_sensor_ptr_ != nullptr)
    {
        //std::cout << "------MSG: found " << msg->measurements.size() << " sats\n";

        std::cout << "creating CaptureGPS\n";
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

//        CaptureGPS* cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, obs);
//        new_captures_.push(cpt_ptr_);
    }

}

bool WolfGPSNode::hasDataToProcess()
{
    return (gps_data_arrived_ > 0);
}

void WolfGPSNode::process()
{
    std::cout << "COMPUTING the last " << gps_data_arrived_ << " GPS obs. ros time now() = " << ros::Time::now() << std::endl;
    gps_data_arrived_ = 0;

    //solve problem
    //std::cout << "wolf updating..." << std::endl;
    update();
    //std::cout << "wolf updated" << std::endl;
    ceres_manager_->update(use_auto_diff_wrapper_, apply_loss_function_);
    //std::cout << "ceres updated" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_->solve(ceres_options_);
    //std::cout << "------------------------- SOLVED -------------------------" << std::endl;
    //std::cout << (use_auto_diff_wrapper_ ? "AUTO DIFF WRAPPER" : "CERES AUTO DIFF") << std::endl;
    //std::cout << summary.FullReport() << std::endl;
    //std::cout << summary.BriefReport() << std::endl;
    ceres_manager_->computeCovariances();
    //std::cout << "covariances computed" << std::endl;

    // Sets localization timestamp & Gets wolf localization estimate
    ros::Time loc_stamp = ros::Time::now();
    Eigen::VectorXs vehicle_pose  = getVehiclePose();

    // Broadcast transform ---------------------------------------------------------------------------
    //Get map2base from Wolf result, and builds base2map pose
    tf::Pose map2base;
    map2base.setOrigin( tf::Vector3(vehicle_pose(0), vehicle_pose(1), 0) );
    map2base.setRotation( tf::createQuaternionFromYaw(vehicle_pose(2)) );

    std::cout << "Loc: (" << vehicle_pose(0) << "," << vehicle_pose(1) << "," << vehicle_pose(2) << ")" << std::endl;

    //base2map: invert map2base to get base2map (map wrt base), and stamp it
    tf::Stamped<tf::Pose> base2map(map2base.inverse(), loc_stamp, base_frame_name_);

    //gets odom2map (map wrt odom), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::Stamped<tf::Pose> odom2map;
    if ( tfl_.waitForTransform(odom_frame_name_, base_frame_name_, loc_stamp, ros::Duration(0.1)) )
    {
        //gets odom2map
        tfl_.transformPose(odom_frame_name_, base2map, odom2map);

        //invert odom2map to get map2odom, and stamp it
        //tf::Stamped<tf::Pose> map2odom(odom2map.inverse(), loc_stamp, map_frame_name_);

        //broadcast map2odom = odom2map.inverse()
        //tfb_.sendTransform( tf::StampedTransform(map2odom, loc_stamp, map_frame_name_, odom_frame_name_) );
        tfb_.sendTransform( tf::StampedTransform(odom2map.inverse(), loc_stamp, map_frame_name_, odom_frame_name_) );

    }
    else
        ROS_WARN("No odom to base frame received");

    //End Broadcast transform -----------------------------------------------------------------------------
    // [fill msg structures]

    // MARKERS VEHICLE & CONSTRAINTS
    // [...]


    if(gps_sensor_ptr_!= nullptr)
    {
        std::cout << std::setprecision(12);
        std::cout << "\n~~~~ RESULTS ~~~~\n";
        std::cout << "|\tinitial P: " << gps_sensor_ptr_->getInitVehiclePPtr()->getVector().transpose() <<
        std::endl;// initial vehicle position (ecef)
        std::cout << "|\tinitial O: " << gps_sensor_ptr_->getInitVehicleOPtr()->getVector().transpose() <<
        std::endl;// initial vehicle orientation (ecef)
        std::cout << "|\tVehicle Pose: " << getVehiclePose().transpose() <<
        std::endl;// position of the vehicle's frame with respect to the initial pos frame
        //    std::cout << "|\tVehicle P (last frame): " << problem_->getLastFramePtr()->getPPtr()->getVector().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
        //    std::cout << "|\tVehicle O (last frame): " << problem_->getLastFramePtr()->getOPtr()->getVector().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
        std::cout << "|\tsensor P: " << gps_sensor_ptr_->getPPtr()->getVector().transpose() <<
        std::endl;// position of the sensor with respect to the vehicle's frame
        //        std::cout << "|\tsensor O (not needed):" << gps_sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;// orientation of antenna is not needed, because omnidirectional
        std::cout << "|\tbias: " << gps_sensor_ptr_->getIntrinsicPtr()->getVector().transpose() <<
        std::endl;//intrinsic parameter  = receiver time bias
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";
    }


}


Eigen::VectorXs WolfGPSNode::getVehiclePose(const TimeStamp& _now)
{
    if (last_capture_relative_ == nullptr)
        return Eigen::Map<Eigen::Vector3s>(current_frame_->getPPtr()->getPtr());
    else
        return last_capture_relative_->computeFramePose(_now);
}



void WolfGPSNode::update()
{
    //std::cout << "updating..." << std::endl;
    while (!new_captures_.empty())
    {
        // EXTRACT NEW CAPTURE
        CaptureBase* new_capture = new_captures_.front();
        new_captures_.pop();

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
            //std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD/INTEGRATE NEW ODOMETRY TO THE LAST FRAME
            last_capture_relative_->integrateCapture((CaptureMotion*) (new_capture));
            current_frame_->setState(last_capture_relative_->computeFramePose(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
            delete new_capture;
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
    //std::cout << "updated" << std::endl;
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
    //std::cout << "creating new frame..." << std::endl;

    // current frame -> KEYFRAME
    last_key_frame_ = current_frame_;

    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch ( frame_structure_)
    {
        case PO_2D:
        {
            assert( _frame_state.size() == 3 && "Wrong init_frame state vector or covariance matrix size");

            problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                                                 new StateBlock(_frame_state.head(2)),
                                                                 new StateBlock(_frame_state.tail(1))));
            break;
        }
        case PO_3D:
        {
            assert( _frame_state.size() == 7 && "Wrong init_frame state vector or covariance matrix size");

            problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                                                 new StateBlock(_frame_state.head(3)),
                                                                 new StateBlock(_frame_state.tail(4),ST_QUATERNION)));
            break;
        }
        default:
        {
            assert( "Unknown frame structure");
        }
    }
    //std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
    //std::cout << "current_frame_" << std::endl;

    // Zero odometry (to be integrated)
    if (last_key_frame_ != nullptr)
    {
        CaptureMotion* empty_odom = new CaptureOdom2D(_time_stamp, _time_stamp, sensor_prior_, Eigen::Vector3s::Zero());
        current_frame_->addCapture(empty_odom);
        empty_odom->process();
        last_capture_relative_ = empty_odom;
    }
    //std::cout << "last_key_frame_" << std::endl;

    // ---------------------- KEY FRAME ---------------------
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

void WolfGPSNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::relative_odometry_cal lback: New Message Received");
    if (last_odom_stamp_ != ros::Time(0))
    {

        float dt = (msg->header.stamp - last_odom_stamp_).toSec();
        addCapture(new CaptureOdom2D(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                    TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                    sensor_prior_,
                                                    Eigen::Vector3s(msg->twist.twist.linear.x*dt, 0. ,msg->twist.twist.angular.z*dt)));
        //Eigen::Vector3s(msg->pose.pose.position.x, 0. ,tf::getYaw(msg->pose.pose.orientation))));

    }
    last_odom_stamp_ = msg->header.stamp;
}

void WolfGPSNode::createFrame(const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame from prior..." << std::endl;
    createFrame(last_capture_relative_->computeFramePose(_time_stamp), _time_stamp);
}


void WolfGPSNode::addCapture(CaptureBase* _capture)
{
    new_captures_.push(_capture);
//    std::cout << "added new capture: " << _capture->nodeId() << " stamp: ";
//    _capture->getTimeStamp().print();
//    std::cout << std::endl;
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

void WolfGPSNode::addSensor(SensorBase* _sensor_ptr)
{
    std::cout << "adding sensor... to hardware " << problem_->getHardwarePtr()->nodeId() << std::endl;
    problem_->getHardwarePtr()->addSensor(_sensor_ptr);
    std::cout << "added!" << std::endl;
}

///*
// * TODO qui rispetto al suo codice ho tolto tutta la partedi tf
// * è solo per visualizzare i risultati o serve anche per creare il problema?
// */
//void WolfGPSNode::loadGPSTf()
//{
//    std::cout << "loading GPS sensor " << std::endl;
//
//    //DEBUG: prints gps sensor pose
////    std::cout << "GPS sensor " << ": " << gps_frame_name_ << ": " << gps_sensor_p.transpose() << std::endl;
//
//
//    //DEBUG: prints map pose (aka init_vehicle_pose)
////    std::cout << "init_vechicle_pose " << ": " << map_frame_name_ << ": " << init_vehicle_pose.transpose() << std::endl;
//
//
//
//    //set wolf states and sensors
//    gps_sensor_ptr_ = new SensorGPS(sensor_p,
//                                    sensor_o,
//                                    new StateBlock(Eigen::Vector1s::Zero()),//sensor_bias,
//                                    new StateBlock(init_vehicle_pose.head(3)),//init_vehicle_p,
//                                    new StateBlock(init_vehicle_pose.tail(1)));//init_vehicle_o);
//    addSensor(gps_sensor_ptr_);
//    gps_sensor_ptr_->addProcessor(new ProcessorGPS());
//    gps_tf_loaded_ = true;
//
//    std::cout << "GPS antenna initialized" << std::endl;
//
//}