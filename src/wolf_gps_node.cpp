#include <wolf/sensor_gps.h>//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode(StateBlock* _sensor_p,
                         StateBlock* _sensor_o,
                         StateBlock* _sensor_bias,
                         StateBlock* _vehicle_init_p,
                         StateBlock* _vehicle_init_o,
                         SensorBase* _sensor_prior_ptr,
                         const Eigen::VectorXs& _prior,
                         const Eigen::MatrixXs& _prior_cov,
                         const unsigned int& _trajectory_size,
                         const WolfScalar& _new_frame_elapsed_time) :
        nh_(ros::this_node::getName()),
        sensor_p_(_sensor_p),
        sensor_o_(_sensor_o),
        sensor_bias_(_sensor_bias),
        vehicle_init_p_(_vehicle_init_p),
        vehicle_init_o_(_vehicle_init_o),
        gps_sensor_ptr_(new SensorGPS(sensor_p_, sensor_o_, sensor_bias_, vehicle_init_p_, vehicle_init_o_)),
        problem_(new WolfProblem()),
        frame_structure_(PO_3D),
        sensor_prior_(_sensor_prior_ptr),
        current_frame_(nullptr),
        last_key_frame_(nullptr),
        last_capture_relative_(nullptr),
        trajectory_size_(_trajectory_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time)
{
    std::cout << "WolfGPSNode::WolfGPSNode(...) -- constructor\n";

    // Add processor to gps sensor
    gps_sensor_ptr_->addProcessor(new ProcessorGPS());
    //std::cout << "WolfGPSNode::WolfGPSNode(...) -- processor added \n";

    assert( _prior.size() == 7 &&
            _prior_cov.cols() == 7 &&
            _prior_cov.rows() == 7 &&
            "Wrong init_frame state vector or covariance matrix size");

    // Initial frame
    //std::cout << " creating first frame\n";
    createFrame(_prior, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
    //std::cout << " first_window_frame_" << std::endl;

    // Initialize ceres manager
    initCeresManager();

    // Adding the sensor
    problem_->getHardwarePtr()->addSensor(gps_sensor_ptr_);

    // Subscriber
    obs_sub_ = nh_.subscribe("/sat_pseudoranges", 1000, &WolfGPSNode::obsCallback, this);

}

WolfGPSNode::~WolfGPSNode()
{
    std::cout << std::endl << " ========= WolfGPSNode DESTRUCTOR (should not crash) =============" << std::endl;
    problem_->destruct();

    //TODO check if is correct to put this here
    std::cout << std::endl << " ========= destroying ceres manager (now seg fault) =============" << std::endl;
    delete ceres_manager_;
}

void WolfGPSNode::createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
//    std::cout << "creating new frame..." << std::endl;

    // current frame -> KEYFRAME
    last_key_frame_ = current_frame_;

    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    assert( _frame_state.size() == 7 && "Wrong init_frame state vector or covariance matrix size");



    problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                                         new StateBlock(_frame_state.head(3)),
                                                         new StateBlock(_frame_state.tail(4),ST_QUATERNION)));

//    std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
//    std::cout << "current_frame_" << std::endl;

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

void WolfGPSNode::createFrame(const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame from prior..." << std::endl;
    createFrame(Eigen::Vector7s::Zero(), _time_stamp);
}

void WolfGPSNode::initCeresManager()
{
    ceres_options_.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options_.max_line_search_step_contraction = 1e-3;
    ceres_options_.max_num_iterations = 1e4;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;

    ceres_manager_ = new CeresManager(problem_, problem_options);

}

/*
 * TODO da qualche parte devo anche creare il frame
 *
 * crealo ogni volta che arriva gps-data,
 * e integra l'odometry fino a quel punto
 */
void WolfGPSNode::obsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    std::cout << "WolfGPSNode::obsCallback\n";

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

    CaptureGPS* cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, obs);

    // Add capture
    std::cout << "creating CaptureGPS\n";
    new_captures_.push(cpt_ptr_);
    std::cout << "capture added" << std::endl;

    // update wolf tree
    updateWolfProblem();
    std::cout << "wolf Problem updated" << std::endl;

    ceres_manager_->update();
    std::cout << "ceres manager updated" << std::endl;

    ceres::Solver::Summary summary;

    summary = ceres_manager_->solve(ceres_options_);
    if(ceresVerbose)
        std::cout << summary.FullReport() << std::endl;

}

void WolfGPSNode::updateWolfProblem()
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
        if (last_key_frame_ != nullptr && last_key_frame_->getTimeStamp().get() == 0)
            last_key_frame_->setTimeStamp(new_capture->getTimeStamp());
        // NEW KEY FRAME ?
        if (checkNewFrame(new_capture))
            createFrame(new_capture->getTimeStamp());
        // ODOMETRY SENSOR
        if (new_capture->getSensorPtr() == sensor_prior_)
        {
            std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD/INTEGRATE NEW ODOMETRY TO THE LAST FRAME
            last_capture_relative_->integrateCapture((CaptureMotion*) (new_capture));
            current_frame_->setState(last_capture_relative_->computeFramePose(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
            delete new_capture;
        }
        else
        {
            std::cout << "adding not odometry capture..." << new_capture->nodeId() << std::endl;

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
    //std::cout << new_capture->getTimeStamp().get() << std::endl;
    return new_capture->getTimeStamp().get() - (last_key_frame_ == nullptr ? 0 : last_key_frame_->getTimeStamp().get()) > new_frame_elapsed_time_;
}