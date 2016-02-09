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
    problem_->destruct();
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
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;

    ceres_manager_ = new CeresManager(problem_, problem_options);

}

void WolfGPSNode::obsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    std::cout << "WolfGPSNode::obsCallback -- TODO\n";
    //TODO create gps capture

    // msg contains directly a vector of measurements!!! sat pos, vel and pseudorange!

}

