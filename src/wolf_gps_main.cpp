//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF GPS MAIN ===========\n\n";

    //init ros
    ros::init(argc, argv, "wolf_gps_node");

    /*
     * Parameters, to be optimized
     */
    StateBlock* sensor_p = new StateBlock(Eigen::Vector3s::Zero()); //gps sensor position
    sensor_p->fix(); // TODO only for now, to simplify things
    StateBlock* sensor_o = new StateBlock(Eigen::Vector4s::Zero(), ST_QUATERNION);   //gps sensor orientation
    sensor_o->fix(); //orientation is fixed, because antenna omnidirectional, so is not going to be optimized
    StateBlock* sensor_bias = new StateBlock(Eigen::Vector1s::Zero());    //gps sensor bias
    // TODO Should this 2 supplementary blocks go in the sensor?
    StateBlock* vehicle_init_p = new StateBlock(Eigen::Vector3s::Zero());    //vehicle init position
    StateBlock* vehicle_init_o = new StateBlock(Eigen::Vector4s::Zero(), ST_QUATERNION);// vehicle init orientation


    // Trilateration node
    WolfGPSNode wgNode(sensor_p,
                       sensor_o,
                       sensor_bias,
                       vehicle_init_p,
                       vehicle_init_o,
                       PO_3D,
                       nullptr,                           //_sensor_prior_ptr
                       Eigen::Vector7s::Zero(),           //prior
                       Eigen::Matrix7s::Identity()*0.01,  //prior cov
                       5,                                 //window size
                       1);                                //time for new keyframe

    ros::spin();

    return 0;
}