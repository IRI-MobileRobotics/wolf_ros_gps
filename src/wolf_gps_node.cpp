
//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode() :
        nh_(ros::this_node::getName())
{
    std::cout << "WolfGPSNode::WolfGPSNode(...) -- constructor\n";
    gps_data_arrived = 0;
    gps_sub_ = nh_.subscribe("/sat_pseudoranges", 1000, &WolfGPSNode::gpsCallback, this);

}

WolfGPSNode::~WolfGPSNode()
{
//    std::cout << std::endl << " ========= WolfGPSNode DESTRUCTOR (should not crash) =============" << std::endl;
//    problem_->destruct();
//
//    //TODO check if is correct to put this here
//    std::cout << std::endl << " ========= destroying ceres manager (now seg fault) =============" << std::endl;
//    delete ceres_manager_;
}

/*
 * TODO:
 * when a new observation arrive, save it and activate a flag "new_data"
 * and move the elaboration outside the callback:
 * something is monitoring the flag and when is on it start working
 */
void WolfGPSNode::gpsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    //TODO store data
    gps_data_arrived++;
}


bool WolfGPSNode::hasDataToProcess()
{
    return (gps_data_arrived > 0);
}

void WolfGPSNode::process()
{
    std::cout << "COMPUTING the last " << gps_data_arrived << " GPS obs" << std::endl;
    gps_data_arrived = 0;




}