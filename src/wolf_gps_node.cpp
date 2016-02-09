//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"

WolfGPSNode::WolfGPSNode() :
        nh(ros::this_node::getName())
{
    std::cout << "\n\n=========== WOLF GPS ===========\n\n";

}

WolfGPSNode::~WolfGPSNode()
{

}