//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF GPS MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, "wolf_gps_node");

    // some parameters for the node
    Eigen::Vector3s prior = Eigen::Vector3s(0, 0, 0);//prior pose of base in map
    int window_length = 8;
    double new_frame_elapsed_time = 1.0;
    Eigen::Vector3s gps_sensor_p(2, 2, 2);
    Eigen::Vector4s init_vehicle_pose(4789400, 176900, 4194500, 0);
    Eigen::Vector2s odom_std(0.2, 0.2);

    // Wolf GPS ROS node
    WolfGPSNode* wgps = new WolfGPSNode(prior, window_length, new_frame_elapsed_time, gps_sensor_p, init_vehicle_pose, odom_std);


    ros::Rate loopRate(1);

    while(ros::ok())
    {
        //execute pending callbacks
        ros::spinOnce();

        if(wgps->hasDataToProcess())//TODO mettere un altro tipo di controllo. vedi foglio todo
        {
            wgps->process();
        }

        // todo publish pose, broadcast tf and everything

        //relax to fit output rate
        loopRate.sleep();//TODO togli?

    }


    return 0;
}