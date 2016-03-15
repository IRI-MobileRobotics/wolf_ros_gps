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
    int window_length = 5;
    double new_frame_elapsed_time = 1.0;
    // BUONO LUNGO
    Eigen::Vector3s gps_sensor_p(0.55, -0.2, 1.1);
    Eigen::Vector4s map_pose(4789373, 177039, 4194527, 15.0*M_PI/180);//sidewalk near parking quimica

    Eigen::Vector1s sensor_bias; sensor_bias << -0.0002;
//    //BUONO CORTO
//    Eigen::Vector3s gps_sensor_p(0.55, -0.2, 1.1);
//    Eigen::Vector4s map_pose(4789400.30058, 176969.115645, 4194497.87659, (195.0-170.0+45)*M_PI/180);//sidewalk near parking quimica

    Eigen::Vector2s odom_std(0.2, 0.2);

    // Wolf GPS ROS node
    WolfGPSNode* wgps = new WolfGPSNode(prior, window_length, new_frame_elapsed_time, gps_sensor_p, sensor_bias, map_pose, odom_std);


    ros::Rate loopRate(2);

    while(ros::ok())
    {
        //execute pending callbacks
        ros::spinOnce();

        if((wgps->hasDataToProcess())&&(ros::Time::now() > wgps->getTimeLastProcess() + ros::Duration(1)))
        {
            wgps->process();
            wgps->publishTrajectory(true);
        }

        //relax to fit output rate
        loopRate.sleep();

    }


    return 0;
}