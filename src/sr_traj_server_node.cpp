/**
 *
 * @brief Launch a ros node read trajectories and remap them to the sendupdate topic with a given timestep
 *
 *
 */

#include <ros/ros.h>
#include "sr_traj_server/sr_traj_server.h"

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_traj_server");

  TrajectoryServer traj_server;
  ros::spin();

  return 0;
}

