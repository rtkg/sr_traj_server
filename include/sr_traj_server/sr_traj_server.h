/**
 * @file   sr_traj_server.h
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
* 
* Node reading and publishing joint state trajectories for the Shadow Hand
*/

#ifndef   	sr_traj_server_h_
#define   	sr_traj_server_h_


#include "trajectory_parser.h"
//#include <Eigen/Core>
#include <boost/thread/mutex.hpp>


class TrajectoryServer
{
 public:
 
  TrajectoryServer();
  ~TrajectoryServer();

 private:

  ros::NodeHandle nh_, nh_private_;

  static const unsigned int number_hand_joints_;

  TrajectoryParser* trajectory_parser_;

  boost::mutex data_mutex_;

  /////////////////
  //  CALLBACKS  //
  /////////////////


  
}; // end class


#endif 	
