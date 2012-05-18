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
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>
#include "sr_traj_server/LoadTrajectory.h"
#include "sr_traj_server/StepTrajectory.h"
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#define RAD 57.2957795

class TrajectoryServer
{
 public:
 
  TrajectoryServer();
  ~TrajectoryServer();

 private:

  ros::NodeHandle nh_, nh_private_;
  static const unsigned int number_hand_joints_;
  TrajectoryParser* trajectory_parser_;
  boost::mutex lock_;
  bool traj_loaded_;
  unsigned int sample_id_;
  std::vector<std::string> joint_names_; 
  ros::ServiceServer replay_traj_srv_;
  ros::ServiceServer reset_hand_srv_;
  ros::ServiceServer load_traj_srv_;
  ros::ServiceServer step_traj_srv_;
  ros::ServiceServer move_start_srv_;
  // ros::Publisher shadowhand_pub_;
  ros::V_Publisher output_pubs_;
  
  /**
   * Returns a vector containing the joint angles of the hand at the given instance
   *
   * @param sample - between 1-N, where N is the number of samples for the trajectory
   */
  Eigen::VectorXd getStateVector(unsigned int sample);

 /**
   * Creates a sendupdate message from the given joint state vector
   *
   * @param state_vec - vector containing the joint angles
   */
 
 // void generateMessages(Eigen::VectorXd const & state_vec, std::vector<std_msgs::Float64> & joint_angles);
  void initJointNames();

  /////////////////
  //  CALLBACKS  //
  /////////////////

   bool loadTrajectory(sr_traj_server::LoadTrajectory::Request &req, sr_traj_server::LoadTrajectory::Response &res);
   bool stepTrajectory(sr_traj_server::StepTrajectory::Request &req, sr_traj_server::StepTrajectory::Response &res);
   bool resetHand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   bool moveStart(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   bool replayTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
}; // end class


#endif 	
