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
#include "sr_traj_server/SetTimestep.h"
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#define RAD 57.2957795

//-------------------------------------------------------------------
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
  double timestep_;
  std::vector<std::string> joint_names_; 
  ros::ServiceServer replay_traj_srv_;
  ros::ServiceServer load_traj_srv_;
  ros::ServiceServer step_traj_srv_;
  ros::ServiceServer move_start_srv_;
  ros::ServiceServer set_timestep_srv_;

#ifdef SENDUPDATE
   ros::Publisher sendupdate_pub_;
#else
   ros::V_Publisher output_pubs_;
#endif

   
  /**
   * Returns a vector containing the joint angles of the hand at the given instance
   *
   * @param sample - between 1-N, where N is the number of samples for the trajectory
   */
  Eigen::VectorXd getStateVector(unsigned int sample);

   void initJointNames();

#ifdef SENDUPDATE
  void publishSendupdate();
#else
    void publish();
#endif

  /////////////////
  //  CALLBACKS  //
  /////////////////

   bool loadTrajectory(sr_traj_server::LoadTrajectory::Request &req, sr_traj_server::LoadTrajectory::Response &res);
   bool stepTrajectory(sr_traj_server::StepTrajectory::Request &req, sr_traj_server::StepTrajectory::Response &res);
   bool setTimestep(sr_traj_server::SetTimestep::Request &req, sr_traj_server::SetTimestep::Response &res);
   bool moveStart(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   bool replayTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
}; // end class
//-------------------------------------------------------------------
  /**
   * Templated signum function 
   */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
//-------------------------------------------------------------------
#endif 	
