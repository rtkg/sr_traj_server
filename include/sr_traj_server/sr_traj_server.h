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
#include "../srv_gen/cpp/include/sr_traj_server/replay_traj.h"
#include "../srv_gen/cpp/include/sr_traj_server/Grasp.h"
#include <sr_robot_msgs/sendupdate.h>
#include <string>
#include <map>
#include "finger.h"
#include <boost/shared_ptr.hpp>
#include <std_srvs/Empty.h>

#define THUMB        0
#define FOREFINGER   1
#define MIDDLEFINGER 2
#define RINGFINGER   3
#define LITTLEFINGER 4


class TrajectoryServer
{
 public:
 
  TrajectoryServer();
  ~TrajectoryServer();

 void spin();

 private:


  ros::NodeHandle nh_, nh_private_;
  static const unsigned int number_hand_joints_;
  boost::shared_ptr<TrajectoryParser> trajectory_parser_;
  boost::mutex lock_;
  std::vector<std::string> joint_names_; 
  ros::ServiceServer replay_traj_srv_;
  ros::ServiceServer grasp_srv_;
   ros::ServiceServer reset_hand_srv_;
  ros::Publisher shadowhand_pub_;
  //std::map<std::string,unsigned int> joint_ordinals_; 
  std::vector<Finger*> fingers_;
  // void initJointOrdinals();
 bool traj_loaded_;
 double delta_t_;
 std::vector<unsigned int> contact_fingers_;

  int getJointId(std::string const & joint_name);
  bool contactsEstablished();
  bool completed();
  void followTrajectories();
  void initContactFingers();
  
  Eigen::VectorXd getStateVector(unsigned int sample);

 /**
   * Creates a sendupdate message from the given joint state vector
   *
   * @param state_vec - vector containing the joint angles
   */
  sr_robot_msgs::sendupdate generateMessage(Eigen::VectorXd const & state_vec);
  void initJointNames();

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool replayTrajectory(sr_traj_server::replay_traj::Request &req, sr_traj_server::replay_traj::Response &res);
  bool grasp(sr_traj_server::Grasp::Request &req, sr_traj_server::Grasp::Response &res);
  bool resetHand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  
}; // end class


#endif 	
