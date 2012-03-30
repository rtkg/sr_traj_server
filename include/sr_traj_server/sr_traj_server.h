/**
 * @file   sr_traj_server.h
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
* 
*@brief Node reading and publishing joint state trajectories for the Shadow Hand. Also implemented is a
*simple reactive force controller. If one of the specified contact fingers touches during the
*execution of the trajectoies, it will stop until all contact fingers are in contact. Subsequently,
*the joints of all contact fingers will continue executing their trajectories until one of them
*reaches the specified force magnitude maximum.
*/


#ifndef   	sr_traj_server_h_
#define   	sr_traj_server_h_

#include "trajectory_parser.h"
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>
#include "../srv_gen/cpp/include/sr_traj_server/replay_traj.h"
//#include "../srv_gen/cpp/include/sr_traj_server/Grasp.h"
#include <sr_robot_msgs/sendupdate.h>
#include <string>
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

  /**
   *@brief VERY ugly state machine  
   */
 void spin();

 private:

  ros::NodeHandle nh_, nh_private_;
  static const unsigned int number_hand_joints_;
  boost::shared_ptr<TrajectoryParser> trajectory_parser_;
  boost::mutex lock_;
  std::vector<std::string> joint_names_; 

  /**
   *@brief service takes a trajectory file, the fingers intended for contact (e.g. [TH,FF] for a
   *pinch grasp) and the maximum contact force magnitude. This service triggers the state machine
   *responsible for executing the trajectory following + contact reactive grasping
   */
  ros::ServiceServer replay_traj_srv_;
  //ros::ServiceServer grasp_srv_;
  /**
   *@brief Publishes a sendupdate message with 0 on all joint angles
   */
   ros::ServiceServer reset_hand_srv_;
 /**
   *@brief Publishes sendupdate messages corresponding to the current hand joint states
   */
  ros::Publisher shadowhand_pub_;
  /**
   *@brief Vector of fingers - the fingers subscribe to the force sensor topic and return the
   *current state of their respective joint angles, as well as information about the contact state
   *and whether the maximum force magnitude threshold is reached.
   */
  std::vector<Finger*> fingers_;

  /**
   *@brief Vector holding the current joint angles of the hand
   */
  Eigen::VectorXd state_vec_;
  /**
   *@brief Flag indicating whether a trajectory is loaded or not
   */
 bool traj_loaded_;
  /**
   *@brief The sendupdate messages are published at (approximately - no realtime loop) 1/delta_t_ Hz
   */
 double delta_t_;
  /**
   *@brief If any contact force reaches this threshold, the grasp is seen as completed
   */
  double gf_thres_;
  /**
   *@brief Vector holding the indices to TrajectoryServer::joint_names_ indicating the contact fingers (e.g. LF should not be a contact finger for a pinch grasp)
   */
 std::vector<unsigned int> contact_fingers_;

  int getJointId(std::string const & joint_name);
  /**
   *@brief checks if a fingertip is in contact
   */
  bool contactsEstablished();
  /**
   *@brief checks if all joints finished their trajectories - should only trigger if there were no
   *grasp contacts established during execution
   */
  bool completed();
  /**
   *@brief Updates TrajectoryServer::state_vec_ with the current joint angle states
   */
  void followTrajectories();
  /**
   *@brief only updates the entries in TrajectoryServer::state_vec_ which belong to joints of TrajectoryServer::contact_fingers_
   */
  void graspTrajectories();
  /**
   *@brief 
   */
  void initContactFingers();
  
  Eigen::VectorXd getStateVector(unsigned int sample);

 /**
   * Creates a sendupdate message from the given joint state vector
   *
   * @param state_vec - vector containing the joint angles
   */
  sr_robot_msgs::sendupdate generateMessage();
  void initJointNames();

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool replayTrajectory(sr_traj_server::replay_traj::Request &req, sr_traj_server::replay_traj::Response &res);
  // bool grasp(sr_traj_server::Grasp::Request &req, sr_traj_server::Grasp::Response &res);
  bool resetHand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  
}; // end class


#endif 	
