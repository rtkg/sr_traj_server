/**
 * @file   finger.h
 * @author Robert Krug
 * @date   Tue Mar 22, 2012
* 
* 
*/

#ifndef   	finger_h_
#define   	finger_h_

#include "trajectory_parser.h"
/* #include "trajectory_parser.h" */
/* #include <Eigen/Core> */
/* #include <boost/thread/mutex.hpp> */
/* #include "../srv_gen/cpp/include/finger/replay_traj.h" */
/* #include <sr_robot_msgs/sendupdate.h> */
#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
 #include <icr/ContactState.h>

class Finger
{
 public:
 
  Finger(std::vector<int> const & finger_joints,std::vector<int> const & grasp_joints, std::string const & sensor_topic, double force_threshold, boost::shared_ptr<TrajectoryParser> const & traj);
  Finger();
  ~Finger(){};


  bool isTouching();
  boost::shared_ptr<std::map<int,double> > getJointStates();
  void resetTrajectories();
  bool trajCompleted();
  void incrementJointStates();
  void incrementJointStates(unsigned int n_inc);

 private:

  ros::NodeHandle nh_; 

 std::string sst_; //REMOVE
 
  boost::mutex lock_;
  boost::shared_ptr<TrajectoryParser> traj_;
  double f_thresh_;
  bool touching_;
  std::vector<int> finger_joints_;
  std::vector<int> grasp_joints_;
 

  // std::map<int,double> joint_states_;
   boost::shared_ptr<std::map<int,double> > joint_states_;
   unsigned int sample_;
 bool completed_;

  ros::Subscriber sensor_sub_;


 /*  static const unsigned int number_hand_joints_; */
 /*  TrajectoryParser* trajectory_parser_; */
 /*  boost::mutex data_mutex_; */
 /*  std::vector<std::string> joint_names_;  */
 /*  ros::ServiceServer replay_traj_srv_; */
 /*  ros::Publisher shadowhand_pub_; */
 /*  std::map<std::string,unsigned int> joint_ordinals_;  */

 /*  // void initJointOrdinals(); */

 /*  int getJointId(std::string const & joint_name); */

 /*  /\** */
 /*   * Returns a vector containing the joint angles of the hand at the given instance */
 /*   * */
 /*   * @param sample - between 1-N, where N is the number of samples for the trajectory */
 /*   *\/ */
 /*  Eigen::VectorXd getStateVector(unsigned int sample); */

 /* /\** */
 /*   * Creates a sendupdate message from the given joint state vector */
 /*   * */
 /*   * @param state_vec - vector containing the joint angles */
 /*   *\/ */
 /*  sr_robot_msgs::sendupdate generateMessage(Eigen::VectorXd & state_vec); */
 /*  void initJointNames(); */

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void contactListener(const icr::ContactState::ConstPtr& ct_st);
  // bool replayTrajectory(finger::replay_traj::Request &req, finger::replay_traj::Response &res);
  
}; // end class


#endif 	
