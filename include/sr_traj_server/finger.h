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
  std::map<int,double>  getJointStates();
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
 

  std::map<int,double>  joint_states_;
   unsigned int sample_;
 bool completed_;

  ros::Subscriber sensor_sub_;


  /////////////////
  //  CALLBACKS  //
  /////////////////

  void contactListener(const icr::ContactState::ConstPtr& ct_st);
   
}; // end class


#endif 	
