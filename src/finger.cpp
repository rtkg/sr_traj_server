/**
 * @file   finger.cpp
 * @author Robert Krug
 * @date   Tue Mar 22, 2012
 *
 */

#include "sr_traj_server/finger.h"

//-------------------------------------------------------------------
Finger::Finger(std::vector<int> const & finger_joints,std::vector<int> const & grasp_joints, std::string const & sensor_topic, double force_threshold,boost::shared_ptr<TrajectoryParser> const & traj) : 
  traj_(traj), f_thresh_(force_threshold),finger_joints_(finger_joints),grasp_joints_(grasp_joints), sample_(0),completed_(false)
{

  sensor_sub_=nh_.subscribe<icr::ContactState>(sensor_topic, 1, &Finger::contactListener, this);


  sst_=sensor_topic;//REMOVE! only for debugging

}
//-------------------------------------------------------------------
void Finger::contactListener(const icr::ContactState::ConstPtr& ct_st)
{
  lock_.lock();
  if(sqrt(pow(ct_st->wrench.force.x,2)+pow(ct_st->wrench.force.y,2)+pow(ct_st->wrench.force.z,2)) > f_thresh_)
    touching_=true;
   else
     touching_=false;

  lock_.unlock();
}
//-------------------------------------------------------------------
bool Finger::isTouching()
{
  bool touching;
  lock_.lock();
  touching=touching_;
  lock_.unlock();

  return touching;
}
//-------------------------------------------------------------------
std::map<int,double>  Finger::getJointStates()
{
  lock_.lock();
 
  std::map<int,double> joint_states=joint_states_;


  lock_.unlock();

  return joint_states;
}
//------------------------------------------------------------------------------------------------------
void Finger::incrementJointStates()
{

  lock_.lock();
  Eigen::VectorXd state_vec;
  state_vec.resize(traj_->getNumTraj());
  state_vec=traj_->getStateVector(sample_); 


    if((!touching_) || (sample_==0))
      {
	for(unsigned int i=0; i<finger_joints_.size();i++)
            joint_states_[finger_joints_[i]]=state_vec(finger_joints_[i]);

	sample_++;
      }

  if(traj_->getNumSamples() <= sample_)
    {
      sample_=traj_->getNumSamples()-1;
      completed_=true;
    }

  lock_.unlock();
}
//------------------------------------------------------------------------------------------------------
void Finger::incrementJointStates(unsigned int n_inc)
{
  lock_.lock();
  Eigen::VectorXd state_vec;
  state_vec.resize(traj_->getNumTraj());

  if(traj_->getNumSamples() >= (sample_+n_inc)+1)
    sample_+=n_inc;
  else
    sample_=traj_->getNumSamples()-1;

  state_vec=traj_->getStateVector(sample_); 


  for(unsigned int i=0; i<grasp_joints_.size();i++)
    joint_states_[grasp_joints_[i]]=state_vec(grasp_joints_[i]);
  

  lock_.unlock();
}
//------------------------------------------------------------------------------------------------------
void Finger::resetTrajectories()
{
  lock_.lock();
  sample_=0;
  completed_=false;
  touching_=false;
  lock_.unlock();
}
//------------------------------------------------------------------------------------------------------
bool Finger::trajCompleted()
{
  lock_.lock();
  bool completed=completed_;
  lock_.unlock();

  return completed;
}
//------------------------------------------------------------------------------------------------------

