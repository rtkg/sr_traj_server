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

  sensor_sub_=nh_.subscribe<kcl_msgs::KCL_ContactStateStamped>(sensor_topic, 1, &Finger::contactListener, this);


}
//-------------------------------------------------------------------
void Finger::contactListener(const kcl_msgs::KCL_ContactStateStamped::ConstPtr& ct_st)
{
  lock_.lock();

  c_force_(0)=ct_st->contact_normal.x*ct_st->Fnormal+ct_st->tangential_force.x;
  c_force_(1)=ct_st->contact_normal.y*ct_st->Fnormal+ct_st->tangential_force.y;
  c_force_(2)=ct_st->contact_normal.z*ct_st->Fnormal+ct_st->tangential_force.z;


  if(c_force_.norm() > f_thresh_)
    {
      touching_=true;
    }
  // else
  //  touching_=false;

  lock_.unlock();
}
//-------------------------------------------------------------------
bool Finger::isTouching()
{
  bool touching;
  lock_.lock();
  // std::cout<<sst_<<" is touching"<<std::endl;
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

  // if(!strcmp(sst_.c_str(),"/sensor_remapper/ffdistal/contact_state"))
  //   {
  //     std::cout<<"sample "<<sample_<<" force "<<c_force_.norm()<<"touching "<<touching_<<std::endl;
  //    }

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
 bool Finger::incrementGraspJointStates(double gf_thresh)
 {
   lock_.lock();

   if(c_force_.norm() >= gf_thresh )
     {
       lock_.unlock();
       // std::cout<<sst_<<" reached grasp force."<<std::endl;
       return true;
     }

   Eigen::VectorXd state_vec;
   state_vec.resize(traj_->getNumTraj());
   state_vec=traj_->getStateVector(sample_); 

   for(unsigned int i=0; i<grasp_joints_.size();i++)
     joint_states_[grasp_joints_[i]]=state_vec(grasp_joints_[i]);

   sample_++;

   if(traj_->getNumSamples() <= sample_)
     {
       sample_=traj_->getNumSamples()-1;
       ROS_WARN("Using last sample - end of trajectory reached.");
     }
  
   lock_.unlock();
   return false;
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
 Eigen::Vector3d Finger::getCForce()
 {
  lock_.lock();
  Eigen::Vector3d c_force=c_force_;
  lock_.unlock();

  return c_force;
}
//------------------------------------------------------------------------------------------------------
