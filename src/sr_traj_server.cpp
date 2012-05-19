/**
 * @file   sr_traj_server.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
 */

#include <ros/ros.h>
#include <string>
#include "sr_traj_server/sr_traj_server.h"
#include <sys/time.h>
#include <time.h>

#ifdef SENDUPDATE
#include "sr_robot_msgs/sendupdate.h"
#include "sr_robot_msgs/joint.h"
#endif

const unsigned int TrajectoryServer::number_hand_joints_ = 20;
//-------------------------------------------------------------------
TrajectoryServer::TrajectoryServer() : nh_private_("~"),traj_loaded_(false),sample_id_(0),timestep_(0.0)
{
    std::string searched_param;
    std::string traj_dir;
    nh_private_.searchParam("trajectory_dir", searched_param);
    nh_private_.param(searched_param, traj_dir, std::string());
    trajectory_parser_ = new TrajectoryParser(traj_dir); 
    
    ROS_INFO("Trajectory directory set to: %s", traj_dir.c_str());

#ifdef SENDUPDATE
    sendupdate_pub_=nh_.advertise<sr_robot_msgs::sendupdate>("sendupdate",1);
    ROS_INFO("Trajectories will be published on the sendupdate topic");
#else 
  XmlRpc::XmlRpcValue output_topics;
  if (nh_private_.searchParam("output_topics", searched_param))
    {
      nh_.getParam(searched_param,output_topics);
      ROS_ASSERT(output_topics.getType() == XmlRpc::XmlRpcValue::TypeArray); 
      ROS_ASSERT(output_topics.size()==(int)number_hand_joints_);
    
      for (int32_t i = 0; i < output_topics.size(); ++i) 
	{    
	  ROS_ASSERT(output_topics[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          output_pubs_.push_back(nh_.advertise<std_msgs::Float64>(output_topics[i],1));
	}
    }
  else
    {
      ROS_ERROR("No output topics specified - cannot start the Trajectory Server");
      ROS_BREAK();
    }
   ROS_INFO("Trajectories will be published on the specified output topics");
#endif

    initJointNames();

    load_traj_srv_ = nh_.advertiseService("load_trajectory",&TrajectoryServer::loadTrajectory,this);
    step_traj_srv_ = nh_.advertiseService("step_trajectory",&TrajectoryServer::stepTrajectory,this);
    set_timestep_srv_ = nh_.advertiseService("set_timestep",&TrajectoryServer::setTimestep,this);
    replay_traj_srv_ = nh_.advertiseService("replay_trajectory",&TrajectoryServer::replayTrajectory,this); 
    move_start_srv_ = nh_.advertiseService("move_start",&TrajectoryServer::moveStart,this);   
}
//-------------------------------------------------------------------
TrajectoryServer::~TrajectoryServer(){delete trajectory_parser_;}
//-------------------------------------------------------------------
void TrajectoryServer::initJointNames()
{

  joint_names_.resize(number_hand_joints_);

    joint_names_[0] = "THJ1";
    joint_names_[1] = "THJ2";
    joint_names_[2] = "THJ3";
    joint_names_[3] = "THJ4";
    joint_names_[4] = "THJ5";
    joint_names_[5] = "FFJ0";
    joint_names_[6] = "FFJ3";
    joint_names_[7] = "FFJ4";
    joint_names_[8] = "MFJ0";
    joint_names_[9] = "MFJ3";
    joint_names_[10] = "MFJ4";
    joint_names_[11] = "RFJ0";
    joint_names_[12] = "RFJ3";
    joint_names_[13] = "RFJ4";
    joint_names_[14] = "LFJ0";
    joint_names_[15] = "LFJ3";
    joint_names_[16] = "LFJ4";
    joint_names_[17] = "LFJ5";
    joint_names_[18] = "WRJ1";
    joint_names_[19] = "WRJ2";
}
//-------------------------------------------------------------------
bool TrajectoryServer::loadTrajectory(sr_traj_server::LoadTrajectory::Request &req, sr_traj_server::LoadTrajectory::Response &res)
{
  res.success=false;
  lock_.lock();
  
  //Load the trajectory in the parser
  if(!trajectory_parser_->parseFile(req.file))
    {
      ROS_ERROR("Could not parse file %s",(trajectory_parser_->getTrajDir()+req.file).c_str());
      lock_.unlock();
      return res.success;
    }

  if(!trajectory_parser_->getNumTraj()==number_hand_joints_)
    {
      ROS_ERROR("%d number of joint trajectories need to be specified",number_hand_joints_);
      lock_.unlock();
      return res.success;
    }

  timestep_= trajectory_parser_->getTimestep();
  traj_loaded_=true;
  sample_id_=0;
  lock_.unlock();
  ROS_INFO("Trajectory %s loaded",(trajectory_parser_->getTrajDir()+req.file).c_str());
  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------
bool TrajectoryServer::stepTrajectory(sr_traj_server::StepTrajectory::Request &req, sr_traj_server::StepTrajectory::Response &res)
  {
   lock_.lock();
  if(!traj_loaded_)
    {
      ROS_ERROR("No trajectory loaded - cannot step");
      lock_.unlock();
      return false;
    }



  lock_.unlock();
    return true;
  }
//-------------------------------------------------------------------
bool TrajectoryServer::setTimestep(sr_traj_server::SetTimestep::Request &req, sr_traj_server::SetTimestep::Response &res)
{
 lock_.lock();
  if(!traj_loaded_)
    {
      ROS_ERROR("No trajectory loaded - cannot set the timestep");
      lock_.unlock();
      return false;
    }

  if(req.td <= 0)
    {
    ROS_ERROR("Timestep has to be larger than 0");
    lock_.unlock();
    return false;
    }

  timestep_=req.td;
  lock_.unlock();

  ROS_INFO("Timestep set to %f",timestep_);
  return true;
}
//-------------------------------------------------------------------
bool TrajectoryServer::replayTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  
  lock_.lock();
  
  unsigned int num_samples=trajectory_parser_->getNumSamples();
  unsigned int init_sample=sample_id_;

  struct timeval start, now; //using a handmade timer since the ros::Rate stuff made problems with the sim_time/real_time distinction
  while (ros::ok() && (sample_id_ < num_samples))
    {
      gettimeofday(&start,0);

#ifdef SENDUPDATE
       publishSendupdate();
#else
       publish();
#endif
      
      sample_id_+=1;
 
      while(1)
  	{
  	  gettimeofday(&now,0);
  	  if((now.tv_sec - start.tv_sec + 0.000001 * (now.tv_usec - start.tv_usec)) >= timestep_)
  	    break;
  	}
    }

  lock_.unlock();

  ROS_INFO("Finished replaying the trajectory from samples %d to %d",init_sample,sample_id_); //CHECK if the final sample is correct or shifted by 1!!!!!!!!!!
  return true;

}
//-------------------------------------------------------------------
Eigen::VectorXd TrajectoryServer::getStateVector(unsigned int sample)
{
  return trajectory_parser_->getTrajectories()->block<number_hand_joints_,1>(0,sample);
}

//------------------------------------------------------------------------------------------------------
bool TrajectoryServer::moveStart(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  lock_.lock();
  if(!traj_loaded_)
    {
      ROS_ERROR("No trajectory loaded - cannot move to start pose");
      lock_.unlock();
      return false;
    }

  sample_id_=0;

#ifdef SENDUPDATE
       publishSendupdate();
#else
       publish();
#endif

  lock_.unlock();

  ROS_INFO("Moved to start pose");
  return true;
}
//------------------------------------------------------------------------------------------------------
#ifdef SENDUPDATE
void TrajectoryServer::publishSendupdate()
{

 Eigen::VectorXd state_vec=getStateVector(sample_id_);
 sr_robot_msgs::sendupdate sud;
 sr_robot_msgs::joint joint;

 sud.sendupdate_length=number_hand_joints_;  
 for (unsigned int i=0; i < number_hand_joints_;i++)
   {
     joint.joint_name=joint_names_[i];
     joint.joint_target=state_vec(i);
     sud.sendupdate_list.push_back(joint);
   } 

 sendupdate_pub_.publish(sud);
}
#else
//------------------------------------------------------------------------------------------------------
void TrajectoryServer::publish()
{
 Eigen::VectorXd state_vec=getStateVector(sample_id_);
      std_msgs::Float64 joint_angle;
      for(unsigned int i=0;i<number_hand_joints_;i++)
  	{
  	  joint_angle.data=state_vec(i)/RAD;
  	  output_pubs_[i].publish(joint_angle);
  	}
}
#endif
//------------------------------------------------------------------------------------------------------
