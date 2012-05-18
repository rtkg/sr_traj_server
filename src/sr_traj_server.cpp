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

const unsigned int TrajectoryServer::number_hand_joints_ = 20;
//-------------------------------------------------------------------
TrajectoryServer::TrajectoryServer() : nh_private_("~"),traj_loaded_(false),sample_id_(0)
{
    std::string searched_param;
    std::string traj_dir;
    nh_private_.searchParam("trajectory_dir", searched_param);
    nh_private_.param(searched_param, traj_dir, std::string());
    trajectory_parser_ = new TrajectoryParser(traj_dir); //can create a stack smash
    
    ROS_INFO("Trajectory directory set to: %s", traj_dir.c_str());

  XmlRpc::XmlRpcValue output_topics;
  if (nh_private_.searchParam("output_topics", searched_param))
    {
      nh_.getParam(searched_param,output_topics);
      ROS_ASSERT(output_topics.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
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

    initJointNames();


    load_traj_srv_ = nh_.advertiseService("load_trajectory",&TrajectoryServer::loadTrajectory,this);
    step_traj_srv_ = nh_.advertiseService("step_trajectory",&TrajectoryServer::stepTrajectory,this);
    replay_traj_srv_ = nh_.advertiseService("replay_trajectory",&TrajectoryServer::replayTrajectory,this);
    reset_hand_srv_ = nh_.advertiseService("reset_hand",&TrajectoryServer::resetHand,this);   
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
    return true;
  }
//-------------------------------------------------------------------
bool TrajectoryServer::replayTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  

  
  // Eigen::VectorXd state_vec(number_hand_joints_);
  // unsigned int sample_id=0;
  // unsigned int num_samples=trajectory_parser_->getNumSamples();

  // if((req.n_samples == 0) || (req.n_samples > num_samples))
  //   ROS_WARN("Invalid sample number specified. Using %d samples.",num_samples);
  // else
  //   num_samples=req.n_samples;

  // struct timeval start, now; //using a handmade timer since the ros::Rate stuff made problems with the sim_time/real_time distinction
  // while (ros::ok() && (sample_id < num_samples))
  //   {
  //     gettimeofday(&start,0);

  //     state_vec=getStateVector(sample_id);

  //     std_msgs::Float64 joint_angle;
  //     for(unsigned int i=0;i<output_pubs_.size();i++)
  // 	{
  // 	  joint_angle.data=state_vec(i)/RAD;
  // 	  output_pubs_[i].publish(joint_angle);
  // 	}
  //     sample_id+=1;
 
  //     while(1)
  // 	{
  // 	  gettimeofday(&now,0);
  // 	  if((now.tv_sec - start.tv_sec + 0.000001 * (now.tv_usec - start.tv_usec)) >= trajectory_parser_->getTimestep())
  // 	    break;
  // 	}
  //   }
  return true;


}
//-------------------------------------------------------------------
Eigen::VectorXd TrajectoryServer::getStateVector(unsigned int sample)
{
  return trajectory_parser_->getTrajectories()->block<number_hand_joints_,1>(0,sample);
}
//-------------------------------------------------------------------
bool TrajectoryServer::resetHand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::vector<std_msgs::Float64> init_pos(number_hand_joints_);
  init_pos[0].data=1/RAD;
  init_pos[1].data=1/RAD;
  init_pos[2].data=1/RAD;
  init_pos[3].data=1/RAD;
  init_pos[4].data=1/RAD;
  init_pos[5].data=1/RAD;
  init_pos[6].data=1/RAD;
  init_pos[7].data=-5/RAD;
  init_pos[8].data=1/RAD;
  init_pos[9].data=1/RAD;
  init_pos[10].data=-1/RAD;
  init_pos[11].data=1/RAD;
  init_pos[12].data=1/RAD;
  init_pos[13].data=-1/RAD;
  init_pos[14].data=1/RAD;
  init_pos[15].data=1/RAD;
  init_pos[16].data=-1/RAD;
  init_pos[17].data=1/RAD;
  init_pos[18].data=1/RAD;
  init_pos[19].data=1/RAD;

  lock_.lock();
 
  for(unsigned int i=0;i<output_pubs_.size();i++)
    output_pubs_[i].publish(init_pos[i]);

  lock_.unlock();

  ROS_INFO("Hand reseted");
  return true;
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

  Eigen::VectorXd start=getStateVector(0);
  std_msgs::Float64 joint_angle;
  for(unsigned int i=0;i<output_pubs_.size();i++)
    {
      joint_angle.data=start(i);
      output_pubs_[i].publish(joint_angle);
    }
  lock_.unlock();

  ROS_INFO("Moved to start pose");
  return true;
}
//------------------------------------------------------------------------------------------------------


