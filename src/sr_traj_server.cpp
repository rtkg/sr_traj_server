/**
 * @file   sr_traj_server.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
 */

#include <ros/ros.h>
#include <string>
#include "sr_traj_server/sr_traj_server.h"
#include <sr_robot_msgs/joint.h>
#include <sys/time.h>
#include <time.h>

const unsigned int TrajectoryServer::number_hand_joints_ = 20;
//-------------------------------------------------------------------
TrajectoryServer::TrajectoryServer() : nh_private_("~")
{
    std::string param;
    std::string traj_dir;
    nh_private_.searchParam("trajectory_dir", param);
    nh_private_.param(param, traj_dir, std::string());
    trajectory_parser_ = new TrajectoryParser(traj_dir); //can create a stack smash
    
    ROS_INFO("Trajectory directory set to: %s", traj_dir.c_str());

    std::string sendupdate_prefix;
    nh_private_.searchParam("sendupdate_prefix", param);
    nh_private_.param(param, sendupdate_prefix, std::string());

    std::string server_prefix;
    nh_private_.searchParam("server_prefix", param);
    nh_private_.param(param, server_prefix, std::string());
 
    initJointNames();

    replay_traj_srv_ = nh_.advertiseService(server_prefix + "replay_traj",&TrajectoryServer::replayTrajectory,this);
    shadowhand_pub_ = nh_.advertise<sr_robot_msgs::sendupdate> (sendupdate_prefix + "sendupdate", 1); //queue size of only one - maybe change that
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
sr_robot_msgs::sendupdate TrajectoryServer::generateMessage(Eigen::VectorXd & state_vec)
{
  sr_robot_msgs::joint joint;
  sr_robot_msgs::sendupdate msg;
   
  //Generate sendupdate message
  std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
  for(unsigned int i = 0; i < number_hand_joints_; ++i )
    {
      joint.joint_name = joint_names_[i];
      joint.joint_target = state_vec.coeff(i);
      table[i] = joint;
    }

  msg.sendupdate_list = table;
  msg.sendupdate_length = number_hand_joints_;

  return msg;
}
//-------------------------------------------------------------------
bool TrajectoryServer::replayTrajectory(sr_traj_server::replay_traj::Request &req, sr_traj_server::replay_traj::Response &res)
{
  data_mutex_.lock();
  res.success=false;
 
  //Load the trajectory in the parser
  if(!trajectory_parser_->parseFile(req.file))
    {
      ROS_ERROR("Could not parse file %s",(trajectory_parser_->getTrajDir()+req.file).c_str());
      data_mutex_.unlock();
      return res.success;
    }

  if(!trajectory_parser_->getNumTraj()==number_hand_joints_)
    {
      ROS_ERROR("%d number of joint trajectories need to be specified",number_hand_joints_);
      data_mutex_.unlock();
      return res.success;
    }
  
  Eigen::VectorXd state_vec(number_hand_joints_);
  unsigned int sample_id=0;
  unsigned int num_samples=trajectory_parser_->getNumSamples();

  struct timeval start, now; //using a handmade timer since the ros::Rate stuff made problems with the sim_time/real_time distinction
  gettimeofday(&start,0);
  while (ros::ok() && (sample_id < num_samples))
    {
      state_vec=getStateVector(sample_id);
      shadowhand_pub_.publish(generateMessage(state_vec));
      sample_id+=1;

      while(1)
	{
	  gettimeofday(&now,0);
	  if((now.tv_sec - start.tv_sec + 0.000001 * (now.tv_usec - start.tv_usec)) >= trajectory_parser_->getTimestep())
	    break;
	}

      //std::cout<<r.cycleTime()<<std::endl;
    }

  res.success=true;
  data_mutex_.unlock();
  return res.success;
}
//-------------------------------------------------------------------
Eigen::VectorXd TrajectoryServer::getStateVector(unsigned int sample)
{
  return trajectory_parser_->getTrajectories()->block<number_hand_joints_,1>(0,sample);
}
//-------------------------------------------------------------------



