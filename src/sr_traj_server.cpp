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
#include <vector>
#include <sys/time.h>
#include <time.h>

const unsigned int TrajectoryServer::number_hand_joints_ = 20;
//-------------------------------------------------------------------
TrajectoryServer::TrajectoryServer() : nh_private_("~"), traj_loaded_(false), delta_t_(1e-3)
{
  initJointNames();


  std::string searched_param;
  std::string traj_dir;
  nh_private_.searchParam("trajectory_dir", searched_param);
  nh_private_.param(searched_param, traj_dir, std::string());
  trajectory_parser_.reset(new TrajectoryParser(traj_dir));

  ROS_INFO("Trajectory directory set to: %s", traj_dir.c_str());

  XmlRpc::XmlRpcValue grasp_config;
  std::vector<int> finger_joints;
  std::vector<int> grasp_joints;
  if (nh_private_.searchParam("grasp_config", searched_param))
    {
      nh_.getParam(searched_param,grasp_config);
      ROS_ASSERT(grasp_config.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
      for (int32_t i = 0; i < grasp_config.size(); ++i) 
	{    
	  ROS_ASSERT(grasp_config[i]["finger_joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
	  ROS_ASSERT(grasp_config[i]["grasp_joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);
          ROS_ASSERT(grasp_config[i]["sensor_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
          ROS_ASSERT(grasp_config[i]["force_threshold"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

          for (int32_t j = 0; j <grasp_config[i]["finger_joints"].size();j++) 
	    {
              ROS_ASSERT(grasp_config[i]["finger_joints"][j].getType() == XmlRpc::XmlRpcValue::TypeString);
              finger_joints.push_back(getJointId((std::string)grasp_config[i]["finger_joints"][j]));
            }
          for (int32_t j = 0; j <grasp_config[i]["grasp_joints"].size();j++) 
	    {
              ROS_ASSERT(grasp_config[i]["grasp_joints"][j].getType() == XmlRpc::XmlRpcValue::TypeString);
              grasp_joints.push_back(getJointId((std::string)grasp_config[i]["grasp_joints"][j]));
            }

          fingers_.push_back(new Finger(finger_joints,grasp_joints,(std::string)grasp_config[i]["sensor_topic"],(double)grasp_config[i]["force_threshold"],trajectory_parser_));
        

	  finger_joints.clear();
          grasp_joints.clear();
	}
    }
  else
    {
      ROS_ERROR("The Grasp configuration is not specified - cannot start the Trajectory Server");
      exit(0);
    }

  initContactFingers();//By default, all fingers have to come into contact in order to establish a grasp
    

  replay_traj_srv_ = nh_.advertiseService("replay_traj",&TrajectoryServer::replayTrajectory,this);
  grasp_srv_ = nh_.advertiseService("grasp",&TrajectoryServer::grasp,this);
  reset_hand_srv_ = nh_.advertiseService("reset_hand",&TrajectoryServer::resetHand,this);
  shadowhand_pub_ = nh_.advertise<sr_robot_msgs::sendupdate> ("sendupdate", 1); //queue size of only one - maybe change that
}
//-------------------------------------------------------------------
TrajectoryServer::~TrajectoryServer()
{
  for(unsigned int i=0;i<fingers_.size();i++)
    delete fingers_[i];
}
//-------------------------------------------------------------------
void TrajectoryServer::initContactFingers()
{
  contact_fingers_.clear();
  for(unsigned int i=0;i<fingers_.size();i++)
    contact_fingers_.push_back(i);
}
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
int TrajectoryServer::getJointId(std::string const & joint_name)
{
 int id=-1;

  for(unsigned int i=0; i<number_hand_joints_;i++)
    if(!strcmp(joint_names_[i].c_str(),joint_name.c_str()))
      {
	id=(int)i;
	break;
      }

  ROS_ASSERT(id >= 0);

  return id;
}
//-------------------------------------------------------------------
sr_robot_msgs::sendupdate TrajectoryServer::generateMessage(Eigen::VectorXd const & state_vec)
{
  sr_robot_msgs::joint joint;
  sr_robot_msgs::sendupdate msg;
   
  //Generate sendupdate message
  std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
  for(unsigned int i = 0; i < number_hand_joints_; ++i )
    {
      joint.joint_name = joint_names_[i];
      joint.joint_target = state_vec(i);
	table[i] = joint;
    }

  msg.sendupdate_list = table;
  msg.sendupdate_length = number_hand_joints_;

  return msg;
}
//-------------------------------------------------------------------
bool TrajectoryServer::replayTrajectory(sr_traj_server::replay_traj::Request &req, sr_traj_server::replay_traj::Response &res)
{
  lock_.lock();
  res.success=false;
 
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

  initContactFingers();
  if(!req.contact_fingers.empty())
    {
      contact_fingers_.clear();
      for (unsigned int i=0; i< req.contact_fingers.size();i++)
	{
	  if(!strcmp(req.contact_fingers[i].c_str(),"TH"))
            contact_fingers_.push_back(THUMB);
          else if(!strcmp(req.contact_fingers[i].c_str(),"FF"))
            contact_fingers_.push_back(FOREFINGER);
	  else if(!strcmp(req.contact_fingers[i].c_str(),"MF"))
            contact_fingers_.push_back(MIDDLEFINGER);
	  else if(!strcmp(req.contact_fingers[i].c_str(),"RF"))
            contact_fingers_.push_back(RINGFINGER);
	  else if(!strcmp(req.contact_fingers[i].c_str(),"LF"))
            contact_fingers_.push_back(LITTLEFINGER);
	  else
	    ROS_WARN("%s is an unknown contact finger specification. It will be ignored",req.contact_fingers[i].c_str());
	}
    }

  for(unsigned int i=0;i<fingers_.size();i++)
    fingers_[i]->resetTrajectories();
  
  delta_t_=trajectory_parser_->getTimestep(); 

  traj_loaded_=true;

  res.success=true;
  lock_.unlock();
  return res.success;
}
//------------------------------------------------------------------------------------------------------
void TrajectoryServer::spin()
{
  struct timeval start, now;
  gettimeofday(&start,0);

  if(!traj_loaded_)
    ;
  else if(completed())
    {
      traj_loaded_=false;
      ROS_INFO("Trajectory completed");
      delta_t_=1e-3;
    }
  else if(contactsEstablished())
    ROS_INFO("Grasp contacts established - ready to grasp");  
  else
    {
      for(unsigned int i=0; i < fingers_.size();i++)
  	fingers_[i]->incrementJointStates();


      followTrajectories();

    }

  ros::spinOnce();

  while(1)
    {
      gettimeofday(&now,0);
      if((now.tv_sec - start.tv_sec + 0.000001 * (now.tv_usec - start.tv_usec)) >= delta_t_)
	break;

    }
}
//------------------------------------------------------------------------------------------------------
void TrajectoryServer::followTrajectories()
{
  Eigen::VectorXd state_vec(number_hand_joints_);
  std::map<int,double> joint_states;
  std::map<int,double>::iterator joint_states_it;      

  for(unsigned int i=0; i < fingers_.size();i++)
    {
      joint_states=fingers_[i]->getJointStates();
           
      for ( joint_states_it=joint_states.begin() ; joint_states_it != joint_states.end(); joint_states_it++ )
        	state_vec((*joint_states_it).first)=(*joint_states_it).second;

    }

  //EVIL HACK: freeze the wrist joints to zero since their trajectories are not read by any finger
  state_vec(19)=0;
  state_vec(18)=0;
 
  shadowhand_pub_.publish(generateMessage(state_vec));
}
//------------------------------------------------------------------------------------------------------
bool TrajectoryServer::contactsEstablished()
{
  bool all_touching = true;
  for(unsigned int i=0; i < contact_fingers_.size();i++)
    if(!fingers_[contact_fingers_[i]]->isTouching())
      {
	all_touching = false;
	break;
      }

  return all_touching;
}
//------------------------------------------------------------------------------------------------------
bool TrajectoryServer::completed()
{
  bool completed = true;
  for(unsigned int i=0; i < fingers_.size();i++)
    if(!fingers_[i]->trajCompleted())
      {
	completed= false;
	break;
      }

  return completed;
}
//------------------------------------------------------------------------------------------------------
bool TrajectoryServer::grasp(sr_traj_server::Grasp::Request &req, sr_traj_server::Grasp::Response &res)
{
  //DOES NOT WORK
  res.success=false;
 
  if(!traj_loaded_)
    {
      ROS_ERROR("No trajectory loaded - cannot execute grasp");
      return res.success;
    }

  for(unsigned int i=0; i < contact_fingers_.size();i++)
	fingers_[contact_fingers_[i]]->incrementJointStates(req.traj_inc);

  followTrajectories();

  res.success=true;
 
  return res.success;
}
//------------------------------------------------------------------------------------------------------
bool TrajectoryServer::resetHand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  lock_.lock();
  traj_loaded_=false;

  for(unsigned int i=0;i<fingers_.size();i++)
    fingers_[i]->resetTrajectories();

  lock_.unlock();

  Eigen::VectorXd state_vec(number_hand_joints_);

  state_vec.setZero(); state_vec(7)=-3; state_vec(16)=-3;
  shadowhand_pub_.publish(generateMessage(state_vec));
  return true;
}
//------------------------------------------------------------------------------------------------------
