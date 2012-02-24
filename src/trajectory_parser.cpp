#include <ros/ros.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include "sr_traj_server/trajectory_parser.h"


TrajectoryParser::TrajectoryParser(std::string traj_dir) : joint_traj_(new Eigen::MatrixXd()),traj_dir_(traj_dir),
                                                           delta_t_(0),rows_(0),cols_(0){}
//------------------------------------------------------------------------------------------------------
TrajectoryParser::~TrajectoryParser(){delete joint_traj_;}
//------------------------------------------------------------------------------------------------------
bool TrajectoryParser::parseFile(std::string const & file)
{
  std::ifstream joint_traj_file;
  std::string path=traj_dir_ + file;

  joint_traj_file.open(path.c_str());

  //can't find the file
  if( !joint_traj_file.is_open())
    {
      ROS_ERROR("Couldn't open the file %s", path.c_str());
      return false;
    }
 
  unsigned int row_id=0;
  std::string line;
  std::vector<std::string> splitted_string;
  while( !joint_traj_file.eof() )
    {
      getline(joint_traj_file, line);

      //remove leading and trailing whitespace
      line = boost::algorithm::trim_copy(line);

      //ignore empty line
      if( line.size() == 0 )
	continue;

      //check the headers
      if( line == "#Timestep" )
	{
	  getline(joint_traj_file, line);
	  delta_t_= convertToDouble(line); 
	  continue;
	}
      else if (line == "#Dimensions")
	{
          getline(joint_traj_file, line);
          boost::split(splitted_string, line, boost::is_any_of("\t "));
          splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 )), splitted_string.end()); 
          rows_=(unsigned int)convertToDouble(splitted_string[0]);
          cols_=(unsigned int)convertToDouble(splitted_string[1]);
	  continue;
	}
      else if (line == "#Joint trajectories")
	continue;

      if(rows_==0 || cols_==0 || delta_t_==0)
	{
	  ROS_ERROR("File %s has invalid headers",path.c_str());
          return false;
        }

      joint_traj_->resize(rows_,cols_);      

      boost::split(splitted_string, line, boost::is_any_of("\t "));
      splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 )), splitted_string.end());

      if(splitted_string.size() != cols_)
        {
          ROS_ERROR("Invalid joint trajectory file %s",path.c_str());
	  return false;
        }     

      for( unsigned int col_id = 0; col_id < cols_; ++col_id )
	(*joint_traj_)(row_id,col_id) = convertToDouble(splitted_string[col_id]);

      row_id+=1;
    }
  joint_traj_file.close();

  return true;
}
//------------------------------------------------------------------------------------------------------
Eigen::MatrixXd* TrajectoryParser::getTrajectories(){return joint_traj_;}
//------------------------------------------------------------------------------------------------------
unsigned int TrajectoryParser::getNumTraj(){return rows_;}
//------------------------------------------------------------------------------------------------------
unsigned int TrajectoryParser::getNumSamples(){return cols_;}
//------------------------------------------------------------------------------------------------------
double TrajectoryParser::getTimestep(){return delta_t_;}
//------------------------------------------------------------------------------------------------------
std::string TrajectoryParser::getTrajDir(){return traj_dir_;}
//------------------------------------------------------------------------------------------------------
