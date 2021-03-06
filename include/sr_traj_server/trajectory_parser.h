/**
* @file   trajectory_parser.h
* @author Robert Krug
* @date   Thu May 13 09:44:52 2010
*
* Parser to read 20xn Joint trajectories for the Shadow Hand
*
*/

#ifndef   	trajectory_parser_h_
# define   	trajectory_parser_h_

#include <iostream>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include <fstream>

class TrajectoryParser {

public:

  bool parseFile(std::string const & file);
  Eigen::MatrixXd* getTrajectories();
  unsigned int getNumTraj();
  unsigned int getNumSamples();  
  double getTimestep();
  std::string getTrajDir();

  TrajectoryParser(std::string traj_dir);
  ~TrajectoryParser();

private:

  //Private default constructor - The Parser needs a directory as argument
  TrajectoryParser(){};

  Eigen::MatrixXd* joint_traj_;
  std::string traj_dir_;
  double delta_t_;
  unsigned int rows_, cols_;

  inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad trajectory file: %s", s.c_str());
    return x;
  }
}; // end class

#endif 	   
