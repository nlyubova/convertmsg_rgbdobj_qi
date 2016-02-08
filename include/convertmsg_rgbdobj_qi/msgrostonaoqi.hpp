#ifndef MSGROSTONAOQI_H
#define MSGROSTONAOQI_H

#include <visualization_msgs/MarkerArray.h>

#include <alcommon/albroker.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alvisionrecognitionproxy.h>
#include <qi/session.hpp>

#include <boost/program_options.hpp>

#include "ros/ros.h"

class Msgrostonaoqi
{
public:
  Msgrostonaoqi(std::string pip, std::string ip, int port, int pport);
  virtual ~Msgrostonaoqi();
  void init(); //int argc, char ** argv);
  void parse_command_line(int argc, char ** argv);
  bool connectNaoQi();
  bool connectProxy();
  bool connectTablet();

  void notify_names(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void notify_boxes(const visualization_msgs::MarkerArray::ConstPtr& msg);

protected:
  ros::NodeHandle nh_;

  std::string pip_;
  std::string ip_;
  int port_;
  int pport_;
  std::string brokerName_;

  /*std::string topics_ns_;
  std::vector<std::string> topics_names_;
  std::vector<std::string> events_names_;*/
  std::string topic_obj_names_;
  std::string topic_obj_boxes_;
  std::string event_obj_names_;
  std::string event_obj_boxes_;

  bool initialized_naoqi;
  std::string package_name_;

  ros::Subscriber sub_names_, sub_boxes_;

  boost::shared_ptr<AL::ALBroker> m_broker;
  boost::shared_ptr<AL::ALMemoryProxy> pMemoryProxy;

  boost::shared_ptr<AL::ALProxy> pVRProxy;
};

#endif // MSGROSTONAOQI_H
