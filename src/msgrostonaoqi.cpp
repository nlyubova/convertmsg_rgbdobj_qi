#include "msgrostonaoqi.hpp"

#include <string>
#include <iostream>
#include <sstream>

#include <alvalue/alvalue.h>
#include <qi/log.hpp>

//! @brief Define Log category
qiLogCategory("rgbdobj_tonaoqi");

Msgrostonaoqi::Msgrostonaoqi(const std::string &pip, const std::string &ip,
                             const int &port, const int &pport):
  nh_("~"),
  pip_(pip),
  ip_(ip),
  port_(port),
  pport_(pport),
  brokerName_("NaoROSBroker"),
  topic_obj_names_("/rgb_d_object_ext/objects_names"),
  topic_obj_boxes_("/rgb_d_object_ext/objects_bounding_boxes"),
  event_obj_names_(topic_obj_names_),
  event_obj_boxes_(topic_obj_boxes_),
  initialized_naoqi(false),
  initialized_tablet(false),
  package_name_("rgbdobj_tonaoqi")
{
  initialized_naoqi = false;

  if (pip_.empty())
    nh_.getParam("nao_ip", pip_);
  if (ip_.empty())
    nh_.getParam("pc_ip", ip_);
  nh_.getParam("nao_port", pport_);
  nh_.getParam("pc_port", port_);

  nh_.getParam("topic_obj_names", topic_obj_names_);
  nh_.getParam("topic_obj_boxes", topic_obj_boxes_);

  ROS_INFO_STREAM("Connecting to " << pip_ << " " << pport_);

  init();
}

Msgrostonaoqi::~Msgrostonaoqi()
{
  try
  {
    pTabletProxy->callVoid("showImage", "http://198.18.0.1/tmp/temp.jpg");
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }
}

void Msgrostonaoqi::init()
{
  //initialize the NAOqi proxies and events
  if (!connectNaoQi() || !connectProxy())
    ROS_ERROR("Could not connect to NAO proxy");
  else
  {
    initialized_tablet = connectTablet();
    initialized_naoqi = true;
  }

  try
  {
    pMemoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(m_broker));

    ROS_INFO_STREAM("notifying the following Naoqi events: ");

    pMemoryProxy->declareEvent(topic_obj_names_);
    ROS_INFO_STREAM("  " << topic_obj_names_);
    pMemoryProxy->declareEvent(topic_obj_boxes_);
    ROS_INFO_STREAM("  " << topic_obj_boxes_);

    //initialize the ros subscriber
    sub_names_ = nh_.subscribe<visualization_msgs::MarkerArray>(topic_obj_names_, 10, &Msgrostonaoqi::notify_names, this);
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }
}

void Msgrostonaoqi::notify_names(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  visualization_msgs::MarkerArray::_markers_type::const_iterator it_o = msg->markers.begin();
  while (it_o != msg->markers.end())
  {
    std::vector <float> position;
    position.push_back(it_o->pose.position.x);
    position.push_back(it_o->pose.position.y);
    position.push_back(it_o->pose.position.z);
    std::vector <float> orientation;
    orientation.push_back(it_o->pose.orientation.x);
    orientation.push_back(it_o->pose.orientation.y);
    orientation.push_back(it_o->pose.orientation.z);
    orientation.push_back(it_o->pose.orientation.w);

    std::string object("");
    if (it_o->text.length()>1)
      object = it_o->text.substr(0, it_o->text.length()-1);

    if (initialized_naoqi)
    {
      AL::ALValue valOutcome;
      valOutcome.arrayPush(it_o->text);
      valOutcome.arrayPush(position);
      valOutcome.arrayPush(orientation);

      pMemoryProxy->raiseMicroEvent(event_obj_names_, valOutcome);
      //std::cout << "Publishing the Naoqi event " << event_obj_names_ << " : " << valOutcome << std::endl;
    }

    //show an image
    if (initialized_tablet)
    try
    {
      //ROS_INFO_STREAM("visualize the image: http://198.18.0.1/tmp/"+object+".jpg");
      pTabletProxy->callVoid("showImage", "http://198.18.0.1/tmp/"+object+".jpg");
    }
    catch (const AL::ALError& e)
    {
       ROS_ERROR("Cannot execute ALTabletService.showImage");
    }

    ++it_o;
  }
}

bool Msgrostonaoqi::connectNaoQi()
{
  // Need this to for SOAP serialization of floats to work
  setlocale(LC_NUMERIC, "C");
  try
  {
    m_broker = AL::ALBroker::createBroker(brokerName_, ip_, port_, pip_, pport_, false);
  }
  catch(const AL::ALError& e)
  {
    ROS_ERROR( "Failed to connect broker to: %s:%d", pip_.c_str(), port_);
    return false;
  }
  ROS_INFO("NAOqi broker ready");
  return true;
}

bool Msgrostonaoqi::connectProxy()
{
  if (!m_broker)
  {
     ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
     return false;
  }
  try
  {
    pMemoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(m_broker));
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Could not create ALMemoryProxy.");
    return false;
  }
  ROS_INFO("Proxy to ALMemory is ready");

  return true;
}

bool Msgrostonaoqi::connectTablet()
{
  /*qi::SessionPtr session;
  qi::AnyObject _tabletService;
  try
  {
    session = qi::makeSession();
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Could not create a session.");
    return false;
  }

  try
  {
    std::stringstream strstr;
    strstr << pip_.c_str() << ":" << pport_;
    session->connect(strstr.str().c_str());
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Could not create a session -- 189");
    return false;
  }

  try
  {
    std::string _tabletServiceName("ALTabletService");
    //boost::mutex::scoped_lock lock(_mutex);
    if (!_tabletService)
    {
      //session->waitForService(_tabletServiceName);
      _tabletService = session->service(_tabletServiceName);
    }
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Cannot connect to ALTabletService");
    return false;
  }
  ROS_INFO("Proxy to ALTabletService is ready");

  try
  {
    if (_tabletService)
    {
      _tabletService.call<void>("showImage", "http://198.18.0.1/img/help_charger.png");
    }
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Cannot execute ALTabletService.showImage");
  }*/

  //---------------------------------------------

  //connnect to ALTabletService
  try
  {
    pTabletProxy = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(m_broker,"ALTabletService"));
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Cannot connect to ALTabletService");
    return false;
  }
  ROS_INFO("Proxy to ALTabletService is ready");

  return true;
}
