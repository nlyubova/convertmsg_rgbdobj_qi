#include "msgrostonaoqi.hpp"

#include <boost/program_options.hpp>

/**
  launch from the command line
    ./ork/devel/lib/convertmsg_orktonaoqi/./convertmsg_orktonaoqi --pip 10.0.128.63 --pport 9559
  or use the ros.launch file
    roslaunch convertmsg_orktonaoqi convertmsg_orktonaoqi.launch
 */

void parse_command_line(int argc, char ** argv, std::string &m_pip, std::string &m_ip, int &m_port, int &m_pport)
{
  std::string pip;
  std::string ip;
  int pport;
  int port;
  boost::program_options::options_description desc("Configuration");
  desc.add_options()
    ("help", "show this help message")
    ("ip", boost::program_options::value<std::string>(&ip)->default_value(m_ip),
     "IP/hostname of the broker")
    ("port", boost::program_options::value<int>(&port)->default_value(m_port),
     "Port of the broker")
    ("pip", boost::program_options::value<std::string>(&pip)->default_value(m_pip),
     "IP/hostname of parent broker")
    ("pport", boost::program_options::value<int>(&pport)->default_value(m_pport),
     "port of parent broker")
    ;
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  m_port = vm["port"].as<int>();
  m_pport = vm["pport"].as<int>();
  m_pip = vm["pip"].as<std::string>();
  m_ip = vm["ip"].as<std::string>();
  ROS_DEBUG_STREAM("pip is " << m_pip);
  ROS_DEBUG_STREAM("ip is " << m_ip);
  ROS_DEBUG_STREAM("port is " << m_port);
  ROS_DEBUG_STREAM("pport is " << m_pport);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return ;
  }
}

int main(int argc, char **argv)
{
  std::string m_pip = "localhost"; //"nao.local"; //"10.0.163.217"; //
  std::string m_ip = "0.0.0.0";
  int m_port = 9559;
  int m_pport = 9559;
  parse_command_line(argc, argv, m_pip, m_ip, m_port, m_pport);
  //std::cout << "launching with parameters: " << m_pip << " " << m_ip << " " << m_port << " " << m_pport << std::endl;

  //initialize the ros node
  /*std::stringstream strstr;
  strstr << "__master=http://" << m_pip << ":" << m_pport;
  //std::cout << " ---------------------- " << strstr.str() << std::endl;
  char * m_master = &strstr.str()[0];
  char *argv_l[] = {"convertmsg_orktonaoqi", m_master, NULL};
  int argc_l = 2;
  ros::init(argc_l, argv_l, "convertmsg_orktonaoqi");*/
  ros::init(argc, argv, "convertmsg_orktonaoqi");

  Msgrostonaoqi wrapper(m_pip, m_ip, m_port, m_pport);
  //wrapper.init();//argc, argv);
  ros::Rate rate(10);

	while(ros::ok())
	{
  	ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();

  return 0;
}
