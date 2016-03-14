#include <iostream>
#include <vector>
#include <signal.h>
#include "sensor_device.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RelativeHumidity.h>
#include "tinkerforge_sensors_core.h"

using std::string;

void sigintHandler(int sig)
{
  ros::shutdown();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "tinkerforge_sensors");
  ros::NodeHandle n;

  // declare variables that can be modified by launch file or command line.
  int rate;
  int imu_convergence_speed;
  int port;
  string host;

  signal(SIGINT, sigintHandler);

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("host", host, string("localhost"));
  private_node_handle_.param("port", port, int(4223));

  // create a new LaserTransformer object.
  TinkerforgeSensors *node_tfs = new TinkerforgeSensors(host, port);
  
  // read sensors config;
  SensorParam param;
  XmlRpc::XmlRpcValue list;

  if (n.getParam("/tfsensors/sensor_conf/", list))
  {
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = list.begin(); it != list.end(); ++it) {
      if (list[it->first].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        //ROS_DEBUG_STREAM("Found string: " << (std::string)(it->first) << " ==> " << static_cast<std::string>(list[it->first]));
        param.type = ParamType::STRING;
        param.value_str = static_cast<std::string>(list[it->first]);
        node_tfs->conf[(std::string)(it->first)]["topic"] = param;
      } else if (list[it->first].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        //ROS_DEBUG_STREAM("Found struct: " << (std::string)(it->first) << " ==> " << list[it->first].getType());
        XmlRpc::XmlRpcValue l2 = it->second; // struct is plugged in second
        for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it2 = l2.begin() ; it2 != l2.end(); ++it2) {
          if (it2->second.getType() == XmlRpc::XmlRpcValue::TypeString) {
            //ROS_DEBUG_STREAM("  o Value:"  << (std::string)it2->first << "::" << static_cast<std::string>(l2[it2->first]) << "::" << it2->second.getType() );
            param.type = ParamType::STRING;
            param.value_str = static_cast<std::string>(l2[it2->first]);
            node_tfs->conf[(std::string)(it->first)][(std::string)it2->first] = param;
          }
		  else if (it2->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            //ROS_DEBUG_STREAM("  Value:"  << (std::string)it2->first << "::" << static_cast<int>(l2[it2->first]) << "::" << it2->second.getType() );
            param.type = ParamType::INT;
            param.value_int = static_cast<int>(l2[it2->first]);
            node_tfs->conf[(std::string)(it->first)][(std::string)it2->first] = param;
          }
		  else if (it2->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            //ROS_DEBUG_STREAM("  Value:"  << (std::string)it2->first << "::" << static_cast<double>(l2[it2->first]) << "::" << it2->second.getType() );
            param.type = ParamType::DOUBLE;
            param.value_double = static_cast<double>(l2[it2->first]);
            node_tfs->conf[(std::string)(it->first)][(std::string)it2->first] = param;
          }
          else {
            ROS_WARN_STREAM("Cound not read parameter " << (std::string)it2->first);
          }
        }
      }
    }
  }
  // init tinkerforge connection
  if (!node_tfs->init())
  {
    return 1;
  }

  // set node rate
  ros::Rate r(rate);

  // sleep a second for init sensors
  ros::Duration(1.0).sleep();

  // create publishers
  std::list<SensorDevice*>::iterator Iter;
  for (Iter = node_tfs->sensors.begin(); Iter != node_tfs->sensors.end(); ++Iter)
  {
    ROS_DEBUG_STREAM("node" << "::" << (*Iter)->getUID() << "::" << (*Iter)->getTopic());
    //std::cout << "node::" << "::" << (*Iter)->getUID() << "::" << (*Iter)->getTopic() << std::endl;

    switch((*Iter)->getType())
    {
      case AMBIENT_LIGHT_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Illuminance>((*Iter)->getTopic().c_str(), 50));
      break;
      case AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Illuminance>((*Iter)->getTopic().c_str(), 50));
      case DISTANCE_IR_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Range>((*Iter)->getTopic().c_str(), 50));
      break;
      case DISTANCE_US_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Range>((*Iter)->getTopic().c_str(), 50));
      break;
      case GPS_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::NavSatFix>((*Iter)->getTopic().c_str(), 50));
      break;
      case HUMIDITY_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::RelativeHumidity>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Imu>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_V2_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Imu>((*Iter)->getTopic().c_str(), 50));
      break;
      case TEMPERATURE_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Temperature>((*Iter)->getTopic().c_str(), 50));
      break;
      case TEMPERATURE_IR_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Temperature>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_V2_MAGNETIC_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::MagneticField>((*Iter)->getTopic().c_str(), 50));
      break;
    }
  }

  while (n.ok())
  {
    node_tfs->publishSensors();
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO_STREAM("Shutdown node ...!");

  // clean up
  if (node_tfs != NULL)
    delete node_tfs;

  return 0;
}
