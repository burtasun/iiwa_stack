#pragma once


#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>

/*
// ROS headers
//#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_interface.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
//#include <std_msgs/Duration.h>
//#include <urdf/model.h>

#include <pluginlib/class_list_macros.hpp>
*/
#include "TurnTableHandler.h"//wrapper control eje externo
/*
#include <sstream>
#include <vector>
*/

namespace iiwa_hwextaxis
{

class ExtAxisRobotHW : public hardware_interface::RobotHW
{
public:
  //conexion al eje externo, inicializacion de variables y nodo de ROS
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;
  //lectura de posicion actual 
  void read(const ros::Time& time, const ros::Duration& period) override;
  //envio de consigna de posicion
  void write(const ros::Time& time, const ros::Duration& period) override;
  //requerido por interfaz!
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  //requerido por interfaz!
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

private:
  /*Gestor eje externo*/
  TurnTableHandler tth_;
  /* Node handle */
  ros::NodeHandle nh_;

  /* Parameters */
  std::string robot_name_{""}, interface_{""}, movegroup_name_{""};
  
  hardware_interface::JointStateInterface state_interface_;       /**< Interface for joint state */
  hardware_interface::PositionJointInterface position_interface_; /**< Interface for joint position control */

  std::string joint_name_;
  double joint_position_;
  double joint_position_command_;

  double joint_last_position_command_;
  
  /*requerido para state_interface / no se usan*/
  double joint_velocity_;
  double joint_effort_;

};
}  // namespace iiwa_hw

