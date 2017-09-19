#include "oscar_control/input_manager.h"


namespace oscar_controller {

  InputManager::InputManager() {
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool InputManager::init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh) {

    //hardware_interface::DsmInterface* d = hw->get<hardware_interface::DsmInterface>();
    //dsm_ = d->getHandle("dsm");

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &InputManager::cmdVelCallback, this);

    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/

  void InputManager::update(const ros::Time& now) {
    
    //    if (*dsm_.getOk() and *dsm_.getModeSwitch() > 0.5) {
    //  rt_commands_.lin = *dsm_.getDriveStick() * DRIVE_RATE_ADVANCED;
    //  rt_commands_.ang = *dsm_.getTurnStick() * TURN_RATE_ADVANCED;
    //}
    //else {
    rt_commands_ = *(command_.readFromRT());
    const double dt = (now - rt_commands_.stamp).toSec();
    if (dt > 0.5) {
      rt_commands_.lin = 0.;
      rt_commands_.ang = 0.;
    }
    //else
    //	std::cerr << "ros cmd" << std::endl;
    //}
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void InputManager::cmdVelCallback(const geometry_msgs::Twist& command) {
    nrt_ros_command_struct_.ang   = command.angular.z;
    nrt_ros_command_struct_.lin   = command.linear.x;
    nrt_ros_command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (nrt_ros_command_struct_);
  }
  
}
