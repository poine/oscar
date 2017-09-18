#ifndef OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H
#define OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "roboticscape.h"

class OscarHardwareInterface : public hardware_interface::RobotHW
{
 public:
  OscarHardwareInterface();
  virtual ~OscarHardwareInterface();

  bool start();
  void read();
  void write();
  bool shutdown();

};


#endif // OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H
