#ifndef OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H
#define OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <robotcontrol.h>

#define NB_JOINTS 3

class OscarHardwareInterface : public hardware_interface::RobotHW
{
 public:
  OscarHardwareInterface();
  virtual ~OscarHardwareInterface();

  bool start();
  void read(ros::Time now);
  void write();
  bool shutdown();
  void switch_motors_on()  { rc_motor_standby(0); }
  void switch_motors_off() { rc_motor_standby(1); }
  
  void IMUCallback(void);
 
 private:
  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_effort_command_[NB_JOINTS];
  double joint_position_command_[NB_JOINTS];
 
  // IMU
  hardware_interface::ImuSensorHandle::Data imu_data_;
  double imu_orientation_[4];         // quat is in the order of geometry_msg, ie x, y, z, w
  double imu_angular_velocity_[3];
  double imu_linear_acceleration_[3];

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::ImuSensorInterface     imu_sensor_interface_;

  rc_mpu_data_t rc_mpu_data_;
  // Mechanics
  double gear_enc_res_;
  // Timing
  ros::Time last_read_stamp_;
  ros::Duration last_period_;
};


#endif // OSCAR_CONTROL__OSCAR_HARDWARE_INTERFACE_H
