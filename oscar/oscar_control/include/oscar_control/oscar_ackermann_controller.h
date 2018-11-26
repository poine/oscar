#ifndef OSCAR_CONTROL__OSCAR_ACKERMANN_CONTROLLER_H
#define OSCAR_CONTROL__OSCAR_ACKERMANN_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <pluginlib/class_list_macros.h>

#include "roboticscape.h"

#include "oscar_control/oscar_hardware_interface.h"
#include "oscar_control/input_manager.h"
#include "oscar_control/odometry.h"
#include "oscar_control/publisher.h"
#include "oscar_control/oac_raw_odom_publisher.h"


namespace oscar_controller {

  class OscarAckermannController :
    public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface, hardware_interface::ImuSensorInterface>
    {
    public:
      OscarAckermannController();
      ~OscarAckermannController();
      
      bool init(hardware_interface::RobotHW* hw,
		ros::NodeHandle& root_nh,
		ros::NodeHandle &controller_nh);
      void starting(const ros::Time& time);
      void update(const ros::Time& , const ros::Duration&);
      void stopping(const ros::Time&);

    private:
      hardware_interface::JointHandle left_wheel_joint_;
      hardware_interface::JointHandle right_wheel_joint_;
      hardware_interface::JointHandle steering_joint_;
      // we keep a pointer on it for non standard stuff like radio control and motors on/off
      // that sucks... FIXME
      OscarHardwareInterface* hw_;
      oscar_controller::InputManager   input_manager_;
      oscar_controller::Odometry       odometry_;
      oscar_controller::Publisher      publisher_;
      oscar_controller::OACRawOdomPublisher raw_odom_publisher_;

      // measured values
      double left_wheel_rvel_, right_wheel_rvel_;
      // pid structures
      rc_filter_t left_wheel_pid_, right_wheel_pid_;
      // values output to the hardware interface
      double left_wheel_duty_, right_wheel_duty_, steering_angle_, steering_servo_;
      
      void compute_control(const ros::Time& now);

      
    };
}

#endif // OSCAR_CONTROL__OSCAR_ACKERMANN_CONTROLLER_H
