#include <oscar_control/oscar_ackermann_controller.h>

//#define DISABLE_MOTORS
namespace oscar_controller {

#define __NAME "oscar_ackermann_controller"
  
  OscarAckermannController::OscarAckermannController() {
    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::OscarAckermannController...");
  }

  OscarAckermannController::~OscarAckermannController() {

  }

  bool OscarAckermannController::init(hardware_interface::RobotHW* hw,
				    ros::NodeHandle& root_nh,
				    ros::NodeHandle& controller_nh)
  {
    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::init...");
    hw_ = static_cast<OscarHardwareInterface*>(hw);
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");
    hardware_interface::PositionJointInterface* e2 = hw->get<hardware_interface::PositionJointInterface>();
    steering_joint_ = e2->getHandle("steering_joint");

    input_manager_.init(hw, controller_nh);
    
    return true;
  }

  void OscarAckermannController::starting(const ros::Time& now) {
     ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::starting...");
     hw_->switch_motors_on();
  }

  void OscarAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    input_manager_.update(now);
    //double t = now.toSec();
    //double d = sin(t)*0.2;
    //ROS_INFO(" update setCommand %f", d);
    left_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    right_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    steering_joint_.setCommand(input_manager_.rt_commands_.ang);
  }

  void OscarAckermannController::stopping(const ros::Time&) {

  }


  PLUGINLIB_EXPORT_CLASS(oscar_controller::OscarAckermannController, controller_interface::ControllerBase);
}
