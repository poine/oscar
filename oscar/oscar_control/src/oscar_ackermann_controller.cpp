#include <oscar_control/oscar_ackermann_controller.h>

//#define DISABLE_MOTORS
namespace oscar_controller {

#define __NAME "oscar_ackermann_controller"
#define WHEEL_BASE   0.1
#define WHEEL_RADIUS 0.03
#define VELOCITY_ROLLING_WINDOW_SIZE 10
  
  OscarAckermannController::OscarAckermannController():
    left_wheel_duty_(0.)
    ,right_wheel_duty_(0.)
    ,steering_angle_(0.) {
    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::OscarAckermannController...");
  }

  OscarAckermannController::~OscarAckermannController() {

  }

  bool OscarAckermannController::init(hardware_interface::RobotHW* hw,
				    ros::NodeHandle& root_nh,
				    ros::NodeHandle& controller_nh)
  {
    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::init");
    hw_ = static_cast<OscarHardwareInterface*>(hw);
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");
    hardware_interface::PositionJointInterface* e2 = hw->get<hardware_interface::PositionJointInterface>();
    steering_joint_ = e2->getHandle("steering_joint");

    input_manager_.init(hw, controller_nh);
    odometry_.init(WHEEL_BASE, VELOCITY_ROLLING_WINDOW_SIZE);
    publisher_.init(root_nh, controller_nh);
    return true;
  }

  void OscarAckermannController::starting(const ros::Time& now) {
     ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::starting");
     odometry_.starting(now);
     publisher_.starting(now);
     hw_->switch_motors_on();
  }

  void OscarAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    input_manager_.update(now);
    odometry_.update(left_wheel_joint_.getVelocity(), right_wheel_joint_.getVelocity(), steering_joint_.getPosition(), now);

    
    
    left_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    right_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    steering_joint_.setCommand(input_manager_.rt_commands_.ang);

    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
  }

  void OscarAckermannController::stopping(const ros::Time&) {

  }


  void OscarAckermannController::compute_control(const ros::Time&) {
    double virtual_steering_angle = input_manager_.rt_commands_.lin * tan(input_manager_.rt_commands_.ang) / WHEEL_BASE;
    steering_angle_ = virtual_steering_angle; // mechanics is supposedly doing the trick...
    left_wheel_vel = 0.;
    
  }


  
  PLUGINLIB_EXPORT_CLASS(oscar_controller::OscarAckermannController, controller_interface::ControllerBase);
}
