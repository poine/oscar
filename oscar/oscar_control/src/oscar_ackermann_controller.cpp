#include <oscar_control/oscar_ackermann_controller.h>

//#define DISABLE_MOTORS
namespace oscar_controller {

#define __NAME "oscar_ackermann_controller"
#define WHEEL_BASE   0.1
#define WHEEL_RADIUS 0.03

#define GEOM_L 0.1
#define GEOM_D 0.1
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

    
#if 0    
    left_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    right_wheel_joint_.setCommand(input_manager_.rt_commands_.lin);
    steering_joint_.setCommand(input_manager_.rt_commands_.ang);
#else
    compute_control(now);
    steering_joint_.setCommand(steering_angle_);
    //steering_joint_.setCommand(30./180.*M_PI);
    left_wheel_joint_.setCommand(left_wheel_duty_);
    right_wheel_joint_.setCommand(right_wheel_duty_);
#endif

    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
  }

  void OscarAckermannController::stopping(const ros::Time&) {

  }


  double precommand(double omc) {
    if (omc >= 0)
      return (omc+2)*0.024;
    else
      return (omc-2)*0.024;
  }

#define DUTY_OF_WHEEL_RVEL(_rv) (_rv*0.03)
  void OscarAckermannController::compute_control(const ros::Time&) {

    double virtual_steering_angle;
    if (input_manager_.rt_commands_.lin == 0)
      virtual_steering_angle = 0.;
    else
      virtual_steering_angle = std::atan(input_manager_.rt_commands_.ang/input_manager_.rt_commands_.lin* GEOM_L);
					 
    steering_angle_ = virtual_steering_angle; // mechanics is supposedly doing the trick...
    double wheels_rvel = input_manager_.rt_commands_.lin/WHEEL_RADIUS;

    double dv = input_manager_.rt_commands_.ang*GEOM_D/2;
    double left_wheel_rvel  = ( input_manager_.rt_commands_.lin - dv ) / WHEEL_RADIUS;
    double right_wheel_rvel = ( input_manager_.rt_commands_.lin + dv ) / WHEEL_RADIUS;
    
    left_wheel_duty_ =  precommand(left_wheel_rvel);
    right_wheel_duty_ = precommand(right_wheel_rvel);
  }


  
  PLUGINLIB_EXPORT_CLASS(oscar_controller::OscarAckermannController, controller_interface::ControllerBase);
}
