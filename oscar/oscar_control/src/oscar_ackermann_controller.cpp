#include <oscar_control/oscar_ackermann_controller.h>

// Directly transmit setpoints to the hardware interface (for calibration)
//#define BYPASS_CONTROL

namespace oscar_controller {

  
#define __NAME "oscar_ackermann_controller"

  // Geometry
#define WHEEL_BASE   0.1
#define WHEEL_RADIUS 0.03
#define GEOM_L 0.1
#define GEOM_D 0.1

  // Odometry
#define VELOCITY_ROLLING_WINDOW_SIZE 10

  // Control
#define WHEEL_KP 0
#define WHEEL_KI 0.01
#define WHEEL_KD 0
#define WHEEL_TF 0.007
#define WHEEL_DT 0.010
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  OscarAckermannController::OscarAckermannController():
    left_wheel_duty_(0.)
    ,right_wheel_duty_(0.)
    ,steering_angle_(0.) {

    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::OscarAckermannController...");
    left_wheel_pid_ = rc_empty_filter();
    right_wheel_pid_ = rc_empty_filter();
    rc_pid_filter(&left_wheel_pid_, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_TF, WHEEL_DT);
    rc_pid_filter(&right_wheel_pid_, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_TF, WHEEL_DT);
    //rc_enable_saturation(&D1_, -1.0, 1.0);
    //rc_enable_soft_start(&D1_, SOFT_START_SEC);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  OscarAckermannController::~OscarAckermannController() {

  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
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
    raw_odom_publisher_.init(root_nh, controller_nh);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void OscarAckermannController::starting(const ros::Time& now) {
    ROS_INFO_STREAM_NAMED(__NAME, "in OscarAckermannController::starting");
    odometry_.starting(now);
    publisher_.starting(now);
    hw_->switch_motors_on();
  }
  
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void OscarAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    left_wheel_rvel_ = left_wheel_joint_.getVelocity();
    right_wheel_rvel_ = right_wheel_joint_.getVelocity();
    
    input_manager_.update(now);
    odometry_.update(left_wheel_rvel_, right_wheel_rvel_, steering_joint_.getPosition(), now);
    
    
#ifdef BYPASS_CONTROL
    left_wheel_duty_  = input_manager_.rt_commands_.lin;
    right_wheel_duty_ = input_manager_.rt_commands_.lin;
    steering_angle_   = input_manager_.rt_commands_.ang;
#else
    compute_control(now);
#endif
    steering_joint_.setCommand(steering_servo_);
    left_wheel_joint_.setCommand(left_wheel_duty_);
    right_wheel_joint_.setCommand(right_wheel_duty_);
    
    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
    double left_wheel_angle = left_wheel_joint_.getPosition();
    double right_wheel_angle = right_wheel_joint_.getPosition();
    raw_odom_publisher_.publish(left_wheel_angle, right_wheel_angle, steering_angle_, now);
  }
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void OscarAckermannController::stopping(const ros::Time&) {

  }


  double motor_precommand(double omc) {
    if (omc >= 0)
      return (omc+2)*0.024;
    else
      return (omc-2)*0.024;
  }

#define DUTY_OF_WHEEL_RVEL(_rv) (_rv*0.03)
#define STS_A1  0.938104
#define STS_A0 -0.138223
  
  void OscarAckermannController::compute_control(const ros::Time&) {

    double virtual_steering_angle;
    if (input_manager_.rt_commands_.lin == 0)
      virtual_steering_angle = 0.;
    else
      virtual_steering_angle = std::atan(input_manager_.rt_commands_.ang/input_manager_.rt_commands_.lin* GEOM_L);
					 
    steering_angle_ = virtual_steering_angle; // mechanics is supposedly doing the trick...
    steering_servo_ = STS_A1 * steering_angle_ + STS_A0;

    //double wheels_rvel = input_manager_.rt_commands_.lin / WHEEL_RADIUS;
    double dv = input_manager_.rt_commands_.ang * GEOM_D / 2;
    double left_wheel_rvel_sp  = ( input_manager_.rt_commands_.lin - dv ) / WHEEL_RADIUS;
    double right_wheel_rvel_sp = ( input_manager_.rt_commands_.lin + dv ) / WHEEL_RADIUS;

    double left_wheel_err = left_wheel_rvel_sp - left_wheel_rvel_;
    double right_wheel_err = right_wheel_rvel_sp - right_wheel_rvel_;
    
    double left_wheel_feedback = rc_march_filter(&left_wheel_pid_, left_wheel_err);
    double right_wheel_feedback = rc_march_filter(&right_wheel_pid_, right_wheel_err);
    
    left_wheel_duty_ =  motor_precommand(left_wheel_rvel_sp) + left_wheel_feedback;
    right_wheel_duty_ = motor_precommand(right_wheel_rvel_sp) + right_wheel_feedback;
  }


  
  PLUGINLIB_EXPORT_CLASS(oscar_controller::OscarAckermannController, controller_interface::ControllerBase);
}
