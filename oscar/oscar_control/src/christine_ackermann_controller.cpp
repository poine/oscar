#include <oscar_control/christine_ackermann_controller.h>

namespace oscar_controller {

  
#define __NAME "christine_ackermann_controller"

  // Geometry
#define WHEEL_BASE    0.27
#define WHEEL_RADIUS  0.041
  //#define WHEEL_SEP     0.27

#define GEOM_L 0.27
#define GEOM_D 0.09

  // Odometry
#define VELOCITY_ROLLING_WINDOW_SIZE 10

 

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  ChristineAckermannController::ChristineAckermannController():
    lvel_sp_(0.)
    ,steering_angle_(0.) {

    ROS_INFO_STREAM_NAMED(__NAME, "in ChristineAckermannController::ChristineAckermannController...");
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  ChristineAckermannController::~ChristineAckermannController() {

  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool ChristineAckermannController::init(hardware_interface::RobotHW* hw,
				      ros::NodeHandle& root_nh,
				      ros::NodeHandle& controller_nh)
  {
    ROS_INFO_STREAM_NAMED(__NAME, "in ChristineAckermannController::init");
    hardware_interface::EffortJointInterface* e1 = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e1->getHandle("left_wheel_joint");
    right_wheel_joint_ = e1->getHandle("right_wheel_joint");
    hardware_interface::PositionJointInterface* e2 = hw->get<hardware_interface::PositionJointInterface>();
    steering_joint_ = e2->getHandle("steering_joint");

    input_manager_.init(hw, controller_nh);
    odometry_.init(WHEEL_BASE, VELOCITY_ROLLING_WINDOW_SIZE);
    odometry_.setWheelRadius(WHEEL_RADIUS);
    //odometry_.setWheelSep(WHEEL_SEP);
    publisher_.init(root_nh, controller_nh);
    //debug_io_publisher_.init(root_nh, controller_nh);
    //raw_odom_publisher_.init(root_nh, controller_nh);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void ChristineAckermannController::starting(const ros::Time& now) {
    ROS_INFO_STREAM_NAMED(__NAME, "in ChristineAckermannController::starting");
    odometry_.starting(now);
    publisher_.starting(now);
  }
  
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void ChristineAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    left_wheel_rvel_ = left_wheel_joint_.getVelocity();
    right_wheel_rvel_ = right_wheel_joint_.getVelocity();
    
    input_manager_.update(now);
    //std::printf("  input manager %f ",input_manager_.rt_commands_.speed);
    
    odometry_.update(left_wheel_rvel_, right_wheel_rvel_, steering_joint_.getPosition(), now);
    
    compute_control(now);

    steering_joint_.setCommand(steering_angle_);
    double mot_rvel = lvel_sp_ / WHEEL_RADIUS;
    left_wheel_joint_.setCommand(mot_rvel);
    right_wheel_joint_.setCommand(mot_rvel);
    
    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
    //double left_wheel_angle = left_wheel_joint_.getPosition();
    //double right_wheel_angle = right_wheel_joint_.getPosition();
  }
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void ChristineAckermannController::stopping(const ros::Time&) {

  }



  void ChristineAckermannController::compute_control(const ros::Time&) {
    if (input_manager_.rt_commands_.mode == 1) { // ackermann
      lvel_sp_  = input_manager_.rt_commands_.speed;
      steering_angle_   = input_manager_.rt_commands_.steering;
    }
    else  {// twist
      lvel_sp_  = input_manager_.rt_commands_.lin;
      if (abs(lvel_sp_) <= 1e-6)
	steering_angle_ = 0;
      else
	steering_angle_ = std::atan(input_manager_.rt_commands_.ang/lvel_sp_* GEOM_L);
    }
  }
  
  PLUGINLIB_EXPORT_CLASS(oscar_controller::ChristineAckermannController, controller_interface::ControllerBase);
}
