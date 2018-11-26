#include <oscar_control/oscar_hardware_interface.h>
#include <controller_manager/controller_manager.h>

#include <thread>

#define __NAME "oscar_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint", "steering_joint"};

// Mechanics
#define GEARBOX                       100.37
#define ENCODER_RES                     12
//#define WHEEL_RADIUS_M                  0.03
// Electrical hookups
#define STEERING_SERVO_CH               1
#define STEERING_SERVO_NEUTRAL          0.
//-0.11
#define STEERING_SERVO_POLARITY        -1
#define MOTOR_CHANNEL_L                 1
#define MOTOR_CHANNEL_R                 2
#define MOTOR_POLARITY_L               -1
#define MOTOR_POLARITY_R               -1
#define ENCODER_CHANNEL_L               1
#define ENCODER_CHANNEL_R               2
#define ENCODER_POLARITY_L             -1
#define ENCODER_POLARITY_R              1
// IMU
#define IMU_SAMPLE_RATE_HZ 100
#define IMU_DT (1./IMU_SAMPLE_RATE_HZ)

void _imu_callback(void* data) { reinterpret_cast<OscarHardwareInterface*>(data)->IMUCallback(); }

/*******************************************************************************
 *
 *
 *******************************************************************************/
OscarHardwareInterface::OscarHardwareInterface()
{
  ROS_INFO_STREAM_NAMED(__NAME, "in OscarHardwareInterface::OscarHardwareInterface...");

  ROS_INFO_STREAM_NAMED(__NAME, "Registering interfaces");
  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = 0.;
    joint_velocity_[i] = 0.;
    joint_effort_[i] = 0.;
    joint_effort_command_[i] = 0.;
    joint_position_command_[i] = 0.;
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    if (i<2) {
      ej_interface_.registerHandle(hardware_interface::JointHandle(
	js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
    }
    else {
      pj_interface_.registerHandle(hardware_interface::JointHandle(
        js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
    }
  }
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  
  // register IMU
  imu_data_.name = "imu";
  imu_data_.frame_id = "imu_link";
  imu_data_.orientation = imu_orientation_;
  imu_data_.angular_velocity = imu_angular_velocity_;
  imu_data_.linear_acceleration = imu_linear_acceleration_;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
  imu_sensor_interface_.registerHandle(imu_sensor_handle);
  registerInterface(&imu_sensor_interface_);
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
OscarHardwareInterface::~OscarHardwareInterface() {
  ROS_INFO(" ~OscarHardwareInterface");
  // TODO make sure this is called
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::start() {

  if(rc_initialize()){
    ROS_ERROR("in OscarHardwareInterface::start: failed to initialize robotics cape");
    return false;
  }

  // IMU
  rc_imu_config_t imu_config = rc_default_imu_config();
  imu_config.dmp_sample_rate = IMU_SAMPLE_RATE_HZ;
  imu_config.orientation = ORIENTATION_Z_UP;
  if(rc_initialize_imu_dmp(&rc_imu_data_, imu_config)){
    ROS_ERROR("in OscarHardwareInterface::start: can't talk to IMU, all hope is lost\n");
    //rc_blink_led(RED, 5, 5);
    return false;
  }
  rc_set_imu_interrupt_func(&_imu_callback, reinterpret_cast<void*>(this));
  // Servos
  rc_enable_servo_power_rail();
  rc_send_servo_pulse_normalized( STEERING_SERVO_CH, STEERING_SERVO_NEUTRAL );
  
  rc_set_state(RUNNING);
  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::read() {

  double left_wheel_angle = rc_get_encoder_pos(ENCODER_CHANNEL_L) * 2 * M_PI / (ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
  double right_wheel_angle = rc_get_encoder_pos(ENCODER_CHANNEL_R) * 2 * M_PI / (ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
  joint_velocity_[0] = (left_wheel_angle - joint_position_[0]) / IMU_DT;
  joint_velocity_[1] = (right_wheel_angle - joint_position_[1]) / IMU_DT;
  joint_position_[0] = left_wheel_angle;
  joint_position_[1] = right_wheel_angle;
  
  joint_position_[2] = joint_position_command_[2];
}
/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::write() {

  float dutyL =  joint_effort_command_[0];
  float dutyR =  joint_effort_command_[1];
  float steering = joint_position_command_[2];
  //ROS_INFO(" write HW %f %f %f", dutyL, dutyR, steering);
  
  rc_set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
  rc_set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);
  rc_send_servo_pulse_normalized( STEERING_SERVO_CH, STEERING_SERVO_NEUTRAL+steering*STEERING_SERVO_POLARITY );
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::shutdown() {
  ROS_INFO("in OscarHardwareInterface::shutdown");
  rc_power_off_imu();
  rc_cleanup();

  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::IMUCallback(void) {

  // Called by rc IMU thread
  // imu_orientation is in the order of geometry_msg, ie x, y, z, w
  // wheras dmp_quat is w, x, y, z
  imu_orientation_[0] = rc_imu_data_.dmp_quat[1];
  imu_orientation_[1] = rc_imu_data_.dmp_quat[2];
  imu_orientation_[2] = rc_imu_data_.dmp_quat[3];
  imu_orientation_[3] = rc_imu_data_.dmp_quat[0];

  imu_angular_velocity_[0] = rc_imu_data_.gyro[0]/180.*M_PI; // WTF are those units !!!
  imu_angular_velocity_[1] = rc_imu_data_.gyro[1]/180.*M_PI;
  imu_angular_velocity_[2] = rc_imu_data_.gyro[2]/180.*M_PI; 

  imu_linear_acceleration_[0] = rc_imu_data_.accel[0];
  imu_linear_acceleration_[1] = rc_imu_data_.accel[1];
  imu_linear_acceleration_[2] = rc_imu_data_.accel[2];

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "Oscar hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  OscarHardwareInterface hw;
  if (!hw.start()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(IMU_DT);
  while (ros::ok() and rc_get_state()!=EXITING)
    {
      pthread_mutex_lock( &rc_imu_read_mutex );
      pthread_cond_wait( &rc_imu_read_condition, &rc_imu_read_mutex );
      pthread_mutex_unlock( &rc_imu_read_mutex );
      
      hw.read();
      cm.update(ros::Time::now(), period);
      hw.write();
    }

  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "Oscar hardware node exiting...");
  return 0;
}

