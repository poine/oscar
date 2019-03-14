#include <oscar_control/oscar_hardware_interface.h>
#include <controller_manager/controller_manager.h>

#include <thread>

#define __NAME "oscar_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint", "steering_joint"};

// Mechanics
#define GEARBOX                       100.37
#define ENCODER_RES                     12

// Electrical hookups
#define STEERING_SERVO_CH               1
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

static OscarHardwareInterface* _foo_hw_interf = NULL;

void _imu_callback(void* data) { reinterpret_cast<OscarHardwareInterface*>(data)->IMUCallback(); }

/*******************************************************************************
 *
 *
 *******************************************************************************/
OscarHardwareInterface::OscarHardwareInterface():
  gear_enc_res_(GEARBOX*ENCODER_RES)
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
    if (i<2) {     // motors: effort_command
      ej_interface_.registerHandle(hardware_interface::JointHandle(
	js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
    }
    else {         // servo: position command
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


#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static void __mpu_cbk(void) {  _foo_hw_interf->IMUCallback(); }


/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::start() {

  // encoders
  if(rc_encoder_eqep_init()){
    ROS_ERROR("in OscarHardwareInterface::start: failed to initialize eqep");
    return -1;
  }
  // motors
  if (rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ)) {
    ROS_ERROR("in OscarHardwareInterface::start: failed to initialize motors");
    return -1;
  }
  // IMU
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  conf.dmp_sample_rate = IMU_SAMPLE_RATE_HZ;
  conf.dmp_fetch_accel_gyro = true;
  conf.orient = ORIENTATION_Z_UP;
  if(rc_mpu_initialize_dmp(&rc_mpu_data_, conf)){
    ROS_ERROR("in OscarHardwareInterface::start: can't talk to IMU, all hope is lost\n");
    return false;
  }
  _foo_hw_interf = this;
  rc_mpu_set_dmp_callback(&__mpu_cbk);
  // Servos
  rc_servo_init();
  rc_servo_power_rail_en(1);
  rc_servo_send_pulse_normalized(STEERING_SERVO_CH, 0.);
  
  rc_set_state(RUNNING);
  last_read_stamp_ = ros::Time::now();
  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::read(ros::Time now) {
  double left_wheel_angle = rc_encoder_read(ENCODER_CHANNEL_L) * 2 * M_PI / (ENCODER_POLARITY_L * gear_enc_res_);
  double right_wheel_angle = rc_encoder_read(ENCODER_CHANNEL_R) * 2 * M_PI / (ENCODER_POLARITY_R * gear_enc_res_);
  last_period_ = now - last_read_stamp_;
  last_read_stamp_ = now;
  const double dt = last_period_.toSec();
  joint_velocity_[0] = (left_wheel_angle  - joint_position_[0]) / dt;
  joint_velocity_[1] = (right_wheel_angle - joint_position_[1]) / dt;
  joint_position_[0] = left_wheel_angle;
  joint_position_[1] = right_wheel_angle;
  
  joint_position_[2] = joint_position_command_[2];
}
/*******************************************************************************
 *
 *
 *******************************************************************************/
#define STS_A1  -0.938104
#define STS_A0 -0.05
void OscarHardwareInterface::write() {

  float dutyL =  joint_effort_command_[0];
  float dutyR =  joint_effort_command_[1];
  float steering = STS_A1 * joint_position_command_[2] + STS_A0;
  //ROS_INFO(" write HW %f %f %f", dutyL, dutyR, steering);
  
  rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
  rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);
  rc_servo_send_pulse_normalized(STEERING_SERVO_CH, steering);
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::shutdown() {
  ROS_INFO("in OscarHardwareInterface::shutdown");
  rc_encoder_eqep_cleanup();
  rc_mpu_power_off();
  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define _DEG2RAD(_D) _D/180.*M_PI
void OscarHardwareInterface::IMUCallback(void) {

  // Called by rc IMU thread
  // imu_orientation is in the order of geometry_msg, ie x, y, z, w
  // wheras dmp_quat is w, x, y, z
  imu_orientation_[0] = rc_mpu_data_.dmp_quat[1];
  imu_orientation_[1] = rc_mpu_data_.dmp_quat[2];
  imu_orientation_[2] = rc_mpu_data_.dmp_quat[3];
  imu_orientation_[3] = rc_mpu_data_.dmp_quat[0];

  imu_angular_velocity_[0] = _DEG2RAD(rc_mpu_data_.gyro[0]); // WTF are those units !!!
  imu_angular_velocity_[1] = _DEG2RAD(rc_mpu_data_.gyro[1]);
  imu_angular_velocity_[2] = _DEG2RAD(rc_mpu_data_.gyro[2]); 

  imu_linear_acceleration_[0] = rc_mpu_data_.accel[0];
  imu_linear_acceleration_[1] = rc_mpu_data_.accel[1];
  imu_linear_acceleration_[2] = rc_mpu_data_.accel[2];
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
      rc_mpu_block_until_dmp_data();
      ros::Time now = ros::Time::now();
      hw.read(now);
      cm.update(now, period);
      hw.write();
    }

  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "Oscar hardware node exiting...");
  return 0;
}

