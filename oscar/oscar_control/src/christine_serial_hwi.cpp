#include <oscar_control/christine_serial_hwi.h>
#include <controller_manager/controller_manager.h>


#define __NAME "christine_serial_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint", "steering_joint"};



#define LOOP_HZ 50
#define LOOP_DT (1./LOOP_HZ)

ChristineSerialHWI::ChristineSerialHWI()
{
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

ChristineSerialHWI::~ChristineSerialHWI() {
}



void ChristineSerialHWI::serial_msg_cbk() {
  std::printf("Got msg\n");
  //struct ChristineHardwareOutput* hi = reinterpret_cast<struct ChristineHardwareOutput*>(parser_buf_);
  //std::printf("  adc: %f\n", hi->bat_voltage);
  //std::printf("  mot_enc: %f\n", hi->mot_enc);
}

bool ChristineSerialHWI::start() {
  return bbb_link_.init();
}

void ChristineSerialHWI::read(ros::Time now) {
  float motor_pos, motor_vel;
  bbb_link_.get_motor(&motor_pos, &motor_vel);
  float bat;
  bbb_link_.get_bat(&bat);
  float a[3]; bbb_link_.get_accel(a);
  std::printf("\r  bat %.2f pos: %.2f vel %.2f accel %.1f %.1f %.1f", bat, motor_pos, motor_vel, a[0], a[1], a[2]);
  joint_position_[2] = joint_position_command_[2];
  
}



void ChristineSerialHWI::write() {
  struct ChristineHardwareInputMsg him;

  //double now = ros::Time::now().toSec();
  float _steering = joint_position_command_[2];//0.5*sin(now);
  float _lvel = joint_effort_command_[0];//0.07*(sin(now)+1); // FIXME
  him.data.steering_srv = _steering;
  him.data.throttle_servo = _lvel;

  bbb_link_.send(&him);

  
}

bool ChristineSerialHWI::shutdown() {
  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "Christine serial (remote BBB) hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ChristineSerialHWI hw;
  if (!hw.start()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(LOOP_DT); ros::Rate rate(LOOP_HZ);
  while (ros::ok())
    {
      ros::Time now = ros::Time::now();
      hw.read(now);
      cm.update(now, period);
      hw.write();
      rate.sleep();
    }

  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "Christine serial (remote BBB) hardware node exiting...");
  return 0;
}
