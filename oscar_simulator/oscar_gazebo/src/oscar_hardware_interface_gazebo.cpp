#include <oscar_gazebo/oscar_hardware_interface_gazebo.h>

#define __NAME "oscar_hardware_interface_gazebo"


namespace oscar_hardware_gazebo {

  const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint", "steering_joint", "front_axle_joint"};

  OscarHardwareInterface::OscarHardwareInterface():
    motors_on_(false)
  {
    this->registerInterface(static_cast<OscarHardwareInterface *>(this));
    ROS_INFO_STREAM_NAMED( __NAME, "OscarHardwareInterfaceGazebo::OscarHardwareInterfaceGazebo");
  }

  
  bool OscarHardwareInterface::initSim(
				       const std::string& robot_namespace,
				       ros::NodeHandle model_nh,
				       gazebo::physics::ModelPtr parent_model,
				       const urdf::Model *const urdf_model,
				       std::vector<transmission_interface::TransmissionInfo> transmissions) {
    
    ROS_INFO_STREAM_NAMED( __NAME, "OscarHardwareInterfaceGazebo::initSim");
    model_ = parent_model;     // store parent model pointer
    link_ = model_->GetLink(); // base_link
    for (int i=0; i<NB_JOINTS; i++) {
      gz_joints_[i] = parent_model->GetJoint(joint_name_[i]);
    }
    // register joints
    for (int i=0; i<NB_JOINTS; i++) {
      joint_position_[i] = 0.;
      joint_velocity_[i] = 0.;
      joint_effort_[i] = 0.;
      joint_effort_command_[i] = 0.;
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
	joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
      if (i != 2) 
	ej_interface_.registerHandle(hardware_interface::JointHandle(
				     js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
      else
	pj_interface_.registerHandle(hardware_interface::JointHandle(
				     js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
	
    }
    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&ej_interface_);
    // register IMU
    imu_data_.name = "imu";
    imu_data_.frame_id = "imu_link";
    imu_data_.orientation = imu_orientation_;
    imu_data_.angular_velocity = imu_angular_velocity_;
    imu_data_.linear_acceleration = imu_linear_acceleration_;
    hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&imu_sensor_interface_);

    return true;
  }
  
  void OscarHardwareInterface::readSim(ros::Time time, ros::Duration period) {
    for (int i=0; i<NB_JOINTS; i++) {
      joint_position_[i] = gz_joints_[i]->Position(0);
      joint_velocity_[i] = gz_joints_[i]->GetVelocity(0);
    }
  }

  void OscarHardwareInterface::writeSim(ros::Time time, ros::Duration period) {
    for (int i=0; i<NB_JOINTS-1; i++) {
      gz_joints_[i]->SetForce(0, joint_effort_command_[i]/10.);
    }
    gz_joints_[2]->SetPosition(0, joint_position_command_[2]);
  }

  
  void OscarHardwareInterface::switch_motors_on()  { motors_on_ = true; }
  void OscarHardwareInterface::switch_motors_off() { motors_on_ = false; }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oscar_hardware_gazebo::OscarHardwareInterface, gazebo_ros_control::RobotHWSim)

