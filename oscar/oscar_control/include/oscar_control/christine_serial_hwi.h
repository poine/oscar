#ifndef OSCAR_CONTROL__CHRISTINE_SERIAL_HWI_H
#define OSCAR_CONTROL__CHRISTINE_SERIAL_HWI_H
#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

//#include <async_comm/serial.h>

#define NB_JOINTS 3

#include "oscar_control/christine_remote_bbb_protocol.h"
#include "oscar_control/christine_remote_bbb.h"

class ChristineSerialHWI : public hardware_interface::RobotHW
{
  public:
    ChristineSerialHWI();
    virtual ~ChristineSerialHWI();
    bool start();
    void read(ros::Time now);
    void write();
    bool shutdown();

  private:
    // Joints
    double joint_position_[NB_JOINTS];
    double joint_velocity_[NB_JOINTS];
    double joint_effort_[NB_JOINTS];
    double joint_effort_command_[NB_JOINTS];
    double joint_position_command_[NB_JOINTS];
 
    // IMU
    hardware_interface::ImuSensorHandle::Data imu_data_;
    double imu_orientation_[4];         // quat is in the order of geometry_msg, ie x, y, z, w
    double imu_angular_velocity_[3];
    double imu_linear_acceleration_[3];
    
    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::ImuSensorInterface     imu_sensor_interface_;
    

    BBBLink bbb_link_;
    void serial_msg_cbk();
};

#endif // OSCAR_CONTROL__CHRISTINE_BBB_HWI_H
