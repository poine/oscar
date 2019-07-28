#ifndef OSCAR_CONTROL__CHRISTINE_SERIAL_HWI_H
#define OSCAR_CONTROL__CHRISTINE_SERIAL_HWI_H
#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "oscar_control/christine_hwi_msg.h"
#include <async_comm/serial.h>

#define NB_JOINTS 3


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
    async_comm::Serial serial_;
    void serial_callback(const uint8_t* buf, size_t len);
//    void reset_parser();
//    void parse(uint8_t b);
    void serial_msg_cbk();
//    uint8_t parser_status_;
//    uint8_t parser_buf_[255];
//    uint8_t parser_buf_idx_;
//    uint8_t parser_len_;
    struct ChristineHWIParser parser_;
};

#endif // OSCAR_CONTROL__CHRISTINE_BBB_HWI_H
