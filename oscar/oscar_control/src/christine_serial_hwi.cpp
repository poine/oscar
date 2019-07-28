#include <oscar_control/christine_serial_hwi.h>
#include <controller_manager/controller_manager.h>

#include "oscar_control/christine_hwi_msg.h"

#define __NAME "christine_serial_hardware_interface"

#define LOOP_HZ 50
#define LOOP_DT (1./LOOP_HZ)
ChristineSerialHWI::ChristineSerialHWI():
  serial_("/dev/ttyTHS1", 115200)
{
  //sp_ = serial_port_new();
  serial_.register_receive_callback(std::bind(&ChristineSerialHWI::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
}

ChristineSerialHWI::~ChristineSerialHWI() {
}

void ChristineSerialHWI::serial_callback(const uint8_t* buf, size_t len) {
  fprintf(stderr, "read %ld\n", len);
}


bool ChristineSerialHWI::start() {
  //const char *serial_device = "/dev/ttyTHS1";
  //int ret = serial_port_open_raw(sp_, serial_device,  B115200);
  //if (ret != 0) {
  //  fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
  //  return false;
  //}
  if (!serial_.init()) {
    std::printf("Failed to initialize serial port\n");
    return false;
  }
  std::printf("initialized serial port\n");
  
  return true;
}

void ChristineSerialHWI::read(ros::Time now) {

}



void ChristineSerialHWI::write() {
  struct ChristineHardwareInputMsg him;
  him.h1 = CHRISTINE_HWI_MSG_HDR;
  him.len = sizeof(him);
  him.data.steering_srv = 0.;
  him.data.throttle_servo = 0.;
  // TODO: compute checksum
  const uint8_t* buf = reinterpret_cast<const uint8_t*>(&him);
  serial_.send_bytes(buf, sizeof(him));
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
  ros::Duration period(LOOP_DT);
  ros::Rate rate(LOOP_HZ);
  while (ros::ok())
    {
      ros::Time now = ros::Time::now();
      hw.read(now);
      cm.update(now, period);
      hw.write();
      rate.sleep();
    }

  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "Christine serial (remote BBB) node exiting...");
  return 0;
}
