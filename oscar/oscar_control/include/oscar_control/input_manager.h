#ifndef OSCAR_CONTROL__INPUT_MANAGER
#define OSCAR_CONTROL__INPUT_MANAGER

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "oscar_control/oscar_hardware_interface.h"

namespace oscar_controller {

  class InputManager {

  public:
    InputManager();
    bool init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh);
    void update(const ros::Time&);
 
    struct Commands
    {
      double lin;
      double ang;
      double steering;
      double speed;
      int mode;
      ros::Time stamp_twist;
      ros::Time stamp_ack;
      
    Commands() : lin(0.0), ang(0.0), steering(0.), speed(0.), mode(0), stamp_twist(0.0), stamp_ack(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands nrt_ros_command_struct_;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_command_ack_;
 
    Commands rt_commands_;
    
  private:
    void cmdVelCallback(const geometry_msgs::Twist& command);
    void cmdAckCallback(const ackermann_msgs::AckermannDriveStamped &command);
  };

}




#endif // OSCAR_CONTROL__INPUT_MANAGER
