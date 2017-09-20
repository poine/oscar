#ifndef OSCAR_CONTROL__PUBLISHER_H_
#define OSCAR_CONTROL__PUBLISHER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace oscar_controller {

  class Publisher {

  public:
    Publisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    void publish(const double heading, const double x, const double y, const double linear, const double angular, const ros::Time& now);

  private:
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string base_link_;
    bool enable_odom_tf_;
 
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

  };

  
}

#endif // OSCAR_CONTROL__PUBLISHER_H_
