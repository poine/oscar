#include "oscar_control/oac_raw_odom_publisher.h"

#include <boost/assign.hpp>

namespace oscar_controller {
  
  OACRawOdomPublisher::OACRawOdomPublisher() {}

  void OACRawOdomPublisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    _pub_.reset(new realtime_tools::RealtimePublisher<oscar_control::oac_RawOdom>(controller_nh, "raw_odom", 100));
    nb_data_ = 0;
  }

  void OACRawOdomPublisher::publish(const double left_wheel_angle, const double right_wheel_angle, const double steering_angle, const ros::Time& now) {
    // FIXME: what happens if trylock fails... i'll just write over the boudaries of my array...

    left_wheel_angle_[nb_data_] = left_wheel_angle;
    right_wheel_angle_[nb_data_] = right_wheel_angle;
    steering_angle_[nb_data_] = steering_angle;
    stamp_[nb_data_] = now;
    nb_data_ += 1;
    if (nb_data_ >= MIN_SENSOR_FOR_PUBLISH) {
      if (_pub_->trylock()) {
	memcpy(_pub_->msg_.stamp.elems, stamp_, nb_data_*sizeof(ros::Time));
	memcpy(_pub_->msg_.left_wheel_angle.elems, left_wheel_angle_, nb_data_*sizeof(double));
	memcpy(_pub_->msg_.right_wheel_angle.elems, right_wheel_angle_, nb_data_*sizeof(double));
	memcpy(_pub_->msg_.steering_angle.elems, steering_angle_, nb_data_*sizeof(double));
	_pub_->msg_.nb_data = nb_data_;
	_pub_->unlockAndPublish();
	nb_data_ = 0;
      }
      else {
	if (nb_data_ < MAX_SENSOR_LEN) {
	  ROS_INFO("#### OACRawOdomPublisher::publish() could no publish... will retry");
	}
	else {
	  ROS_INFO("#### OACRawOdomPublisher::publish() could no publish... discarding one");
	  nb_data_ -= 1;
	}
      }
    }
  }

}
