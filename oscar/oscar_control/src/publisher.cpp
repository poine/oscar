#include "oscar_control/publisher.h"

#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>

namespace oscar_controller {

  Publisher::Publisher():
      odom_frame_id_("odom")
    , base_frame_id_("base_link")
    , base_link_("base_link")
    , enable_odom_tf_(true)
    , publish_period_(1.0 / 50.)
  {}
    
  void Publisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
// Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
                                      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
                                      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
                                      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
                                      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
                                      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
                                       (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                       (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
                                       (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
                                       (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
                                       (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
                                       (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;

  }

  void Publisher::starting(const ros::Time& now) {
    last_state_publish_time_ = now;
   }
  
  void Publisher::publish(const double heading, const double x, const double y, const double linear, const double angular, const ros::Time& now) {
    if (last_state_publish_time_ + publish_period_ < now) {
      last_state_publish_time_ += publish_period_;

      const geometry_msgs::Quaternion orientation(
						  tf::createQuaternionMsgFromYaw(heading));

        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = now;
            odom_pub_->msg_.pose.pose.position.x = x;
            odom_pub_->msg_.pose.pose.position.y = y;
            odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x  = linear;
            odom_pub_->msg_.twist.twist.angular.z = angular;
            odom_pub_->unlockAndPublish();
        }

        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = now;
            odom_frame.transform.translation.x = x;
            odom_frame.transform.translation.y = y;
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

  }
  
  
}
