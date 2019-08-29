
#include <oscar_control/odometry.h>

namespace oscar_controller
{

namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp_(0.0)
    , x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheel_base_(1.0)
    , wheel_radius_(0.03)
    , velocity_rolling_window_size_(velocity_rolling_window_size)
    , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
{
}

  
void Odometry:: init(double wheel_base, size_t velocity_rolling_window_size) {
  setWheelbase(wheel_base);
  setVelocityRollingWindowSize(velocity_rolling_window_size);
}
  
void Odometry::starting(const ros::Time& time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update( const double left_wheel_joint_velocity, 
		       const double right_wheel_joint_velocity,
		       const double steering_angle,
		       const ros::Time &now)
{
    double linear_sum = 0.0;
    double angular_sum = 0.0;
    double steering_angle_sum = 0.0;

    const double dt = (now - timestamp_).toSec();
    timestamp_ = now;

    const double linear = left_wheel_joint_velocity*wheel_radius_*dt;
    const double angular = linear * tan(steering_angle) / wheel_base_;
    
    /// Integrate odometry:
    const double curvature_radius = wheel_base_ / cos(M_PI/2.0 - steering_angle);

    if (fabs(curvature_radius) > 0.0001)
    {
        const double elapsed_distance = linear;
        const double elapsed_angle = elapsed_distance / curvature_radius;
        const double x_curvature = curvature_radius * sin(elapsed_angle);
        const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
        const double wheel_heading = heading_ + steering_angle;
        y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
        x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
        heading_ += elapsed_angle;
    }

    if (dt < 0.0001)
        return false; // Interval too small to integrate with

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
}


void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
}


void Odometry::resetAccumulators() {
  linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}
  
}
