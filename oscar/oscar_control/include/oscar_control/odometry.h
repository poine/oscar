/**
 * @author Luca Marchionni
 * @author Bence Magyar
 * @author Enrique Fernández
 * @author Paul Mathieu
 * @author Gérald Lelong
 */

#ifndef OSCAR_CONTROL__ODOMETRY_H_
#define OSCAR_CONTROL__ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>


namespace oscar_controller
{
namespace bacc = boost::accumulators;

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class Odometry
{
public:

    Odometry(size_t velocity_rolling_window_size = 10);

    void init(double wheel_base, size_t velocity_rolling_window_size);
    void starting(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(const double left_wheel_joint_velocity, 
		const double right_wheel_joint_velocity,
		const double steering_joint_position,
		const ros::Time &time);


    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
        return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
        return x_ + wheel_base_ * (1.0 - cos(heading_));
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
        return y_ - wheel_base_ * sin(heading_);
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
        return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
        return angular_;
    }

    void setWheelbase(double wheel_base)
    {
        wheel_base_ = wheel_base;
    }

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
    void setWheelRadius(const double _wr) { wheel_radius_ = _wr;}
    //void setWheelSep(const double _ws) { wheel_sep_ = _ws;}
    

private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

    double wheel_base_;    // dist beween back and front wheels
    //double wheel_sep_;     // dist beween back and front wheels
    double wheel_radius_;  // wheel radius

    /// Previou wheel position/state [rad]:
    //std::map<std::string, double> wheels_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

};
}

#endif /* OSCAR_CONTROL__ODOMETRY_H_ */
