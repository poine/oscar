#include <oscar_control/oscar_hardware_interface.h>
#include <controller_manager/controller_manager.h>

#include <thread>

#define __NAME "oscar_hardware_interface"

// IMU
#define IMU_SAMPLE_RATE_HZ 100
#define IMU_DT (1./IMU_SAMPLE_RATE_HZ)


/*******************************************************************************
 *
 *
 *******************************************************************************/
OscarHardwareInterface::OscarHardwareInterface()
{
  ROS_INFO_STREAM_NAMED(__NAME, "in OscarHardwareInterface::OscarHardwareInterface...");

}


/*******************************************************************************
 *
 *
 *******************************************************************************/
OscarHardwareInterface::~OscarHardwareInterface() {
  ROS_INFO(" ~OscarHardwareInterface");
  // TODO make sure this is called
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::start() {

  if(rc_initialize()){
    ROS_ERROR("in OscarHardwareInterface::start: failed to initialize robotics cape");
    return false;
  }

  rc_set_state(RUNNING);
  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::read() {
}
/*******************************************************************************
 *
 *
 *******************************************************************************/
void OscarHardwareInterface::write() {
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool OscarHardwareInterface::shutdown() {
  ROS_INFO("in OscarHardwareInterface::shutdown");
  rc_power_off_imu();
  rc_cleanup();

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "Oscar hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  OscarHardwareInterface hw;
  if (!hw.start()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(IMU_DT);
  while (ros::ok() and rc_get_state()!=EXITING)
    {
      pthread_mutex_lock( &rc_imu_read_mutex );
      pthread_cond_wait( &rc_imu_read_condition, &rc_imu_read_mutex );
      pthread_mutex_unlock( &rc_imu_read_mutex );
      
      hw.read();
      cm.update(ros::Time::now(), period);
      hw.write();
    }

  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "Oscar hardware node exiting...");
  return 0;
}

