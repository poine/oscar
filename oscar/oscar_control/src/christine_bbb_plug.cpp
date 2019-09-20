#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <glib.h>

#include <robotcontrol.h>

#include "roverboard_odrive/roverboard_odrive_can.h"
#include "oscar_control/christine_serial_port.h"
#include "oscar_control/christine_remote_bbb_protocol.h"


//
// This runs on a beaglebone blue. It talks to a master computer through a serial link.
// It drives an odrive for motor and a servo for steering
// It reports odrive motor odometry as well as IMU and radio control
//


#define DSM_CHAN_STEERING 2
#define DSM_CHAN_THROTTLE 3
#define DSM_CHAN_MODE     5

#define DSM_MODE_AUTO    0.9
#define DSM_MODE_MANUAL -0.9


//
// messages profiler
//
struct FreqCnt {
  uint64_t latest_event_date;
  float period;
  float freq;
};

void freq_cnt_init(struct FreqCnt* self) {
  self->latest_event_date = 0;
  self->period = 1.;
  self->freq = 1.;
}

float freq_cnt_sec_since_last_event(struct FreqCnt* self, uint64_t now) {
  return (now-self->latest_event_date)*1e-9;
}

void freq_cnt_record(struct FreqCnt* self, uint64_t now) {
  if (self->latest_event_date != 0) {
    self->period = freq_cnt_sec_since_last_event(self, now);
    self->freq = 1./self->period;
  }
  self->latest_event_date = now;
}


//
// messages link
//
struct ROSLink {
  // serial port
  struct SerialPort* sp; int sp_fd; GIOChannel *channel;
  struct ChristineHWIParser parser;
  int tx_seq;
  
  struct FreqCnt rx_time_stats;
  struct FreqCnt tx_time_stats;
  
};
static int ros_link_init(struct ROSLink* self, const char *serial_device);
static gboolean ros_link_serial_data_received(GIOChannel *source,
					      GIOCondition condition,
					      gpointer data);
static int ros_link_send(struct ROSLink* self, struct ChristineHardwareOutputMsg* hom);

struct Main {
  int quit;
  
  uint32_t periodic_counter;

  struct ROSLink ros_link;
  // received from link (ROS)
  uint8_t link_valid;
  float link_setpoint_steering;  // normalized (-1, 1)
  float link_setpoint_vel;       // cps (enc click per sec)

  // IMU
  rc_mpu_data_t rc_mpu_data;

  // DSM
  uint8_t dsm_valid;
  float dsm_throttle;
  float dsm_steering;
  float dsm_mode;

  // What we will feed to actuators
  float act_setpoint_steering;  // normalized (-1, 1)
  float act_setpoint_vel;       // cps (enc click per sec)
  
  // Odrive
  OdriveCAN* odrv;
  // sent to odrive
  double vsps[2];
  double iffs[2];
  // read from odrive
  double pos[2], vel[2];
  double iqsp[2], iqm[2];
  
  // Servos
  float steering_servo_input;
  //float throttle_servo_input;
  
};

static struct Main _main;

static void msg_cbk(void* data, uint8_t* buf, uint8_t len);

static void parse_data(char *buf, int len);
static void drive_actuators();
static void display();

// IMU
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21
#define IMU_SAMPLE_RATE_HZ 50
//100
#define IMU_DT (1./IMU_SAMPLE_RATE_HZ)

static void __mpu_cbk(void) {
  if (_main.dsm_valid) {
    if (_main.dsm_mode > DSM_MODE_MANUAL) {
      _main.act_setpoint_steering = _main.dsm_steering;
      _main.act_setpoint_vel = _main.dsm_throttle*1000.; // FIXME
    }
    else {
      _main.act_setpoint_steering = _main.link_setpoint_steering;
      _main.act_setpoint_vel = _main.link_setpoint_vel;
    }
  }
  else {
    if (_main.link_valid) {
      _main.act_setpoint_steering =  _main.link_setpoint_steering;
      _main.act_setpoint_vel = _main.link_setpoint_vel;
    }
    else {
      _main.act_setpoint_steering = 0;
      _main.act_setpoint_vel = 0;
    }
  }
  //fprintf(stderr, "mpu\n");
  drive_actuators();
}

// Serial communications 

static int ros_link_init(struct ROSLink* self, const char *serial_device) {
  self->sp = serial_port_new();
  int ret = serial_port_open_raw(self->sp, serial_device,  B115200);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    return -1;
  }
  fprintf(stderr, "Opened serial port %s\n", serial_device);
  self->channel = g_io_channel_unix_new(self->sp->fd);
  g_io_channel_set_encoding(self->channel, NULL, NULL);
  g_io_add_watch(self->channel, G_IO_IN , ros_link_serial_data_received, NULL);

  self->parser.msg_cbk = msg_cbk;
  parser_reset(&self->parser);
  self->tx_seq = 0;
  freq_cnt_init(&self->rx_time_stats);
  freq_cnt_init(&self->tx_time_stats);
}


static gboolean ros_link_serial_data_received(GIOChannel *source,
					      GIOCondition condition __attribute__((unused)),
					      gpointer data __attribute__((unused)))
{
  char buf[255];
  gsize bytes_read;
  GError *_err = NULL;
  GIOStatus st = g_io_channel_read_chars(source, buf, 255, &bytes_read, &_err);
  if (!_err) {
    if (st == G_IO_STATUS_NORMAL) {
#if 0
      printf("read %u\n", bytes_read);
      for (int i=0; i<bytes_read; i++)
	fprintf(stderr, "  %x", buf[i]);
      fprintf(stderr, "\n");
#endif
      for (int i=0; i<bytes_read; i++)
	parser_parse(&_main.ros_link.parser, buf[i]);
    }
  } else {
    printf("error reading serial: %s\n", _err->message);
    g_error_free(_err);
  }
  return TRUE;
}

static int ros_link_send(struct ROSLink* self, struct ChristineHardwareOutputMsg* hom) {
  freq_cnt_record(&self->tx_time_stats, rc_nanos_since_boot());
  hom->stx = CHRISTINE_HWI_MSG_STX;
  hom->len = sizeof(hom->data);
  hom->seq = self->tx_seq;
  uint8_t* buf = (uint8_t*)(hom);
  compute_checksum(buf+4, sizeof(hom->data), &hom->ck1, &hom->ck2);
  self->tx_seq += 1;
  gsize bytes_written; GError *_err = NULL;
  g_io_channel_write_chars(self->channel, (gchar*)(hom), sizeof(*hom), &bytes_written, &_err);
  if (_err) {
    printf("error writing serial: %s\n", _err->message);
    g_error_free(_err);
    return -1;
  }
  g_io_channel_flush(self->channel, NULL);
  return 0;
}

static void msg_cbk(void* data, uint8_t* buf, uint8_t len) {
#if 0
  fprintf(stderr, "Got msg (%u)\n", len);
  for (auto i=0; i<len; i++)
    fprintf(stderr, " %02x", buf[i]);
  fprintf(stderr, "\n");
#endif
  freq_cnt_record(&_main.ros_link.rx_time_stats, rc_nanos_since_boot());
  memcpy(&_main.link_setpoint_steering, buf, sizeof(float));
  memcpy(&_main.link_setpoint_vel, buf+sizeof(float), sizeof(float));
  
}

static int bp_init(const char *serial_device) {
  _main.periodic_counter = 0;

  // initialize serial port
  ros_link_init(&_main.ros_link, serial_device);
  _main.link_valid = false;

  // initialize ADCS (battery monitor)
  if(rc_adc_init()){
    fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
    return -2;
  }

  // initialize DSM (radio control)
  if(rc_dsm_init()==-1) {
    fprintf(stderr, "Error initializing DSM\n");
    return -4;
  }
  _main.dsm_valid = 0;

  // initialize PRU (servos)
  if(rc_servo_init()) {
    fprintf(stderr, "Error initializing PRU servos\n");
    return -3;
  }
  // TODO/WARNING: not turned off
  rc_servo_power_rail_en(1);


  // initialize MPU
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  conf.dmp_sample_rate = IMU_SAMPLE_RATE_HZ;
  conf.dmp_fetch_accel_gyro = true;
  conf.orient = ORIENTATION_Z_UP;
  if(rc_mpu_initialize_dmp(&_main.rc_mpu_data, conf)){
    fprintf(stderr, "can't talk to IMU, all hope is lost\n");
    return -4;
  }
  rc_mpu_set_dmp_callback(&__mpu_cbk);

  _main.odrv = new OdriveCAN();
  _main.vsps[0] = 0.;
  _main.iffs[0] = 0.;
  _main.odrv->init();
  return 0;
}

static void bp_deinit() {
  // TODO serial port
  rc_adc_cleanup();
  rc_servo_power_rail_en(0);
  rc_servo_cleanup();
}


#define _DEG2RAD(_D) _D/180.*M_PI
static void send() {
  struct ChristineHardwareOutputMsg hom;
  hom.data.bat_voltage = rc_adc_batt();
  hom.data.mot_pos = _main.pos[0];
  hom.data.mot_vel = _main.vel[0];
  hom.data.dsm_steering = _main.dsm_steering;
  hom.data.dsm_throttle = _main.dsm_throttle;
  //hom.data.dsm_mode = _main.dsm_mode;
  //hom.data.dsm_valid = _main.dsm_valid;
  
  hom.data.ax = _main.rc_mpu_data.accel[0];
  hom.data.ay = _main.rc_mpu_data.accel[1];
  hom.data.az = _main.rc_mpu_data.accel[2];
  hom.data.gx = _DEG2RAD(_main.rc_mpu_data.gyro[0]);
  hom.data.gy = _DEG2RAD(_main.rc_mpu_data.gyro[1]);
  hom.data.gz = _DEG2RAD(_main.rc_mpu_data.gyro[2]);
  hom.data.qw = _main.rc_mpu_data.dmp_quat[0];
  hom.data.qx = _main.rc_mpu_data.dmp_quat[1];
  hom.data.qy = _main.rc_mpu_data.dmp_quat[2];
  hom.data.qz = _main.rc_mpu_data.dmp_quat[3];
  ros_link_send(&_main.ros_link, &hom);
}

static void drive_actuators() {
  _main.steering_servo_input = _main.act_setpoint_steering + 0.135; //0.13; //0.175
  rc_servo_send_pulse_normalized(1, _main.steering_servo_input);
  _main.vsps[0] = abs(_main.act_setpoint_vel) > 20 ? _main.act_setpoint_vel : 0;
  _main.odrv->sendVelSetpoints(_main.vsps, _main.iffs);
}

static void read_dsm() {
  _main.dsm_valid    = rc_dsm_is_connection_active();
  _main.dsm_steering = rc_dsm_ch_normalized(DSM_CHAN_STEERING);
  _main.dsm_throttle = rc_dsm_ch_normalized(DSM_CHAN_THROTTLE);
  _main.dsm_mode     = rc_dsm_ch_normalized(DSM_CHAN_MODE);
}

static void display() {
  float time_since_last_msg = freq_cnt_sec_since_last_event(&_main.ros_link.rx_time_stats, rc_nanos_since_boot());
  fprintf(stderr, "\rLINK: last %.2fs freq %.1fhz err %d | ", time_since_last_msg, _main.ros_link.rx_time_stats.freq, _main.ros_link.parser.err_cnt);
  if (_main.dsm_valid)
    fprintf(stderr, "DSM: %.01f %.01f %.01f | ", _main.dsm_steering, _main.dsm_throttle, _main.dsm_mode);
  else
    fprintf(stderr, "DSM: N/A               | ");
  fprintf(stderr, "SERVOS: %.01f %% | ", _main.steering_servo_input*100);
  fprintf(stderr, "ODRIVE: sp: % 5.01f cps cur %.01f A%%", _main.vsps[0],  _main.iqsp[0]);
}

gboolean periodic_callback(gpointer data)
{
  _main.periodic_counter += 1;
  uint64_t link_delay = rc_nanos_since_boot()-_main.ros_link.rx_time_stats.latest_event_date;
  _main.link_valid = link_delay > uint64_t(0.2*1e9)?false:true;

  _main.odrv->readFeedback(_main.pos, _main.vel, _main.iqsp, _main.iqm);
  read_dsm();
  send();
  if (_main.periodic_counter%10 == 0)
    display();
}


int main(int argc, char *argv[]) {
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  const char* serial_device = "/dev/ttyS5";
  if (bp_init(serial_device)) {
    return 1;
  }
  _main.quit = 0;
  float freq_transmit = 50.;
  g_timeout_add(1000 / freq_transmit, periodic_callback, NULL);
  g_main_loop_run(ml);
  return 0;
}
