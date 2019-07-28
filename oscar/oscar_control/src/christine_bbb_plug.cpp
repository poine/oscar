#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <glib.h>

#include <robotcontrol.h>

#include "oscar_control/christine_serial_port.h"
#include "oscar_control/christine_hwi_msg.h"

struct State {
  uint32_t periodic_counter;
  struct SerialPort* sp;
  int sp_fd;
  GIOChannel *channel;
  int quit;
  int tx_seq;
  struct ChristineHWIParser parser;
  float steering_servo_input;
  float throttle_servo_input;
  uint64_t last_rx_msg_time;

  float   motor_vel;
  int32_t motor_pos;
  
};

static struct State state;

static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition,
                                        gpointer data);
static void parse_data(char *buf, int len);
static void drive_servos();
static void display();

static void msg_cbk(uint8_t* buf, uint8_t len) {
#if 0
  fprintf(stderr, "Got msg (%u)\n", len);
  for (auto i=0; i<len; i++)
    fprintf(stderr, " %02x", buf[i]);
  fprintf(stderr, "\n");
#endif
  memcpy(&state.steering_servo_input, buf, sizeof(float));
  memcpy(&state.throttle_servo_input, buf+sizeof(float), sizeof(float));
  state.last_rx_msg_time = rc_nanos_since_boot();
  drive_servos();
  
  //struct ChristineHardwareInput* hi = reinterpret_cast<struct ChristineHardwareInput*>(buf);
  //struct ChristineHardwareInput* hi = (struct ChristineHardwareInput*)buf;
  //fprintf(stderr, "  steering: %f\n", hi->steering_srv);
  //std::printf("  throttle: %f\n", hi->throttle_servo);
}

static int bp_init(const char *serial_device) {
  // initialize serial port
  state.periodic_counter = 0;
  state.sp = serial_port_new();
  int ret = serial_port_open_raw(state.sp, serial_device,  B115200);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    return 1;
  }
  fprintf(stderr, "Opened serial port %s\n", serial_device);
  state.channel = g_io_channel_unix_new(state.sp->fd);
  g_io_channel_set_encoding(state.channel, NULL, NULL);
  g_io_add_watch(state.channel, G_IO_IN , on_serial_data_received, NULL);

  state.tx_seq = 0;
  state.parser.msg_cbk = msg_cbk;
  parser_reset(&state.parser);
  state.last_rx_msg_time = 0;
  
  // initialize ADCS
  if(rc_adc_init()){
    fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
      return -1;
  }
  // initialize PRU
  if(rc_servo_init()) {
    fprintf(stderr, "Error initializing PRU servos\n");
    return -1;
  }
  // TODO/WARNING: not turned off
  rc_servo_power_rail_en(1);
  // initialize encoder
  rc_encoder_eqep_init();
  state.motor_pos = rc_encoder_eqep_read(1); 
  return 0;
}

static void bp_deinit() {
  // TODO serial port
  rc_adc_cleanup();
  rc_servo_power_rail_en(0);
  rc_servo_cleanup();
}


static gboolean on_serial_data_received(GIOChannel *source,
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
	parser_parse(&state.parser, buf[i]);
    }
  } else {
    printf("error reading serial: %s\n", _err->message);
    g_error_free(_err);
  }
  return TRUE;
}





static void send() {
  state.tx_seq += 1;
  struct ChristineHardwareOutputMsg hom;
  hom.stx = CHRISTINE_HWI_MSG_STX;
  hom.len = sizeof(hom.data);
  hom.seq = state.tx_seq;
  hom.data.bat_voltage = rc_adc_batt();
  hom.data.mot_vel = state.motor_vel;

  uint8_t* buf = (uint8_t*)(&hom);
  compute_checksum(buf+4, sizeof(hom.data), &hom.ck1, &hom.ck2);
  gsize bytes_written; GError *_err = NULL;
  g_io_channel_write_chars(state.channel, (gchar*)(&hom), sizeof(hom), &bytes_written, &_err);
  if (!_err) {
    //printf("\rwrote %u\n", bytes_written);
  } else {
    printf("error writing serial: %s\n", _err->message);
    g_error_free(_err);
  }
  g_io_channel_flush(state.channel, NULL);
}

static void drive_servos() {
  rc_servo_send_pulse_normalized(1, state.steering_servo_input);
  rc_servo_send_pulse_normalized(2, state.throttle_servo_input);
}


static void display() {
  fprintf(stderr, "last_rx_time %Ld  ", state.last_rx_msg_time);
  fprintf(stderr, "servos: %.03f %.03f\r\n", state.steering_servo_input, state.throttle_servo_input);

}

gboolean periodic_callback(gpointer data)
{
  state.periodic_counter += 1;
  uint64_t delay = rc_nanos_since_boot()-state.last_rx_msg_time;
  if (delay > 200000000) {
    state.steering_servo_input = 0;
    state.throttle_servo_input = 0;
    drive_servos();
  }
  int tmp = rc_encoder_eqep_read(1);
  float klp = 0.9;
  state.motor_vel = klp*state.motor_vel + (1-klp)*float(tmp-state.motor_pos);
  state.motor_pos = tmp;
  send();
  if (state.periodic_counter%10 == 0)
    display();
}


int main(int argc, char *argv[]) {
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  const char* serial_device = "/dev/ttyS5";
  if (bp_init(serial_device)) {
    return 1;
  }
  state.quit = 0;
  float freq_transmit = 50.;
  g_timeout_add(1000 / freq_transmit, periodic_callback, NULL);
  g_main_loop_run(ml);
  return 0;
}
