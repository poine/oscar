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
  struct SerialPort* sp;
  int sp_fd;
  GIOChannel *channel;
  int quit;
  int seq;
  struct ChristineHWIParser parser;
};

static struct State state;

static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition,
                                        gpointer data);
static void parse_data(char *buf, int len);
static void handle_frame(void);

static void msg_cbk(uint8_t* buf, uint8_t len) {
  fprintf(stderr, "Got msg (%u)\n", len);
  struct ChristineHardwareInput* hi = reinterpret_cast<struct ChristineHardwareInput*>(buf);
  //struct ChristineHardwareInput* hi = (struct ChristineHardwareInput*)buf;
  //fprintf(stderr, "  steering: %f\n", hi->steering_srv);
  //std::printf("  throttle: %f\n", hi->throttle_servo);
}

static int bp_init(const char *serial_device) {
  // initialize serial port
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

  state.seq = 0;
  state.parser.msg_cbk = msg_cbk;
  parser_reset(&state.parser);
  
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
      for (int i=0; i<bytes_read; i++)
	fprintf(stderr, "  %x", buf[i]);
      fprintf(stderr, "\n");
      for (int i=0; i<bytes_read; i++)
	parser_parse(&state.parser, buf[i]);
      printf("read %u\n", bytes_read);
    }
  } else {
    printf("error reading serial: %s\n", _err->message);
    g_error_free(_err);
  }
  return TRUE;
}





static void send() {
  state.seq += 1;
  struct ChristineHardwareOutputMsg hom;
  hom.stx = CHRISTINE_HWI_MSG_STX;
  hom.len = sizeof(hom.data);
  hom.seq = state.seq;
  //  hom.data.seq = state.seq;
  //hom.data.seq2 = 0xFF;
  hom.data.bat_voltage = rc_adc_batt();
  //hom.data.mot_enc = 1;//state.seq;
  hom.ck1 = state.seq;
  hom.ck2 = state.seq;
  gsize bytes_written;
  GError *_err = NULL;
  g_io_channel_write_chars(state.channel, (gchar*)(&hom), sizeof(hom), &bytes_written, &_err);
  if (!_err) {
    //printf("\rwrote %u\n", bytes_written);
  } else {
    printf("error writing serial: %s\n", _err->message);
    g_error_free(_err);
  }
  g_io_channel_flush(state.channel, NULL);
}

gboolean periodic_callback(gpointer data)
{
  send();
}


int main(int argc, char *argv[]) {
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  const char* serial_device = "/dev/ttyS5";
  if (bp_init(serial_device)) {
    return 1;
  }
  state.quit = 0;
  float freq_transmit = 10.;
  g_timeout_add(1000 / freq_transmit, periodic_callback, NULL);
  g_main_loop_run(ml);
  return 0;
}
