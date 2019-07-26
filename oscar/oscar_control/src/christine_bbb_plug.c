#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <glib.h>

#include <robotcontrol.h>

#include "oscar_control/christine_serial_port.h"
#include "oscar_control/christine_hw.h"

struct State {
  struct SerialPort* sp;
  int sp_fd;
  GIOChannel *channel;
  int quit;
};

static struct State state;

static gboolean on_serial_data_received(GIOChannel *source,
                                        GIOCondition condition,
                                        gpointer data);
static void parse_data(char *buf, int len);
static void handle_frame(void);

static int bp_init(const char *serial_device) {
  // initialize serial port
  state.sp = serial_port_new();
  int ret = serial_port_open_raw(state.sp, serial_device,  B115200);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    return 1;
  }
  state.channel = g_io_channel_unix_new(state.sp->fd);
  g_io_channel_set_encoding(state.channel, NULL, NULL);
  g_io_add_watch(state.channel, G_IO_IN , on_serial_data_received, NULL);

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
      //parse_data(buf, bytes_read);
      printf("read %d\n", bytes_read);
    }
  } else {
    printf("error reading serial: %s\n", _err->message);
    g_error_free(_err);
  }
  return TRUE;
}





static void send() {
  const gchar buf[] = {1, 2, 3, '\n'};
  struct ChristineHardwareOutput ho;
  ho.bat_voltage = rc_adc_batt();
  
  gsize bytes_written;
  GError *_err = NULL;
  g_io_channel_write_chars(state.channel, (gchar*)(&ho), sizeof(ho), &bytes_written, &_err);
  if (!_err) {
    printf("wrote %d\n", bytes_written);
  } else {
    printf("error reading serial: %s\n", _err->message);
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
  /* while (!state.quit) { */
  /*   usleep(100000); */
  /*   send(); */
  /* } */
  
  return 0;
}
