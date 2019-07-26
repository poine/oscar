//
//  runs on BeagleBone
//  reads DSM and drives servos
//


#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>

static int running;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{

	uint64_t dsm_nanos=0;
	int frequency_hz = 50; // default 50hz frequency to send pulses
	int radio_ch=2;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	// read adc to make sure battery is connected
	/* if(rc_adc_init()){ */
	/* 	fprintf(stderr,"ERROR: failed to run rc_adc_init()\n"); */
	/* 	return -1; */
	/* } */
	/* if(rc_adc_batt()<6.0){ */
	/* 	fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n"); */
	/* 	return -1; */
	/* } */
	/* rc_adc_cleanup(); */

	// initialize PRU
	if(rc_servo_init()) return -1;

	if(rc_dsm_init()==-1) return -1;
	printf("Waiting for first DSM packet\n");
	fflush(stdout);
	while(rc_dsm_is_new_data()==0){
	  if(running==0) return 0;
	  rc_usleep(50000);
	}

	// turn on power
	printf("Turning On 6V Servo Power Rail\n");
	rc_servo_power_rail_en(1);


	// Main loop runs at frequency_hz
	while(running){
	  dsm_nanos = rc_dsm_nanos_since_last_packet();
	  if(dsm_nanos > 200000000){
	    rc_servo_send_pulse_normalized(1, 0.);
	    rc_servo_send_pulse_normalized(2, 0.);
	    printf("\rSeconds since last DSM packet: %.2f              ", dsm_nanos/1000000000.0);
	  }
	  else{
	    double rc_steering, rc_throttle;
	    rc_steering = rc_dsm_ch_normalized(2);
	    rc_throttle = rc_dsm_ch_normalized(3);
	    const double a0_st = 0.187;
	    double srv_steering, srv_throttle;
	    srv_steering = rc_steering + a0_st;
	    srv_throttle = rc_throttle;
	    rc_servo_send_pulse_normalized(1, srv_steering);
	    rc_servo_send_pulse_normalized(2, srv_throttle);

	    printf("RC: st: %.3f th: %.3f SRV st: %.3f th: %.3f\n", rc_steering, rc_throttle, srv_steering, srv_throttle);

	  }
	  
	  // sleep roughly enough to maintain frequency_hz
	  rc_usleep(1000000/frequency_hz);
	}
	
	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	rc_dsm_cleanup();
	return 0;
}

