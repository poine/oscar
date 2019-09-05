#!/usr/bin/env python
import sys, time
import numpy as np

import christine_hwi_ext

def send_inputs(bbbl, input_fun, period=0.02):
    while True:
        _start = time.time()
        steering, mot_sp = input_fun(_start)
        print('sending: steering {:.2f} %% mot_sp {:.1f} cps'.format(steering, mot_sp))
        bbbl.send(steering, mot_sp)
        mot_pos, mot_vel = bbbl.get_motor()
        print('mot pos {} vel {}'.format(mot_pos, mot_vel))
        _end = time.time(); elapsed = _end - _start; nap_time = period - elapsed
        if nap_time > 0: time.sleep(nap_time)


def pass_dsm(bbbl, period=0.02):
    def _input(t): return bbbl.get_dsm();
    return send_inputs(bbbl, _input, period=0.02)

def _sin(a, b, om, t): return b + a*np.sin(om*t)

def steering_sin(bbbl, om=1., b=0.17, a=0.5):
    def _input(t): return _sin(a, b, om, t), 0.
    return send_inputs(bbbl, _input, period=0.02)

def throttle_sin(bbbl, om=1., b=0.02, a=0.0):
    def _input(t): return 0., _sin(a, b, om, t)
    return send_inputs(bbbl, _input, period=0.02)
        

def main(args):
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    #pass_dsm(bbbl)
    #steering_sin(bbbl)
    #throttle_sin(bbbl)
    throttle_sin(bbbl, b=0.0, a=60.)
    
if __name__ == '__main__':
    main(sys.argv)
