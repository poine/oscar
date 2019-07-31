#!/usr/bin/env python
import sys, time
import numpy as np

import christine_hwi_ext



def main(args):
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    dt = 0.02
    while True:
        _start = time.time()
        #steering, throttle = 0.17 + 0.5*np.sin(0.9*_start), 0.
        steering, throttle = bbbl.get_dsm()
        bbbl.send(steering, throttle)
        _end = time.time(); elapsed = _end - _start
        time_to_sleep = dt - elapsed
        if time_to_sleep > 0: time.sleep(time_to_sleep)

if __name__ == '__main__':
    main(sys.argv)
