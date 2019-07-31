#!/usr/bin/env python
import time
import numpy as np

import christine_hwi_ext

bbbl = christine_hwi_ext.BBBLink()
bbbl.init()
dt = 0.02
while True:
    _start = time.time()
    steering, throttle = 0.17 + 0.5*np.sin(0.9*time.time()), 0.
    bbbl.send(steering, throttle)
    _end = time.time(); elapsed = _end - _start
    time_to_sleep = dt - elapsed
    if time_to_sleep > 0: time.sleep(time_to_sleep)
#hwi.write()
#time.sleep(1.)
#hwi.shutdown()

