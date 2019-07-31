#!/usr/bin/env python
import sys, time
import numpy as np
import matplotlib.pyplot as plt

import christine_hwi_ext


def measure(filename='/tmp/foo.npz'):
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    dt, n_samples = 0.02, 500
    _time, throttle, mot_vel = np.zeros(n_samples), np.zeros(n_samples), np.zeros(n_samples) 
    for i in range(n_samples):
        _start = _time[i] = time.time()
        steering, throttle[i] = 0, 0.06 + 0.01*np.sin(0.9*time.time())
        bbbl.send(steering, throttle[i])
        mot_pos, mot_vel[i] = bbbl.get_motor()
        _end = time.time(); elapsed = _end - _start
        time_to_sleep = dt - elapsed
        if time_to_sleep > 0: time.sleep(time_to_sleep)
    np.savez(filename, time=_time, throttle=throttle, mot_vel=mot_vel)
    return time, throttle, mot_vel

def plot(filename='/tmp/foo.npz'):
    data =  np.load(filename)
    _time, throttle, mot_vel = [data[w] for w in ['time', 'throttle', 'mot_vel']]
    #LOG.info('  found {} samples'.format(len(time)))

    plt.plot(_time, mot_vel)
    plt.show()

def main(args):
    measure()
    #plot()

if __name__ == '__main__':
    main(sys.argv)

