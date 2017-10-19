#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt
import sklearn.neural_network
import pdb

LOG = logging.getLogger('test_ann_ctl')

import test_motor as mot, utils


''' identify the DC motor controller with a neural network '''

def sim_motor(Pmot, time, sp, mode='run', filename='/tmp/motor_nli_sim.npz'):
    if mode=='run':
        LOG.info(' Simulating motor with NLI controller ({} steps)'.format(len(time)))
        _start = timeit.default_timer()
        ref = utils.MotorRef(omega=4., xi=0.9)
        ctl = mot.ControllerNLI(Pmot)
        X, Xref, U = np.zeros((len(time), 3)), np.zeros((len(time), 4)), np.zeros((len(time), 2))
        for i in range(1, len(time)):
            Xref[i] = ref.run(time[i]-time[i-1], sp[i])
            U[i-1] = ctl.get(time[i-1], X[i-1], Xref[i])
            unused, X[i] = scipy.integrate.odeint(mot.dyn_motor, X[i-1], [time[i-1], time[i]], args=(U[i-1], Pmot ))
        LOG.info('  done... (took {:.1f}s)'.format(timeit.default_timer()-_start))
        np.savez(filename, time=time, sp=sp, X=X, Xref=Xref, U=U)
        LOG.info('  saved to {}'.format(filename))
    else:
        LOG.info(' Loading motor simulation from {}'.format(filename)) 
        data =  np.load(filename)
        time, sp, X, Xref, U = [data[w] for w in ['time', 'sp', 'X', 'Xref', 'U']]
        LOG.info('  found {} samples'.format(len(time)))
    return time, sp, X, Xref, U

def plot_sim(time, sp, X, Xref, U):
    plt.subplot(2, 1, 1)
    plt.plot(time, sp)
    plt.plot(time, X[:,1])
    plt.plot(time, Xref[:,1])
    plt.title('omega')
    plt.legend(['sp', 'sys', 'ref'])
    
    plt.subplot(2, 1, 2)
    plt.plot(time, U[:,0])
    plt.title('U')
    
def prepare_dataset(time, X, Xref, U):
    ann_delay = 3
    n_samples = len(time)-ann_delay
    y_km2, y_km1, y_k, yr_km3, yr_km2, yr_km1, yr_k, ann_input_size = range(8)

    ann_input, ann_output = np.zeros((n_samples, ann_input_size)), np.zeros(n_samples)
    for i in range(ann_delay, len(time)-3):
        ann_output[i] = U[i, 0]
        ann_input[i, y_km2:y_k+1] = X[i-2:i+1, 0]
        ann_input[i, yr_km3:yr_k+1] = Xref[i-3:i+1, 0]

    return ann_input, ann_output

def train_ann(time, X, Xref, U, filename='ann_ctl.pkl'):
    LOG.info(' Training ann on {} samples'.format(len(time))) 
    _start = timeit.default_timer()
    ann_input, ann_output = prepare_dataset(time, X, Xref, U)
    ann = sklearn.neural_network.MLPRegressor(activation='identity')
    ann.fit(ann_input, ann_output)
    LOG.info('  done (took {:.1f}s)'.format(timeit.default_timer()-_start))
    sklearn.externals.joblib.dump(ann, filename) 
    LOG.info('  saved to {}'.format(filename))
    return ann



def test_ann(ann, dt, Pmot):
    n_test = int(10e3)
    time =  np.arange(0, n_test*dt, dt)
    sp = scipy.signal.square(0.5*time)
    ref = utils.MotorRef(omega=4., xi=0.9)
    X, Xr, U = np.zeros((len(time), 3)), np.zeros((len(time), 4)), np.zeros((len(time), 2))
    for i in range(1, len(time)):
        Xr[i] = ref.run(time[i]-time[i-1], sp[i])
        U[i-1] = 0 if i<3 else ann.predict([[X[i-3,0], X[i-2,0], X[i-1,0], Xr[i-4,0], Xr[i-3,0], Xr[i-2,0], Xr[i-1,0]]]) 
        unused, X[i] = scipy.integrate.odeint(mot.dyn_motor, X[i-1], [time[i-1], time[i]], args=(U[i-1], Pmot ))
    return time, sp, X, Xr, U

        
def main():
    dt, n_training = 0.005, int(500e3)
    
    Pmot = mot.Param()

    sim_filename = '/tmp/motor_nli_sim.npz'
    if 0: # run simulation
        time, sp = utils.make_random_pulses(dt, n_training, min_nperiod=1, max_nperiod=20)
        time, sp, X, Xref, U = sim_motor(Pmot, time, sp, mode='run', filename=sim_filename)
        plot_sim(time, sp, X, Xref, U)
        #return
    else: # loads pre-made simulation
        time, sp, X, Xref, U = sim_motor(None, None, None, mode='load', filename=sim_filename)
    
    ann_filename = '/tmp/ann_ctl.pkl'
    if 1:
        ann = train_ann(time, X, Xref, U, ann_filename)
    else:
        LOG.info(' Loading ann from {}'.format(ann_filename)) 
        ann = sklearn.externals.joblib.load(ann_filename) 
        
    time, sp, X, Xr, U = test_ann(ann, dt, Pmot)
    plt.figure()
    plot_sim(time, sp, X, Xr, U)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
    plt.show()
