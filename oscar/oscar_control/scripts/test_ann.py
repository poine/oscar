#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math, numpy as np, scipy.integrate, matplotlib.pyplot as plt
import sklearn.neural_network
import pdb

import test_motor as mot, utils

''' identify the DC motor with a neural network '''


''' generate a dataset by integrating the motor dynamic on the input '''
def generate_dataset(Pmot, time, U):
    X = np.zeros((len(time), 3))
    X0 = [0, 0, 0]
    X[0] = X0
    for i in range(len(time)-1):
        _unused, X[i+1] = scipy.integrate.odeint(mot.dyn_motor, X[i], [time[i], time[i+1]], args=([U[i], 0], Pmot ))
    return X

''' simulate time delay '''
def prepare_dataset(time, X, U):
    ann_delay = 2
    n_samples = len(time)-ann_delay
    y_km2, y_km1, u_km2, u_km1, u_k, ann_input_size = range(6)

    ann_input, ann_output = np.zeros((n_samples, ann_input_size)), np.zeros(n_samples)
    for i in range(n_samples):
        ann_output[i] = X[i+ann_delay, 0]
        ann_input[i, y_km2:y_km1+1] = X[i+ann_delay-2:i+ann_delay, 0]
        ann_input[i, u_km2:u_k+1] = U[i+ann_delay-2:i+ann_delay+1]

    return ann_input, ann_output


def train_ann(ann_input, ann_output):
    ann = sklearn.neural_network.MLPRegressor(activation='identity')
    ann.fit(ann_input, ann_output)  
    return ann

def save_ann(ann, filename):
    sklearn.externals.joblib.dump(ann, filename) 

def load_ann(filename):
    return sklearn.externals.joblib.load(filename) 

def run_ann(ann, ann_input):
    Xest = ann.predict(ann_input)
    return Xest


def plot(time, X, U):
    plt.subplot(4, 1, 1)
    plt.plot(time, X[:,mot.s_phi])
    #plt.legend(['sys', 'ref', 'est'])
    plt.title('phi')
    plt.subplot(4, 1, 2)
    plt.plot(time, X[:,mot.s_om])
    #plt.legend(['sys', 'ref', 'est'])
    plt.title('omega')
    plt.subplot(4, 1, 3)
    plt.plot(time, X[:,mot.s_ia])
    plt.title('i a')
    plt.subplot(4, 1, 4)
    plt.plot(time, U)
    plt.title('U')      

if __name__ == "__main__":
    Pmot = mot.Param()
    dt = 0.005
    if 1:
        n_training = int(100e3)
        if 1:
            time_training =  np.arange(0, n_training*dt, dt)
            U_training = np.random.uniform(low=-1.0, high=1.0, size=n_training)
        else:
            time_training, U_training = utils.make_random_pulses(dt, n_training)
            
        X_training = generate_dataset(Pmot, time_training, U_training)
        plot(time_training, X_training, U_training)
        
        ann_input, ann_output = prepare_dataset(time_training, X_training, U_training)
        ann = train_ann(ann_input, ann_output)
        save_ann(ann, '/tmp/filename.pkl')
    else:
        ann =  load_ann('/tmp/filename.pkl')
    

    n_test = 10000
    time_test =  np.arange(0, n_test*dt, dt)
    U_test = scipy.signal.square(0.5*time_test)
    X_test_truth = generate_dataset(Pmot, time_test, U_test)
    ann_input, ann_output = prepare_dataset(time_test, X_test_truth, U_test)
    X_test_est = run_ann(ann, ann_input)

    plt.figure()
    plot(time_test, X_test_truth, U_test)
    plt.subplot(4, 1, 1)
    plt.plot(time_test[2:], X_test_est)
    plt.legend(['real', 'predicted'])
    om_est = np.gradient(X_test_est)/dt
    plt.subplot(4, 1, 2)
    plt.plot(time_test[2:], om_est)
    #pdb.set_trace()
    
    plt.show()
