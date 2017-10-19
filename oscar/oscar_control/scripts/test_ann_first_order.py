#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging, timeit, math, numpy as np, scipy.signal, scipy.integrate, matplotlib.pyplot as plt
import sklearn.neural_network
import pdb

''' plant and controller identification on first order system '''

LOG = logging.getLogger('test_ann_first_order')


def random(t): return np.random.uniform(low=-1.0, high=1.0, size=1)[0]

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def sin(t): return math.sin(t)

def sawtooth(t): return scipy.signal.sawtooth([t])[0]


def random_input_vec(time):
    return np.random.uniform(low=-1.0, high=1.0, size=len(time))

def step_input_vec(time):
    return [step(t) for t in time]

def sine_input_vec(time):
    return np.sin(time)

def sawtooth_input_vec(time):
    return scipy.signal.sawtooth(time)

class LinPlant:

    def __init__(self, tau=0.1, g=2., dt=0.005):
        self.tau, self.g, self.dt = tau, g, dt
        exp_m_dt_ov_tau = math.exp(-dt/tau)
        self.a, self.b = exp_m_dt_ov_tau, (1-exp_m_dt_ov_tau)*g
        
    def dyn_cont(self, X, t, U, tau=0.1, g=2):
        ''' first order LTI cont dynamics '''
        return -1/tau*(X-g*U)

    def dyn_disc(self, X, U):
        ''' first order LTI disc dynamics '''
        return self.a*X + self.b*U

    def sim_with_input_vec(self, time, U):
        X = np.zeros(len(time))
        X[0] = 0
        for i in range(1, len(time)):
            X[i] = self.dyn_disc(X[i-1], U[i-1])
        return X

    def sim_with_input_fun(self, time, U):
        X, Uvec, SPvec = np.zeros(len(time)),  np.zeros(len(time)), []
        X[0] = 0
        U0, SP0 = U.get(time[0], X[0])
        SPvec.append(SP0)
        for i in range(1, len(time)):
            Uvec[i-1], SP = U.get(time[i], X[i-1])
            SPvec.append(SP)
            X[i] = self.dyn_disc(X[i-1], Uvec[i-1])
        Uvec[-1] = Uvec[-2]
        return X, Uvec, np.array(SPvec)



class NonLinPlant(LinPlant):
    def dyn_disc(self, X, U):
        ''' first order non linear dynamic disc dynamics '''
        return self.a*X + self.b*U**3#math.sin(U)


    
class CtlStateFeedback:
    def __init__(self, tau_obj, tau_plant=0.1, g_plant=2., dt=0.005):
        exp_m_dt_ov_tau_plant = math.exp(-dt/tau_plant)
        a_plant, b_plant = exp_m_dt_ov_tau_plant, (1-exp_m_dt_ov_tau_plant)*g_plant
        exp_m_dt_ov_tau_obj = math.exp(-dt/tau_obj)
        a_obj = exp_m_dt_ov_tau_obj
        self.K = (a_plant - a_obj)/b_plant
        self.H = (1- a_obj)/b_plant

    def get(self, t, x):
        yc = self.yc(t)
        return -self.K*x + self.H*yc, yc


        

class ANN_PLANT:
    def __init__(self):
        #self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True)
        self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True, hidden_layer_sizes=(4,), random_state=1)

    def fit(self, time, X, U):  
        ann_delay = 1
        n_samples = len(time)-ann_delay
        y_km1, u_km1, ann_input_size = range(3)
        ann_input, ann_output = np.zeros((n_samples, ann_input_size)), np.zeros(n_samples)
        for i in range(ann_delay, len(time)):
            ann_output[i-ann_delay] = X[i]
            ann_input[i-ann_delay, y_km1] = X[i-1]
            ann_input[i-ann_delay, u_km1] = U[i-1]

        self.ann.fit(ann_input, ann_output)
        #print self.ann.coefs_

    def get(self, Xkm1, Ukm1):
        return self.ann.predict([[Xkm1, Ukm1]])
    
    def sim_with_input_vec(self, time, U):
        X =  np.zeros(len(time))
        for i in range(1, len(time)):
            X[i] = self.ann.predict([[X[i-1], U[i-1]]])
        return X

   
class ANN_CTL:
    def __init__(self):
        self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True)

    def fit(self, time, X, U, SP):
        ann_delay = 1
        n_samples = len(time)-ann_delay
        x_k, yc_kp1, ann_input_size = range(3)
        ann_input, ann_output = np.zeros((n_samples, ann_input_size)), np.zeros(n_samples)
        for i in range(0, len(time)-ann_delay):
            ann_output[i] = U[i]
            ann_input[i, x_k] = X[i]
            ann_input[i, yc_kp1] = SP[i+1]
   
        self.ann.fit(ann_input, ann_output)
        print self.ann

    def get(self, t, x):
        yc = self.yc(t)
        return self.ann.predict([[x, yc]]), yc



def plot(time, X, U=None):
    plt.subplot(2,1,1)
    plt.plot(time, X)
    plt.title('state')

    if U is not None:
        plt.subplot(2,1,2)
        plt.plot(time, U)
        plt.title('input')


''' testing self made ann predict and recovering state space representation'''
def test_self_made_comp(ann, time, U, plant):
    w1, w2 = ann.ann.coefs_
    b1, b2 = ann.ann.intercepts_
    def my_get(Xkm1, Ukm1):
        ns = Xkm1*w1[0] + Ukm1*w1[1] + b1
        return  np.dot(ns, w2) + b2

    X = np.zeros(len(time))
    for i in range(1, len(time)):
        X[i] = my_get(X[i-1], U[i-1])

    A = np.dot(w1[0], w2)[0]
    B = np.dot(w1[1], w2)[0]
    C = (np.dot(b1, w2) + b2)[0]
    print('identified linear SSR: xk = {} xk + {} uk + {}'.format(A, B, C)) 
    print('true linear ssr:       xk = {} xk + {} uk + {}'.format(plant.a, plant.b, 0))
    
    def ssr(Xkm1, Ukm1): return A*Xkm1 + B*Ukm1 + C
    for i in range(1, len(time)):
        X[i] = ssr(X[i-1], U[i-1])
    
    return X
    
        

        
def test_id_plant():
    dt, tf = 0.005, 100.
    time =  np.arange(0., tf, dt)
    U = random_input_vec(time)
    plant = LinPlant(tau=0.5, g=3., dt=dt)
    X = plant.sim_with_input_vec(time, U)
    plot(time, X, U)
    
    ann = ANN_PLANT()
    ann.fit(time, X, U)
    
    
    dt, tf = 0.005, 10.
    time =  np.arange(0., tf, dt)
    U2 = step_input_vec(time)
    #U2 = sine_input_vec(time)
    X2t = plant.sim_with_input_vec(time, U2)
    X2e = ann.sim_with_input_vec(time, U2)
    X2etest = test_self_made_comp(ann, time, U2, plant)
    
    plt.figure()
    plot(time, X2t, U2)
    plot(time, X2e)
    plot(time, X2etest)
    plt.legend(['real', 'predicted'])
    plt.subplot(2,1,2)
    plt.cla()
    plt.plot(time, X2e-X2t)
    plt.title('prediction error')    


def test_id_ctl():
    dt = 0.005
    plant = LinPlant(tau=3., g=3., dt=dt)
    ctl = CtlStateFeedback(tau_obj=0.25, tau_plant=plant.tau, g_plant=plant.g, dt=dt)
    ctl.yc = random
    time =  np.arange(0., 100., dt)
    X, U, SP = plant.sim_with_input_fun(time, ctl)
    if 0:
        plot(time, X, U)
        plt.subplot(2,1,1)
        plt.plot(time, SP)

    ann = ANN_CTL()
    ann.fit(time, X, U, SP)

    ann.yc = step#sin
    time =  np.arange(0., 10., dt)
    X, U, SP = plant.sim_with_input_fun(time, ann)
    plot(time, X, U)
    
    ctl.yc = step#sin
    X, U, SP = plant.sim_with_input_fun(time, ctl)
    #plt.figure()
    plot(time, X, U)
    


    
def main():
    test_id_plant()
    #test_id_ctl()
    plt.show()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()



