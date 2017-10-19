#! /usr/bin/env python
# -*- coding: utf-8 -*-


import logging, timeit, math, numpy as np, scipy.signal, scipy.integrate, matplotlib.pyplot as plt
import sklearn.neural_network
import control
import pdb

''' plant and controller identification on second order system '''

LOG = logging.getLogger('test_ann_second_order')

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def random_input_vec(time): return np.random.uniform(low=-1.0, high=1.0, size=len(time))
def step_input_vec(time): return [step(t) for t in time]

class LinPlant:

    ''' xdd = -2 xi omega xd -2 omega^2 (x-g.u) '''
    
    def __init__(self, om=1., xi=0.9, g=1., dt=0.005):
        self.Ac = np.array([[0, 1],[-om**2, -2*xi*om]])
        self.Bc = np.array([[0],[g*om**2]])
        print('Ac\n{}\nBc\n{}'.format(self.Ac, self.Bc))
        self.Ad = scipy.linalg.expm(dt*self.Ac)
        tmp = np.dot(np.linalg.inv(self.Ac), self.Ad-np.eye(2))
        self.Bd = np.dot(tmp, self.Bc)
        print('Ad\n{}\nBd\n{}'.format(self.Ad, self.Bd))

        self.S = control.ss(self.Ad, self.Bd, [1, 0], [0])
        #pdb.set_trace()
        #(self.Acc, self.Bcc, self.Ccc, self.Dcc),
        self.Scc, self.Mcc = control.canonical_form(self.S, form='reachable')
        print self.Scc
        

    def disc_dyn(self, X, U):
        ''' discrete time dynamics '''
        #pdb.set_trace()
        return np.dot(self.Ad, X) + (self.Bd*U).squeeze()
        
    def sim_with_input_vec(self, time, U, X0=[0, 0]):
        X = np.zeros((len(time), 2))
        X[0] = X0
        for i in range(1, len(time)):
            X[i] = self.disc_dyn(X[i-1], U[i-1])
        return X

class ANN_PLANT:
    def __init__(self):
        self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True)
        #self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True, hidden_layer_sizes=(8,), random_state=1)

    def fit(self, time, X, U):  
        ''' x_kp2 = a0 x_k + a1 x_kp1 + u_kp1 '''
        ''' x_k = a0 x_km2 + a1 x_km1 + u_km1 '''
        ann_delay = 3
        n_samples = len(time)-ann_delay
        x_km1, x_km2, x_km3, u_km1, u_km2, ann_input_size = range(6)
        ann_input, ann_output = np.zeros((n_samples, ann_input_size)), np.zeros(n_samples)
        for i in range(ann_delay, len(time)):
            ann_output[i-ann_delay] = X[i,0]
            ann_input[i-ann_delay, x_km1] = X[i-1,0]
            ann_input[i-ann_delay, x_km2] = X[i-2,0]
            ann_input[i-ann_delay, x_km3] = X[i-3,0]
            ann_input[i-ann_delay, u_km1] = U[i-1]
            ann_input[i-ann_delay, u_km2] = U[i-2]

        self.ann.fit(ann_input, ann_output)

    def get(self, Xkm1, Xkm2, Xkm3, Ukm1, Ukm2):
        return self.ann.predict([[Xkm1, Xkm2, Xkm3, Ukm1, Ukm2]])

    def sim_with_input_vec(self, time, U):
        X =  np.zeros(len(time))
        for i in range(3, len(time)):
            X[i] = self.get(X[i-1], X[i-2], X[i-3], U[i-1], U[i-2])
        return X

        
    
def plot(time, X, U):
    plt.subplot(3,1,1)
    plt.plot(time, X[:,0])
    plt.title('X[0]')
    plt.subplot(3,1,2)
    plt.plot(time, X[:,1])
    plt.title('X[1]')
    plt.subplot(3,1,3)
    plt.plot(time, U)
    plt.title('U')
    

def test_id_plant():
    dt=0.005
    plant = LinPlant(om=1.5, xi=0.7, g=1., dt=dt)

    nsamples = int(100*1e3)
    time =  np.arange(0., nsamples*dt, dt)
    print len(time)
    #U = np.ones(len(time))
    U = random_input_vec(time)
    X = plant.sim_with_input_vec(time, U)
    #plot(time, X, U)
    ann = ANN_PLANT()
    ann.fit(time, X, U)

    time =  np.arange(0., 10, dt)
    U2 = step_input_vec(time)

    X2t = plant.sim_with_input_vec(time, U2)
    X2e = ann.sim_with_input_vec(time, U2)
    plot(time, X2t, U2)
    plt.subplot(3,1,1)
    plt.plot(time, X2e)
    
def main():
    test_id_plant()
    plt.show()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
