#! /usr/bin/env python
# -*- coding: utf-8 -*-


import logging, timeit, math, numpy as np, scipy.signal, scipy.integrate, matplotlib.pyplot as plt, pickle
import sklearn.neural_network
import control
import pdb
import utils
''' plant and controller identification on second order system in controlable cannonical form'''

LOG = logging.getLogger('test_ann_second_order')

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def random_input_vec(time): return np.random.uniform(low=-1.0, high=1.0, size=len(time))
def step_input_vec(time): return [step(t) for t in time]
def sine_input_vec(time): return np.sin(time)
def sawtooth_input_vec(time): return scipy.signal.sawtooth(time)

class LinPlant:

    ''' xk = a0 xkm2 + a1 xkm1 + ukm1 '''
    
    def __init__(self):
        self.a0 =  1.98949898
        self.a1 = -0.98955493


    def set_omega_xi(self, om, xi, dt):
        Ac = np.array([[0, 1],[-om**2, -2*om*xi]])
        print('Ad check\n{}'.format(scipy.linalg.expm(dt*Ac)))
        self.a0 = math.exp(-dt*xi*om)*2*math.cos(dt*om*math.sqrt(1-xi**2))
        self.a1 = -math.exp(-2*dt*xi*om)
        print self.a0, self.a1

    def disc_dyn(self, x_km1, x_km2, u_km1):
        return self.a0*x_km1 + self.a1*x_km2 + u_km1
        
    def sim_with_input_vec(self, time, U, X0=[0, 0]):
        X = np.zeros(len(time))
        X[0] = X0[0]
        X[1] = X0[1]    
        for i in range(2, len(time)):
            X[i] = self.disc_dyn(X[i-1], X[i-2], U[i-1])
        return X

    def __str__(self): return 'ssr: xk = {:.8f}*xkm1 + {:.8f}*xkm2 + ukm1'.format(self.a0, self.a1)  
    

class ANN_PLANT:

    delay = 2
    x_km1, x_km2, u_km1, input_size = range(4)
       
    def __init__(self):
        #self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True, max_iter=500)
        self.ann = sklearn.neural_network.MLPRegressor(activation='identity', verbose=True, hidden_layer_sizes=(8,), random_state=1, max_iter=500, tol=1e-6)

    def make_input(self, X, U):
        _input = np.zeros(( len(X)- self.delay, self.input_size))
        for i in range(self.delay, len(X)):
            _input[i-self.delay, self.x_km1] = X[i-1]
            _input[i-self.delay, self.x_km2] = X[i-2]
            _input[i-self.delay, self.u_km1] = U[i-1]
        return _input
            
    def fit(self, time, X, U):  
        ''' x_kp2 = a0 x_k + a1 x_kp1 + u_kp1 '''
        ''' x_k = a0 x_km2 + a1 x_km1 + u_km1 '''
        print('building training set')
        ann_input, ann_output = self.make_input(X, U), X[self.delay:]
        print('done, now scaling training set')

        self.scaler = sklearn.preprocessing.StandardScaler()
        self.scaler.fit(ann_input)
        scaled_input = self.scaler.transform(ann_input)
        print('fiting set')
        self.ann.fit(scaled_input , ann_output)
        print('done')
        print('score: {}'.format(self.ann.score(scaled_input , ann_output)))
        
    def get(self, Xkm1, Xkm2, Ukm1):
        return self.ann.predict(self.scaler.transform([[Xkm1, Xkm2, Ukm1]]))

    def sim_with_input_vec(self, time, U):
        X =  np.zeros(len(time))
        for i in range(3, len(time)):
            X[i] = self.get(X[i-1], X[i-2], U[i-1])
        return X

    def score_on_input_vec(self, time, U, Xtrue):
        Xest = self.sim_with_input_vec(time, U)
        _input, _output = self.make_input(Xest, U), Xest[self.delay:]
        scaled_input = self.scaler.transform(_input)
        score = self.ann.score(scaled_input , _output)
        print('score: {:f}'.format(score))
        return Xest, score
         
    def compute_ssr(self):
        w1, w2 = self.ann.coefs_
        b1, b2 = self.ann.intercepts_
        #print w1, w2, b1, b2
        a0 = np.dot(w1[0], w2)[0]
        a1 = np.dot(w1[1], w2)[0]
        b = np.dot(w1[2], w2)[0]
        c = (np.dot(b1, w2) + b2)[0]
        #print a0, a1, b, c
        #print self.scaler.scale_ , self.scaler.mean_
        _a0 = (a0)/self.scaler.scale_[0]
        _a1 = (a1)/self.scaler.scale_[1]
        _b = (b)/self.scaler.scale_[2]
        print('est. plant ssr: xk = {:.8f}*xkm1 + {:.8f}*xkm2 + {:.8f}*ukm1'.format(_a0, _a1, _b))
        
         
    def save(self, filename):
        with open(filename, "wb") as f:
            pickle.dump([self.ann, self.scaler], f)

    def load(self, filename):
         with open(filename, "rb") as f:
             self.ann, self.scaler = pickle.load(f)
    
def plot(time, X, U):
    plt.subplot(3,1,1)
    plt.plot(time, X)
    plt.title('X[0]')
    #plt.subplot(3,1,2)
    #plt.plot(time, X[:,1])
    #plt.title('X[1]')
    plt.subplot(3,1,3)
    plt.plot(time, U)
    plt.title('U')
    

def test_id_plant():
    dt=0.005
    plant = LinPlant()
    plant.set_omega_xi(6, 0.5, dt)
    
    ann = ANN_PLANT()
    if 0: # train
        nsamples = int(100*1e3)
        time =  np.arange(0., nsamples*dt, dt)
        print('using {:d} random samples'.format(len(time)))
        #U = np.ones(len(time))
        U = random_input_vec(time)
        time, U = utils.make_random_pulses(dt, nsamples)
        X = plant.sim_with_input_vec(time, U)
        plot(time, X, U)
        ann.fit(time, X, U)
        ann.save('/tmp/sec_order.pkl')
    else: # load
        ann.load('/tmp/sec_order.pkl')

    print('true plant {}'.format(plant))
    ann.compute_ssr()

    plt.figure()
    time =  np.arange(0., 10, dt)
    U2 = step_input_vec(time)
    #U2 = sine_input_vec(time)
    #U2 = sawtooth_input_vec(time)

    X2t = plant.sim_with_input_vec(time, U2)
    #X2e, score = ann.sim_with_input_vec(time, U2), None
    X2e, score = ann.score_on_input_vec(time, U2, X2t)
    plot(time, X2t, U2)
    plt.subplot(3,1,1)
    plt.plot(time, X2e)
    
def main():
    test_id_plant()
    plt.show()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
