#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math, numpy as np, scipy.integrate, matplotlib.pyplot as plt
import control.matlab

import pdb
# http://www.iaeng.org/publication/IMECS2009/IMECS2009_pp1203-1207.pdf
# http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
import utils




    
# Params
class Param:
    def __init__(self):
        self.Ra =  1.      # armature resistance, (ohm)
        self.La =  0.5     # armature inductance, (H)
        self.J =   0.01    # moment of inertia of the motor rotor and load, (Kg.m2/s2)
        self.B =   0.1     # damping ratio of the mechanical system, (Nms)
        self.Kv =  0.01    # back EMF factor  (V/rad/sec)
        self.Kt =  0.01    # torque factor constant, (Nm/Amp)

# State
# phi: The shaft position, (rad)
# om:  The speed of the shaft and the load (angular velocity), (rad/s)
# ia:  The armature current (Amp)
s_phi, s_om, s_ia = range(3)

# input
# Va: input voltage
# Tl: load torque
i_va, i_tl = range(2)



def dyn_motor(X, t, U, P):
  phi_dot = X[s_om]
  om_dot = 1/P.J*( P.Kt*X[s_ia] - U[i_tl] - P.B*X[s_om])
  ia_dot = 1/P.La*( U[i_va] - P.Kv*X[s_om] - P.Ra*X[s_ia])
  return np.array([phi_dot, om_dot, ia_dot])

def eq_motor(om, tl):
    ie = 1/P.Kt*(tl+P.B*om)
    Ve = P.Ra*ie +P.Kv*om
    return [0, om, ie], [Ve, tl]
    
def get_SS(P):
    A = np.array([[0, 1, 0],[0, -P.B/P.J, P.Kt/P.J],[0, -P.Kv/P.La, -P.Ra/P.La]])
    B = np.array([[0, 0], [0, -1/P.J], [1/P.La, 0]])
    return A, B


def plot(time, X, Xref, U, Xest):
    plt.subplot(3, 2, 1)
    plt.plot(time, X[:,s_phi])
    plt.plot(time, Xref[:,s_phi])
    plt.plot(time, Xest[:,s_phi])
    plt.legend(['sys', 'ref', 'est'])
    plt.title('phi')
    plt.subplot(3, 2, 3)
    plt.plot(time, X[:,s_om])
    plt.plot(time, Xref[:,s_om])
    plt.plot(time, Xest[:,s_om])
    plt.legend(['sys', 'ref', 'est'])
    plt.title('omega')
    plt.subplot(3, 2, 5)
    plt.plot(time, X[:,s_ia])
    plt.title('i a')

    plt.subplot(3, 2, 2)
    plt.plot(time, X[:,s_phi]-Xref[:,s_phi])
    plt.title('err tracking phi')
    
    plt.subplot(3, 2, 4)
    plt.plot(time, X[:,s_om]-Xref[:,s_om])
    plt.title('err tracking omega')
    plt.subplot(3, 2, 6)
    plt.plot(time, U[:,0])
    plt.title('U')

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def sim(ref, ctl, est, Pmot, Pctl, X0=[0,0,0], time=np.arange(0., 3., 0.001), Xref0=[0, 0, 0, 0], sp=step, pert=0.):
    X, Xref, U = np.zeros((len(time), 3)), np.zeros((len(time), 4)), np.zeros((len(time), 2))
    X[0] = X0
    Xref[0] = Xref0; ref.X = np.array(Xref0)
    Xest = np.zeros((len(time), 3))
    est.X[0] = 0.25
    for i in range(1, len(time)):
        dt = time[i]-time[i-1]
        Xref[i] = ref.run(dt, sp(time[i]))
        phi_meas, phidd_ref = X[i-1][0], Xref[i][2]
        Xest[i-1] = est.update(dt, phi_meas, phidd_ref)
        U[i-1] = ctl.get(time[i-1], X[i-1], Xref[i])
        U[i-1,1] = pert
        unused, X[i] = scipy.integrate.odeint(dyn_motor, X[i-1], [time[i-1], time[i]], args=(U[i-1], Pmot ))
    U[-1] = U[-2]; Xest[-1] = Xest[-2]
    return time, X, Xref, U, Xest


# /media/mint17/home/poine/home_stripe/work/python-aerospace-toolbox/python-aerospace-toolbox/src/test/test_discrete_time_2.py
class Estimator:
    def __init__(self, om=3, xi=0.9, dt=0.001):
        Ac = np.array([[0, 1],[-om**2, -2*om*xi]])
        print('Ad check\n{}'.format(scipy.linalg.expm(dt*Ac)))
        ceva, ceve = np.linalg.eig(Ac)
        print('AdPolesCheck {}'.format(np.exp(ceva*dt)))

        self.X = np.zeros(3)
        # caracteristic polynomyal coefficients of equivalent disc time
        a1 = -math.exp(-dt*xi*om)*2*math.cos(dt*om*math.sqrt(1-xi**2))
        a0 = math.exp(-2*dt*xi*om)
        l1 = 2 + a1
        l2 = 1/dt*(a0+l1-1)
        self.L = np.array([[l1],[l2]])
        Acl = np.array([[1, dt],[0, 1]]) - np.dot(self.L, np.array([[1, 0]]))
        eva, eve = np.linalg.eig(Acl)
        print('Estimator Acl poles {}'.format(eva))
        
        
    def update(self, dt, meas_phi, phidd_ref):
        self.X[0] += self.X[1]*dt+phidd_ref*dt**2/2 # predict
        self.X[1] += phidd_ref*dt                   # predict
        #pdb.set_trace()
        self.X[:2] += (meas_phi-self.X[0]) * self.L.squeeze()
        return self.X
        

class ControllerNLI:
    def __init__(self, P, ome=6, xie=0.7):
        print('desired closed loop poles {}'.format(np.array(utils.get_lambdas(ome, xie))))
        Kphi = -1.
        Kom = P.Kv - P.La*P.B**2/P.J/P.Kt - ome**2*P.J*P.La/P.Kt + 2*ome*xie*P.B*P.La/P.Kt
        Ki = P.Ra + P.B*P.La/P.J - 2*ome*xie*P.La
        self.K = np.array([[-Kphi, -Kom, -Ki]])
        A, B = get_SS(P)
        Acl = A - np.dot(B[:,0].reshape(3,1), self.K)
        eva_cl, eve_cl = np.linalg.eig(Acl)
        print('achieved closed loop poles at {}'.format(eva_cl))
        
        Hphi = -Kphi
        Hom = P.J*P.La/P.Kt*ome**2
        Homd = P.J*P.La/P.Kt*2*xie*ome
        Homdd = P.J*P.La/P.Kt
        self.H = np.array([[Hphi, Hom, Homd, Homdd]])

    def get(self, t, X, Xr):
        U = -np.dot(self.K, X) + np.dot(self.H, Xr)
        return [U, 0]



    
if __name__ == "__main__":
    Pmot = Param()
    Pctl = Param()
    if 0:
        Pctl.Ra *= 0.9
        Pctl.La *= 0.9
        Pctl.J  *= 1.2
        Pctl.B  *= 1.2
        Pctl.Kv  *= 1.2
        Pctl.Kt  *= 1.2
 
    ref = utils.MotorRef(omega=4., xi=0.9)
    print('ref poles {}'.format(ref.poles()))
    time = np.arange(0., 10., 0.001)
    ctl = ControllerNLI(Pctl)
    est = Estimator()
    time, X , Xref, U, Xest = sim(ref, ctl, est, Pmot, Pctl, X0=[0, 0, 0], time=time, Xref0=[0, 0., 0, 0], sp=step, pert=0.)
    plot(time, X, Xref, U, Xest)
    plt.show()
